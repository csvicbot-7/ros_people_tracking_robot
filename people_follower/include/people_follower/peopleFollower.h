// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
// SVM
#include "svm.h"

typedef struct feature {
	/*** for visualization ***/
	Eigen::Vector4f centroid;
	Eigen::Vector4f min;
	Eigen::Vector4f max;
	/*** for classification ***/
	int numberPoints;
	float minDistance;
	Eigen::Matrix3f covariance3d;
	Eigen::Matrix3f moment3d;
	// float partial_covariance_2d[9];
	// float histogram_main_2d[98];
	// float histogram_second_2d[45];
	float slice[20];
	float intensity[27];
} Feature;

static const int FEATURE_SIZE = 61;

const int m_nestedRegions = 4;
int m_zone[m_nestedRegions] = {2,3,3,3};

class PeopleFollower {
private:
	/*** Publishers and Subscribers ***/
	ros::NodeHandle m_nodeHandle;
	ros::NodeHandle m_privateNh;
	ros::Subscriber m_pointCloudSub;
	ros::Subscriber m_startTrackingSub;
	ros::Publisher m_poseArrayPub;
	ros::Publisher m_markerArrayPub;
	ros::Publisher m_markerHumanGoalPub;
	ros::Publisher m_humanPosePub;
	ros::Publisher m_cropBoxPub;
	ros::Publisher m_humanGoalPub;
	ros::Publisher m_clusterCloudPub;
	ros::Publisher m_lostPub;
	
	std::string m_frameId;
	float m_distX;
	float m_distY;
	float m_ZlimitMin;
	float m_ZlimitMax;
	int m_clusterSizeMin;
	int m_clusterSizeMax;
	
	std::vector<Feature> m_features;
	std::string m_modelFileName;
	std::string m_rangeFileName;
	struct svm_node *m_svmNode;
	struct svm_model *m_svmModel;
	bool m_useSvmModel;
	bool m_isProbabilityModel;
	float m_svmScaleRange[FEATURE_SIZE][2];
	float m_xLower;
	float m_xUpper;
	float m_humanProbability;
	bool m_humanSizeLimit;

	bool m_startTracking;
	bool m_firstIter;
	pcl::PointXYZ m_humanPos;
  pcl::PointXYZ m_lastHumanPos;
	geometry_msgs::Point m_humanGoal;
	std_msgs::Bool m_lost;
	visualization_msgs::Marker m_humanMarker;

	void cropBox(const sensor_msgs::PointCloud2ConstPtr& cloudMsg,
							pcl::PointCloud<pcl::PointXYZI>::Ptr outCloudPtr);
	
	pcl::PointXYZ getClosestPose(pcl::PointXYZ &p, geometry_msgs::PoseArray &poseArray);
	
	double euclideanDistance(pcl::PointXYZ pt1, pcl::PointXYZ pt2) {
    return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
  }
	
	//Feature functions

	void computeMomentOfInertiaTensorNormalized(pcl::PointCloud<pcl::PointXYZI> &pc, Eigen::Matrix3f &moment3d);
	void computeSlice(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, int n, float *slice);
	void computeIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, int bins, float *intensity);
	
	
public:
	PeopleFollower();
	~PeopleFollower();
	
	void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc2);
	void trackingCallback(const std_msgs::Bool &start);
	
	void extractCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr pc,
											pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud);
	void extractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Feature &f,
											Eigen::Vector4f &min, Eigen::Vector4f &max, Eigen::Vector4f &centroid);
	void saveFeature(Feature &f, struct svm_node *x);
	geometry_msgs::PoseArray classify();
};