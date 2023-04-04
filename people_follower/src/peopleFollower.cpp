// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
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

#include "people_follower/peopleFollower.h"

PeopleFollower::PeopleFollower() : m_privateNh ("~") {
	m_pointCloudSub = m_nodeHandle.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 1, &PeopleFollower::pointCloudCallback, this);
	m_startTrackingSub = m_nodeHandle.subscribe("/people_follower/start", 1, &PeopleFollower::trackingCallback, this);
	
	m_markerArrayPub = m_nodeHandle.advertise<visualization_msgs::MarkerArray>("/people_follower/markers", 100);
	m_markerHumanGoalPub = m_nodeHandle.advertise<visualization_msgs::Marker>("/people_follower/markerHuman", 100);
	m_poseArrayPub = m_nodeHandle.advertise<geometry_msgs::PoseArray>("/people_follower/poses", 100);
	m_cropBoxPub = m_nodeHandle.advertise<sensor_msgs::PointCloud2>("/people_follower/cropBox", 100);
	m_humanGoalPub = m_nodeHandle.advertise<geometry_msgs::Point>("/people_follower/humanGoal", 100);
	m_clusterCloudPub = m_nodeHandle.advertise<sensor_msgs::PointCloud2>("/people_follower/clusterCloud", 100);
	m_lostPub = m_nodeHandle.advertise<std_msgs::Bool>("/people_follower/lost",1);
	
	/*** Parameters ***/
	
	m_privateNh.param<std::string>("frame_id", m_frameId, "os_sensor");
	m_privateNh.param<float>("distX", m_distX, 6.0f);
	m_privateNh.param<float>("distY", m_distY, 2.0f);
	m_privateNh.param<float>("z_limit_min", m_ZlimitMin, -0.6);
	m_privateNh.param<float>("z_limit_max", m_ZlimitMax, 1.2);
	m_privateNh.param<int>("cluster_size_min", m_clusterSizeMin, 5);
	m_privateNh.param<int>("cluster_size_max", m_clusterSizeMax, 30000);
	m_privateNh.param<float>("human_probability", m_humanProbability, 0.70);
	m_privateNh.param<bool>("human_size_limit", m_humanSizeLimit, true);
	
	m_humanMarker.id = 0;
	m_humanMarker.type = visualization_msgs::Marker::SPHERE;
	m_humanMarker.action = visualization_msgs::Marker::ADD;
	m_humanMarker.ns = "follower";
	m_humanMarker.scale.x = 0.2;
	m_humanMarker.scale.y = 0.2;
	m_humanMarker.scale.z = 0.2;
	m_humanMarker.color.a = 1.0;
	m_humanMarker.color.r = 1.0;
	m_humanMarker.color.g = 0.0;
	m_humanMarker.color.b = 0.0;

	/****** load a pre-trained svm model ******/
	m_privateNh.param<std::string>("model_file_name", m_modelFileName, "");
	m_privateNh.param<std::string>("range_file_name", m_rangeFileName, "");
	
	m_useSvmModel = false;
	if((m_svmModel = svm_load_model(m_modelFileName.c_str())) == NULL) {
		ROS_WARN("[object3d detector] can not load SVM model, use model-free detection.");
	} else {
		ROS_INFO("[object3d detector] load SVM model from '%s'.", m_modelFileName.c_str());
		m_isProbabilityModel = svm_check_probability_model(m_svmModel)?true:false;
		m_svmNode = (struct svm_node *)malloc((FEATURE_SIZE+1)*sizeof(struct svm_node)); // 1 more size for end index (-1)
		
		// load range file
		std::fstream rangeFile;
		rangeFile.open(m_rangeFileName.c_str(), std::fstream::in);
		if(!rangeFile.is_open()) {
			ROS_WARN("[object3d detector] can not load range file, use model-free detection.");
		} else {
			ROS_INFO("[object3d detector] load SVM range from '%s'.", m_rangeFileName.c_str());
			std::string line;
			std::vector<std::string> params;
			std::getline(rangeFile, line);
			std::getline(rangeFile, line);
			boost::split(params, line, boost::is_any_of(" "));
			m_xLower = atof(params[0].c_str());
			m_xUpper = atof(params[1].c_str());
			int i = 0;
			while(std::getline(rangeFile, line)) {
				boost::split(params, line, boost::is_any_of(" "));
				m_svmScaleRange[i][0] = atof(params[1].c_str());
				m_svmScaleRange[i][1] = atof(params[2].c_str());
				i++;
				//std::cerr << i << " " <<  m_svmScaleRange[i][0] << " " << m_svmScaleRange[i][1] << std::endl;
			}
			m_useSvmModel = true;
		}
	}
}

PeopleFollower::~PeopleFollower() {
	if(m_useSvmModel) {
		svm_free_and_destroy_model(&m_svmModel);
		free(m_svmNode);
	}
}

void PeopleFollower::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud) {
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>);
	
	cropBox(cloud, cloudPtr);
	// cloudPtr->header.frame_id = cloud->header.frame_id;
	// sensor_msgs::PointCloud2 rosCloud;
  // pcl::toROSMsg(*cloudPtr, rosCloud);
  //m_cropBoxPub.publish(rosCloud);

	extractCluster(cloudPtr, clusterCloud);
	// clusterCloud->header.frame_id = cloud->header.frame_id;
	// sensor_msgs::PointCloud2 rosCloud2;
	// pcl::toROSMsg(*clusterCloud, rosCloud2);
	// m_clusterCloudPub.publish(rosCloud2);
	auto humanPoseArray  = classify();

	if (m_startTracking){

		m_humanMarker.pose.position.x = m_lastHumanPos.x;
		m_humanMarker.pose.position.y = m_lastHumanPos.y;
		m_humanMarker.pose.position.z = 0.0;
		m_humanMarker.header = cloud->header;
		m_markerHumanGoalPub.publish(m_humanMarker);
		
		auto min_distance = DBL_MAX;
		double dist;
		
		if (m_firstIter){
			m_humanGoal.x = m_humanPos.x;
			m_humanGoal.y = m_humanPos.y;
			m_humanGoal.z = m_humanPos.z;
			m_humanGoalPub.publish(m_humanGoal);
			m_firstIter = false;
			m_lastHumanPos = m_humanPos;
		}
		else {
			if (humanPoseArray.poses.size() == 0) {
				ROS_WARN("Human goal being tracked was lost!");
				m_humanGoal.x = m_humanPos.x;
				m_humanGoal.y = m_humanPos.y;
				m_humanGoal.z = m_humanPos.z;
				m_humanGoalPub.publish(m_humanGoal);
				m_lost.data = true;
				m_lastHumanPos = m_humanPos;
			} else {
				m_humanPos = getClosestPose(m_lastHumanPos, humanPoseArray);
				if (euclideanDistance(m_humanPos, m_lastHumanPos) > 0.6) {
					ROS_WARN("Human goal being tracked was lost!");
					m_lost.data = true;
					//return;
				} else {
					m_humanGoal.x = m_humanPos.x;
					m_humanGoal.y = m_humanPos.y;
					m_humanGoal.z = m_humanPos.z;
					m_humanGoalPub.publish(m_humanGoal);
					m_lost.data = false;
					m_firstIter = false;
					m_lastHumanPos = m_humanPos;
				}
			}
			m_lostPub.publish(m_lost);
		}
		ROS_INFO("lazarillo position: (%lf, %lf, %lf)", m_humanPos.x, m_humanPos.y, m_humanPos.z);
		ROS_INFO("last lazarillo position: (%lf, %lf, %lf)", m_lastHumanPos.x, m_lastHumanPos.y, m_lastHumanPos.z);    
	}
}

void PeopleFollower::trackingCallback(const std_msgs::Bool &start) {
  m_startTracking = start.data;
}

void PeopleFollower::cropBox (const sensor_msgs::PointCloud2ConstPtr& cloudMsg,
															pcl::PointCloud<pcl::PointXYZI>::Ptr outCloudPtr)
{
	ROS_INFO("Into cropbox");
	pcl::PointXYZI pointFiltered;
	int intensityIdx = -1;

	for (size_t i = 0; i < cloudMsg->fields.size(); ++i) {
		if (cloudMsg->fields[i].name == "intensity") {
			intensityIdx = i;
		}
	}

	if (intensityIdx == -1) {
		ROS_WARN("There is not instensity field");
	}

	sensor_msgs::PointCloud2ConstIterator<float> xIt(*cloudMsg, "x");
	sensor_msgs::PointCloud2ConstIterator<float> yIt(*cloudMsg, "y");
	sensor_msgs::PointCloud2ConstIterator<float> zIt(*cloudMsg, "z");
	sensor_msgs::PointCloud2ConstIterator<float> intensityIt(*cloudMsg, "intensity");
	
	if (!m_startTracking){
		for (int i = 0; i < cloudMsg->width * cloudMsg->height; ++i, ++xIt, ++yIt, ++zIt, ++intensityIt){
			if (*xIt > m_distX || *xIt < 0 ||
				*yIt > m_distY || *yIt < -m_distY ||
				*zIt > m_ZlimitMax || *zIt < m_ZlimitMin){
			continue;
			}
			pointFiltered.x = *xIt;
			pointFiltered.y = *yIt;
			pointFiltered.z = *zIt;
			pointFiltered.intensity = *intensityIt;
			outCloudPtr->points.push_back(pointFiltered);
			//ROS_INFO("intensity point %f", pointFiltered.intensity);
		}
	} else {
		if (m_firstIter) {
			m_humanPos.x = 2.0f;
			m_humanPos.y = 0.0f;
			m_humanPos.z = 0.0f;
			m_distX = m_humanPos.x + 1.0f;
			m_distY = m_humanPos.y + 1.0f;  
		} else {
			m_distX = m_lastHumanPos.x + 1.0f;
			m_distY = m_lastHumanPos.y + 1.0f;
		}
		for (int i = 0; i < cloudMsg->width * cloudMsg->height; ++i, ++xIt, ++yIt, ++zIt, ++intensityIt){
			if (*xIt > m_distX || *xIt < (m_distX - 2.0f) ||
				*yIt > m_distY || *yIt < (m_distY - 2.0f) ||
				*zIt > m_ZlimitMax || *zIt < m_ZlimitMin){
			continue;
			}
			pointFiltered.x = *xIt;
			pointFiltered.y = *yIt;
			pointFiltered.z = *zIt;
			pointFiltered.intensity = *intensityIt;
			outCloudPtr->points.push_back(pointFiltered);
		}
	}
}

void PeopleFollower::extractCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr pc,
																		pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud) {
	m_features.clear();

	pcl::IndicesPtr pcIndices(new std::vector<int>);
	
	boost::array<std::vector<int>, m_nestedRegions> indicesArray;
/* 	for(int i = 0; i < pc->size(); i++) {
		float range = 0.0;
		float d2 = pc->points[i].x * pc->points[i].x +
								pc->points[i].y * pc->points[i].y +
								pc->points[i].z * pc->points[i].z;
		for(int j = 0; j < m_nestedRegions; j++) {
			if(d2 > range*range && d2 <= (range + m_zone[j])*(range + m_zone[j])) {
				indicesArray[j].push_back(i);
				break;
			}
			range += m_zone[j];
		}
	} */

	for(int i = 0; i < pc->size(); i++) {
		if (i % 2 == 0) {
			pcIndices->push_back(i);
		}
	}

	for(int i = 0; i < pcIndices->size(); i++) {
		float range = 0.0;
		float d2 = pc->points[(*pcIndices)[i]].x * pc->points[(*pcIndices)[i]].x +
							 pc->points[(*pcIndices)[i]].y * pc->points[(*pcIndices)[i]].y +
							 pc->points[(*pcIndices)[i]].z * pc->points[(*pcIndices)[i]].z;
		for(int j = 0; j < m_nestedRegions; j++) {
			if(d2 > range*range && d2 <= (range + m_zone[j])*(range + m_zone[j])) {
				indicesArray[j].push_back(i);
				break;
			}
			range += m_zone[j];
		}
	}
	
	float tolerance = 0.0;
	for(int i = 0; i < m_nestedRegions; i++) {
		tolerance += 0.1;
		if(indicesArray[i].size() > m_clusterSizeMin) {
			boost::shared_ptr<std::vector<int> > indicesArrayPtr(new std::vector<int>(indicesArray[i]));
			pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
			tree->setInputCloud(pc, indicesArrayPtr);
			
			std::vector<pcl::PointIndices> clusterIndices;
			pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
			ec.setClusterTolerance(tolerance);
			ec.setMinClusterSize(m_clusterSizeMin);
			ec.setMaxClusterSize(m_clusterSizeMax);
			ec.setSearchMethod(tree);
			ec.setInputCloud(pc);
			ec.setIndices(indicesArrayPtr);
			ec.extract(clusterIndices);
			
			for(std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); it++) {
				pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
				for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
					cluster->points.push_back(pc->points[*pit]);
					clusterCloud->points.push_back(pc->points[*pit]);
				}
				cluster->width = cluster->size();
				cluster->height = 1;
				cluster->is_dense = true;
	
				Eigen::Vector4f min, max, centroid;
				pcl::getMinMax3D(*cluster, min, max);
				pcl::compute3DCentroid(*cluster, centroid);
	
				// Size limitation is not reasonable, but it can increase fps.
				if(m_humanSizeLimit &&
					(max[0]-min[0] < 0.2 || max[0]-min[0] > 1.0 ||
						max[1]-min[1] < 0.2 || max[1]-min[1] > 1.0 ||
						max[2]-min[2] < 0.5 || max[2]-min[2] > 2.0)) 
					continue;
				
				Feature f;
				extractFeature(cluster, f, min, max, centroid);
        ROS_INFO("centroid: X= %f Y= %f Z= %f", f.centroid[0], f.centroid[1], f.centroid[2]);
				ROS_INFO("min: X= %f Y= %f Z= %f", f.min[0], f.min[1], f.min[2]);
				ROS_INFO("max: X= %f Y= %f Z= %f", f.max[0], f.max[1], f.max[2]);
				m_features.push_back(f);
			}
			ROS_INFO("fectures vector: %lu", m_features.size());
		}
	}
	clusterCloud->width = clusterCloud->size();
	clusterCloud->height = 1;
	clusterCloud->is_dense = true;
}

void PeopleFollower::extractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Feature &f,
							Eigen::Vector4f &min, Eigen::Vector4f &max, Eigen::Vector4f &centroid) {
	f.centroid = centroid;
	f.min = min;
	f.max = max;
	
	if(m_useSvmModel) {
		// f1: Number of points included the cluster.
		f.numberPoints = pc->size();
		// f2: The minimum distance to the cluster.
		f.minDistance = FLT_MAX;
		float d2; //squared Euclidean distance
		for(int i = 0; i < pc->size(); i++) {
			d2 = pc->points[i].x*pc->points[i].x + pc->points[i].y*pc->points[i].y + pc->points[i].z*pc->points[i].z;
			if(f.minDistance > d2)
				f.minDistance = d2;
		}
		//f.minDistance = sqrt(f.minDistance);
		
		pcl::PCA<pcl::PointXYZI> pca;
		pcl::PointCloud<pcl::PointXYZI>::Ptr pcProjected(new pcl::PointCloud<pcl::PointXYZI>);
		pca.setInputCloud(pc);
		pca.project(*pc, *pcProjected);
		// f3: 3D covariance matrix of the cluster.
		pcl::computeCovarianceMatrixNormalized(*pcProjected, centroid, f.covariance3d);
		// f4: The normalized moment of inertia tensor.
		computeMomentOfInertiaTensorNormalized(*pcProjected, f.moment3d);
		// Navarro et al. assume that a pedestrian is in an upright position.
		//pcl::PointCloud<pcl::PointXYZI>::Ptr main_plane(new pcl::PointCloud<pcl::PointXYZI>), secondary_plane(new pcl::PointCloud<pcl::PointXYZI>);
		//computeProjectedPlane(pc, pca.getEigenVectors(), 2, centroid, main_plane);
		//computeProjectedPlane(pc, pca.getEigenVectors(), 1, centroid, secondary_plane);
		// f5: 2D covariance matrix in 3 zones, which are the upper half, and the left and right lower halves.
		//compute3ZoneCovarianceMatrix(main_plane, pca.getMean(), f.partial_covariance_2d);
		// f6 and f7
		//computeHistogramNormalized(main_plane, 7, 14, f.histogram_main_2d);
		//computeHistogramNormalized(secondary_plane, 5, 9, f.histogram_second_2d);
		// f8
		computeSlice(pc, 10, f.slice);
		// f9
		computeIntensity(pc, 25, f.intensity);
	}
}

void PeopleFollower::computeMomentOfInertiaTensorNormalized(pcl::PointCloud<pcl::PointXYZI> &pc, Eigen::Matrix3f &moment3d) {
	moment3d.setZero();
	for(size_t i = 0; i < pc.size(); i++) {
		moment3d(0,0) += pc[i].y*pc[i].y+pc[i].z*pc[i].z;
		moment3d(0,1) -= pc[i].x*pc[i].y;
		moment3d(0,2) -= pc[i].x*pc[i].z;
		moment3d(1,1) += pc[i].x*pc[i].x+pc[i].z*pc[i].z;
		moment3d(1,2) -= pc[i].y*pc[i].z;
		moment3d(2,2) += pc[i].x*pc[i].x+pc[i].y*pc[i].y;
	}
	moment3d(1, 0) = moment3d(0, 1);
	moment3d(2, 0) = moment3d(0, 2);
	moment3d(2, 1) = moment3d(1, 2);
}

void PeopleFollower::computeSlice(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, int n, float *slice) {
	Eigen::Vector4f pcMin, pcMax;
	pcl::getMinMax3D(*pc, pcMin, pcMax);
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr blocks[n];
	float itv = (pcMax[2] - pcMin[2]) / n;
	if(itv > 0) {
		for(int i = 0; i < n; i++) {
			blocks[i].reset(new pcl::PointCloud<pcl::PointXYZI>);
		}
		for(unsigned int i = 0, j; i < pc->size(); i++) {
			j = std::min((n-1), (int)((pc->points[i].z - pcMin[2]) / itv));
			blocks[j]->points.push_back(pc->points[i]);
		}
		
		Eigen::Vector4f blockMin, blockMax;
		for(int i = 0; i < n; i++) {
			if(blocks[i]->size() > 0) {
	// pcl::PCA<pcl::PointXYZI> pca;
	// pcl::PointCloud<pcl::PointXYZI>::Ptr block_projected(new pcl::PointCloud<pcl::PointXYZI>);
	// pca.setInputCloud(blocks[i]);
	// pca.project(*blocks[i], *block_projected);
				pcl::getMinMax3D(*blocks[i], blockMin, blockMax);
			} else {
				blockMin.setZero();
				blockMax.setZero();
			}
			slice[i*2] = blockMax[0] - blockMin[0];
			slice[i*2+1] = blockMax[1] - blockMin[1];
		}
	} else {
		for(int i = 0; i < 20; i++)
			slice[i] = 0;
	}
}

void PeopleFollower::computeIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, int bins, float *intensity) {
	float sum = 0, mean = 0, sumDev = 0;
	float min = FLT_MAX, max = -FLT_MAX;
	for(int i = 0; i < 27; i++)
		intensity[i] = 0;
	
	for(size_t i = 0; i < pc->size(); i++) {
		sum += pc->points[i].intensity;
		min = std::min(min, pc->points[i].intensity);
		max = std::max(max, pc->points[i].intensity);
	}
	mean = sum / pc->size();
	
	for(size_t i = 0; i < pc->size(); i++) {
		sumDev += (pc->points[i].intensity - mean)*(pc->points[i].intensity - mean);
		int ii = std::min(float(bins - 1), std::floor((pc->points[i].intensity - min)/((max - min)/bins)));
		intensity[ii]++;
	}
	/* intensity[25] = sqrt(sumDev/pc->size());
	intensity[26] = mean; */
	intensity[25] = 15.2052;
	intensity[26] = 24.6162;
}

geometry_msgs::PoseArray PeopleFollower::classify() {
	visualization_msgs::MarkerArray markerArray;
	geometry_msgs::PoseArray poseArray;
	
	for(std::vector<Feature>::iterator it = m_features.begin(); it != m_features.end(); ++it) {
		if(m_useSvmModel) {
			saveFeature(*it, m_svmNode);
			//std::cerr << "test_id = " << it->id << ", numberPoints = " << it->numberPoints << ", minDistance = " << it->minDistance << std::endl;
			
			// scale data
			for(int i = 0; i < FEATURE_SIZE; i++) {
				if(m_svmScaleRange[i][0] == m_svmScaleRange[i][1]) // skip single-valued attribute
					continue;
				if(m_svmNode[i].value == m_svmScaleRange[i][0])
					m_svmNode[i].value = m_xLower;
				else if(m_svmNode[i].value == m_svmScaleRange[i][1])
					m_svmNode[i].value = m_xUpper;
				else
					m_svmNode[i].value = m_xLower + (m_xUpper - m_xLower) * (m_svmNode[i].value - m_svmScaleRange[i][0]) / (m_svmScaleRange[i][1] - m_svmScaleRange[i][0]);
			}
			
			// predict
			if(m_isProbabilityModel) {
				double probEstimates[m_svmModel->nr_class];
				svm_predict_probability(m_svmModel, m_svmNode, probEstimates);
				if(probEstimates[0] < m_humanProbability)
					continue;
				ROS_INFO("probability: %f", probEstimates[0]);
			} else {
				double svmP = svm_predict(m_svmModel, m_svmNode);
				//ROS_INFO("svmP: %f", svmP);
				if(svmP != 1)
					continue;
			}
		}
		
		visualization_msgs::Marker marker;
		marker.header.stamp = ros::Time::now();
		marker.header.frame_id = m_frameId;
		marker.ns = "follower";
		marker.id = it-m_features.begin();
		marker.type = visualization_msgs::Marker::LINE_LIST;
		geometry_msgs::Point p[24];
		p[0].x = it->max[0]; p[0].y = it->max[1]; p[0].z = it->max[2];
		p[1].x = it->min[0]; p[1].y = it->max[1]; p[1].z = it->max[2];
		p[2].x = it->max[0]; p[2].y = it->max[1]; p[2].z = it->max[2];
		p[3].x = it->max[0]; p[3].y = it->min[1]; p[3].z = it->max[2];
		p[4].x = it->max[0]; p[4].y = it->max[1]; p[4].z = it->max[2];
		p[5].x = it->max[0]; p[5].y = it->max[1]; p[5].z = it->min[2];
		p[6].x = it->min[0]; p[6].y = it->min[1]; p[6].z = it->min[2];
		p[7].x = it->max[0]; p[7].y = it->min[1]; p[7].z = it->min[2];
		p[8].x = it->min[0]; p[8].y = it->min[1]; p[8].z = it->min[2];
		p[9].x = it->min[0]; p[9].y = it->max[1]; p[9].z = it->min[2];
		p[10].x = it->min[0]; p[10].y = it->min[1]; p[10].z = it->min[2];
		p[11].x = it->min[0]; p[11].y = it->min[1]; p[11].z = it->max[2];
		p[12].x = it->min[0]; p[12].y = it->max[1]; p[12].z = it->max[2];
		p[13].x = it->min[0]; p[13].y = it->max[1]; p[13].z = it->min[2];
		p[14].x = it->min[0]; p[14].y = it->max[1]; p[14].z = it->max[2];
		p[15].x = it->min[0]; p[15].y = it->min[1]; p[15].z = it->max[2];
		p[16].x = it->max[0]; p[16].y = it->min[1]; p[16].z = it->max[2];
		p[17].x = it->max[0]; p[17].y = it->min[1]; p[17].z = it->min[2];
		p[18].x = it->max[0]; p[18].y = it->min[1]; p[18].z = it->max[2];
		p[19].x = it->min[0]; p[19].y = it->min[1]; p[19].z = it->max[2];
		p[20].x = it->max[0]; p[20].y = it->max[1]; p[20].z = it->min[2];
		p[21].x = it->min[0]; p[21].y = it->max[1]; p[21].z = it->min[2];
		p[22].x = it->max[0]; p[22].y = it->max[1]; p[22].z = it->min[2];
		p[23].x = it->max[0]; p[23].y = it->min[1]; p[23].z = it->min[2];
		for(int i = 0; i < 24; i++)
			marker.points.push_back(p[i]);
		marker.scale.x = 0.02;
		marker.color.a = 1.0;
		if(!m_useSvmModel) {
			marker.color.r = 0.0;
			marker.color.g = 0.5;
			marker.color.b = 1.0;
		} else {
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.5;
		}
		
		marker.lifetime = ros::Duration(0.1);
		markerArray.markers.push_back(marker);
		
		geometry_msgs::Pose pose;
		pose.position.x = it->centroid[0];
		pose.position.y = it->centroid[1];
		pose.position.z = it->centroid[2];
		pose.orientation.w = 1;
		poseArray.poses.push_back(pose);
	}
	
	if(markerArray.markers.size()) {
		m_markerArrayPub.publish(markerArray);
	}
	if(poseArray.poses.size()) {
		poseArray.header.stamp = ros::Time::now();
		poseArray.header.frame_id = m_frameId;
		m_poseArrayPub.publish(poseArray);
	}

	return poseArray;
}

void PeopleFollower::saveFeature(Feature &f, struct svm_node *x) {
	x[0].index  = 1;  x[0].value  = f.numberPoints; // libsvm indices start at 1
	x[1].index  = 2;  x[1].value  = f.minDistance;
	x[2].index  = 3;  x[2].value  = f.covariance3d(0,0);
	x[3].index  = 4;  x[3].value  = f.covariance3d(0,1);
	x[4].index  = 5;  x[4].value  = f.covariance3d(0,2);
	x[5].index  = 6;  x[5].value  = f.covariance3d(1,1);
	x[6].index  = 7;  x[6].value  = f.covariance3d(1,2);
	x[7].index  = 8;  x[7].value  = f.covariance3d(2,2);
	x[8].index  = 9;  x[8].value  = f.moment3d(0,0);
	x[9].index  = 10; x[9].value  = f.moment3d(0,1);
	x[10].index = 11; x[10].value = f.moment3d(0,2);
	x[11].index = 12; x[11].value = f.moment3d(1,1);
	x[12].index = 13; x[12].value = f.moment3d(1,2);
	x[13].index = 14; x[13].value = f.moment3d(2,2);
	for(int i = 0; i < 20; i++) {
		x[i+14].index = i+15;
		x[i+14].value = f.slice[i];
	}
	for(int i = 0; i < 27; i++) {
		x[i+34].index = i+35;
		x[i+34].value = f.intensity[i];
	}
	x[FEATURE_SIZE].index = -1;
	
	for(int i = 0; i < FEATURE_SIZE; i++) {
	//ROS_INFO_STREAM("Feature: Index = " << x[i].index << " Value = " << x[i].value << " ");
	}
}

pcl::PointXYZ PeopleFollower::getClosestPose(pcl::PointXYZ &p, geometry_msgs::PoseArray &poseArray) {
	auto minDistance = DBL_MAX;
	double distance2;
	pcl::PointXYZ closestPose;
	for (auto &i : poseArray.poses) {
		distance2 = (p.x - i.position.x)*(p.x - i.position.x) + (p.y - i.position.y)*(p.y - i.position.y);
		if (distance2 < minDistance) {
			closestPose.x = i.position.x;
			closestPose.y = i.position.y;
			minDistance = distance2;
		}
	}
	closestPose.z = 0;
	return closestPose;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "people_follower");
  auto peopleF = PeopleFollower();
  ros::spin();
  return 0;
}