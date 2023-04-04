cmake_minimum_required(VERSION 3.5)
project(anticrash)

set(CMAKE_CXX_FLAGS "-std=c++17 -fPIC")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        visualization_msgs
        pcl_ros
        pcl_conversions
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(LIBRARIES anticrash
        CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        visualization_msgs
        pcl_ros
        pcl_conversions
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/antiCrash.cpp
)

add_executable(antiCrash src/antiCrashNode.cpp)
target_link_libraries(antiCrash ${PROJECT_NAME} ${catkin_LIBRARIES} -lrt -lrt)
add_dependencies(antiCrash ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
