cmake_minimum_required(VERSION 2.8.3)
project(ros_point_cloud_display)

## Find catkin and any catkin packages
#find_package(catkin REQUIRED )

find_package(catkin REQUIRED COMPONENTS roscpp rospy genmsg pcl_ros pcl_conversions sensor_msgs)
#find_package(PCL REQUIRED)

#std_msgs
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

add_executable(show_pointcloud src/show_pointcloud.cpp)
target_link_libraries(show_pointcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES} /usr/local/lib/libyaml-cpp.a)
add_dependencies(show_pointcloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )



