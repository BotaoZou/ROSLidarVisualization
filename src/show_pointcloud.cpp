//
// Created by zbt on 2022/4/2.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "utils/pcd_process.h"
#include "utils/pcd_filters.h"
#include "config/config.h"

using namespace std;

void init_ros(int argc, char **argv) {
    ros::init(argc, argv, "show_pcd");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("showpointcloud_output", 1);
    sensor_msgs::PointCloud2 output;
}

int pre_process_pcd(){
    Config config("/home/zbt/ROSLidarVisualization/src/dataset/config.yaml");
    string bin_file = "/home/zbt/ROSLidarVisualization/src/dataset/bin/000000.bin";
    string pc_file = "/home/zbt/ROSLidarVisualization/src/dataset/pcd/000000.pcd";
    string label_file = "/home/zbt/ROSLidarVisualization/src/dataset/label/000000.label";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    transform_bin_to_pcd(bin_file, pc_file);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pc_file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read pcd file \n");
        return (-1);
    }

    //Get the color map from config
    map<int, Color> color_map = init_color_map(config.color_map_file);

    //Add color to point cloud
    add_semantic_label(label_file, cloud, color_map);

//    //Down sample the lidar point cloud
//    random_filter(cloud, PC_downsamppts);
//
//    //Pass through
//    pass_through_filter(cloud);
}

int main(int argc, char **argv) {
//    init_ros(argc, argv);
    pre_process_pcd();

    //Convert the cloud to ROS message
//    pcl::toROSMsg(*cloud, output);
//    output.header.frame_id = "global";
//
//    ros::Rate loop_rate(0.1);
//    while (ros::ok()) {
//        pcl_pub.publish(output);
//        ros::spinOnce();
//        loop_rate.sleep();
//    }

    return 0;
}