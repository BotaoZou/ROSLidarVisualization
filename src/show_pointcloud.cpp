//
// Created by zbt on 2022/4/2.
//

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>
#include "yaml-cpp/yaml.h"

using namespace std;

struct Color {
    int b = 0;
    int g = 0;
    int r = 0;

    Color() = default;

    Color(int b, int g, int r) : b(b), g(g), r(r) {}
};

// Downsample, according to the number npoints.
int random_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &PtrCloud, int npoints = 8000) {
    if (PtrCloud->points.size() < npoints)
        return 0;

    ///sample downsample filter, random downsize (takes 0.001s)
    pcl::RandomSample<pcl::PointXYZRGB> random_sampler;
    // random_sampler.setKeepOrganized(true);// extremly time-consuming
    random_sampler.setInputCloud(PtrCloud);

    uint num_output_points = npoints;
    random_sampler.setSample(num_output_points);
    random_sampler.filter(*PtrCloud);

    return 0;
}


int pass_through_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &PtrCloud) {
    // PassThrough filter
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(PtrCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.2, 3.5);
    // pass.setFilterFieldName ("y"); 
    // pass.setFilterLimits (-0.5, 0.5);
    //pass.setFilterFieldName ("x");
    //pass.setFilterLimits (-18, +18);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*PtrCloud);
    return 0;
}

map<int, Color> init_color_map(string yaml_file) {
//    YAML::Node config = YAML::LoadFile(yaml_file);
    YAML::Node config = YAML::LoadFile("/home/zbt/catkin_ws/src/beginner_tutorials/src/semantic-kitti.yaml");
    YAML::Node config_color_map = config["color_map"];
    map<int, Color> color_map;
    for (YAML::const_iterator i = config_color_map.begin(); i != config_color_map.end(); i++) {
        vector<int> bgr = i->second.as<vector<int>>();
        Color color(bgr[0], bgr[1], bgr[2]);
        color_map[i->first.as<int>()] = color;
    }
    cout << "color map is initialized " << endl;
}

void add_semantic_label_color(string label_file, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                              map<int, Color> &color_map) {
    fstream input("/home/zbt/catkin_ws/src/beginner_tutorials/src/000000.label");
    short label;
//    input.open(label_file);
    input.seekg(ios::beg);
    for (auto &i: *points) {
        input.read((char *) &label, 4);
        Color &color = color_map[label];
        i.b = color.b;
        i.g = color.g;
        i.r = color.r;
    }
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB>("/home/zbt/catkin_ws/src/beginner_tutorials/src/target.pcd", *points, false);
    cout << "semantic label and color is added " << endl;

}

void readKittiPclBinData(std::string &in_file, std::string &out_file) {
    // load point cloud
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if (!input.good()) {
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(new pcl::PointCloud<pcl::PointXYZRGB>);

    int i;
    char intensity;
    for (i = 0; input.good() && !input.eof(); i++) {
        pcl::PointXYZRGB point;
        input.read((char *) &point.x, 3 * sizeof(float));
        input.read((char *) &intensity, sizeof(float));
        point.b = 0;
        point.g = 0;
        point.r = 0;
        points->push_back(point);
    }
    input.close();
//    g_cloud_pub.publish( points );

    std::cout << "Read KITTI point cloud with " << i << " points, writing to " << out_file << std::endl;
    pcl::PCDWriter writer;

    // Save DoN features
    writer.write<pcl::PointXYZRGB>(out_file, *points, false);
}


int main(int argc, char **argv) {
//    ros::init(argc, argv, "readin");
//
//    ros::NodeHandle nh;
//    ros::NodeHandle nhPrivate("~");
//    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("showpointcloud_output", 1);
//    sensor_msgs::PointCloud2 output;
    string bin_path = "/home/zbt/catkin_ws/src/beginner_tutorials/src/000000.bin";

    string PC_path = "/home/zbt/catkin_ws/src/beginner_tutorials/src/000000.pcd";
//    const int PC_downsamppts = 300000;
//    std::string aa;
//
//    // no label
    readKittiPclBinData(bin_path, PC_path);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(PC_path, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read pcd file \n");
        return (-1);
    }

    //Get the color map from config
    map<int, Color> color_map = init_color_map("");

    //Add color to point cloud
    add_semantic_label_color("", cloud, color_map);

//    //Down sample the lidar point cloud
//    random_filter(cloud, PC_downsamppts);
//
//    //Pass through
//    pass_through_filter(cloud);

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