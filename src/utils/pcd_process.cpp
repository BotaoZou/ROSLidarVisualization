//
// Created by zbt on 22-4-16.
//
#include "pcd_process.h"

using namespace std;

map<int, Color> init_color_map(const string &color_map_file) {
    YAML::Node config = YAML::LoadFile(color_map_file);
    YAML::Node config_color_map = config["color_map"];
    map<int, Color> color_map;
    for (YAML::const_iterator i = config_color_map.begin(); i != config_color_map.end(); i++) {
        vector<int> bgr = i->second.as<vector<int>>();
        Color color(bgr[0], bgr[1], bgr[2]);
        color_map[i->first.as<int>()] = color;
    }
    cout << "color map is initialized " << endl;
    return color_map;
}



void transform_bin_to_pcd(const string &bin_file, string &pcd_file) {
    fstream input(bin_file, ios::in | ios::binary);
    if (!input.good()) {
        cerr << "Could not read file: " << bin_file << endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, ios::beg);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(new pcl::PointCloud<pcl::PointXYZRGB>);

    int i = 0;
    while (!input.eof()) {
        pcl::PointXYZRGB point;
        input.read((char *) &point.x, 3 * sizeof(float));
        input.seekg(sizeof(float), ios::cur);
        point.b = 0;
        point.g = 0;
        point.r = 0;
        points->push_back(point);
        i++;
        if (input.peek() == EOF) break;
    }
    input.close();

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB>(pcd_file, *points, false);
    cout << "Read KITTI point cloud with " << i << " points, writing to " << pcd_file << endl;

}

void add_semantic_label(const string &label_file, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                        map<int, Color> &color_map) {
    fstream input(label_file, ios::in | ios::binary);
    short label;
    input.seekg(0, ios::beg);
    for (auto &i: *points) {
        input.read((char *) &label, 4);
        Color &color = color_map[label];
        i.b = color.b;
        i.g = color.g;
        i.r = color.r;
    }
    input.close();

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB>("/home/zbt/ROSLidarVisualization/src/dataset/pcd/target.pcd", *points, false);
    cout << "semantic label and color is added " << endl;
}