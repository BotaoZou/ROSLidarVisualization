//
// Created by zbt on 22-4-16.
//
#include "yaml-cpp/yaml.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#ifndef BEGINNER_TUTORIALS_PCD_PROCESS_H
#define BEGINNER_TUTORIALS_PCD_PROCESS_H
struct Color {
    int b = 0;
    int g = 0;
    int r = 0;

    Color() = default;

    Color(int b, int g, int r) : b(b), g(g), r(r) {}
};

std::map<int, Color> init_color_map(const std::string &yaml_file);

void transform_bin_to_pcd(const std::string &bin_file, std::string &pcd_file);

void add_semantic_label(const std::string &label_file, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                        std::map<int, Color> &color_map);
#endif //BEGINNER_TUTORIALS_PCD_PROCESS_H
