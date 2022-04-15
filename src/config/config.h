//
// Created by zbt on 22-4-16.
//
#include "yaml-cpp/yaml.h"

#ifndef BEGINNER_TUTORIALS_CONFIG_H
#define BEGINNER_TUTORIALS_CONFIG_H


class Config {
public:
    std::string root_path;
    std::string bin_path;
    std::string label_path;
    std::string pcd_path;
    std::string color_map_file;
    Config(const std::string& config_file);
};

#endif //BEGINNER_TUTORIALS_CONFIG_H
