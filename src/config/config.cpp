//
// Created by zbt on 22-4-16.
//
#include "config.h"
using namespace std;
Config::Config(const string &config_file) {
    YAML::Node config = YAML::LoadFile(config_file);
    this->root_path = config["root_path"].as<string>();
    this->bin_path = this->root_path + config["bin_path"].as<string>();
    this->label_path = this->root_path + config["label_path"].as<string>();
    this->pcd_path = this->root_path + config["pcd_path"].as<string>();
    this->color_map_file = this->root_path + config["color_map_file"].as<string>();
}