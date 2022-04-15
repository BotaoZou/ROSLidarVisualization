//
// Created by zbt on 22-4-16.
//
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>
#ifndef BEGINNER_TUTORIALS_PCD_FILTERS_H
#define BEGINNER_TUTORIALS_PCD_FILTERS_H
void random_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &PtrCloud, int npoints );
void pass_through_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &PtrCloud);
#endif //BEGINNER_TUTORIALS_PCD_FILTERS_H
