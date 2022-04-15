//
// Created by zbt on 22-4-16.
//
#include "pcd_filters.h"

// Downsample, according to the number npoints.
void random_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &PtrCloud, int npoints = 8000) {
    if (PtrCloud->points.size() < npoints)
        return;

    pcl::RandomSample<pcl::PointXYZRGB> random_sampler;
    // random_sampler.setKeepOrganized(true);// extremly time-consuming
    random_sampler.setInputCloud(PtrCloud);

    uint num_output_points = npoints;
    random_sampler.setSample(num_output_points);
    random_sampler.filter(*PtrCloud);
}

void pass_through_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &PtrCloud) {
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
}