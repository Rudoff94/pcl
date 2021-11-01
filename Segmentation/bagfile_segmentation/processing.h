#pragma once

#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <librealsense2/rs.hpp>

pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtering(pcl::PointCloud<pcl::PointXYZ>::Ptr);
pcl::PointCloud <pcl::PointXYZRGB>::Ptr reg_grow_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr);
