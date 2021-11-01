#include <iostream>
#include <thread>
#include <vector>
#include <iostream>
#include <cstdlib>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing_rgb.h>

using namespace std::chrono_literals;

int main (int argc, char** argv)
{
  if(argc < 5){
    std::cout << "Too fewarguments" << std::endl
              << "Distance PointColorThreshold RegionColorThreshold ClusterSize" << std::endl;
              return -1;
  }
  int dist, pointThreshold, regionThreshold, clusterSize;
  dist =  std::atoi(argv[1]);
  pointThreshold =  std::atoi(argv[2]);
  regionThreshold =  std::atoi(argv[3]);
  clusterSize =  std::atoi(argv[4]);

  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  //region_growing_rgb_tutorial
  if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("/home/dmrud/Git/pcl_tutor/data/pass_through.pcd", *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::removeNaNFromPointCloud (*cloud, *indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloud);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (dist); //default 10
  reg.setPointColorThreshold (pointThreshold); //default 6
  reg.setRegionColorThreshold (regionThreshold); //default 5
  reg.setMinClusterSize (clusterSize); //default 600

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud (colored_cloud);
  while (!viewer.wasStopped ())
  {
    std::this_thread::sleep_for(100us);
  }

  return (0);
}