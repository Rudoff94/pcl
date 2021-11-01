#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <filesystem>
#include <string>


int main (int argc, char** argv)
{

  if(argc != 2){
        std::cout << "ERORR: wrong number of arguments (correct: 2)" << std::endl;
        return -1;
    }

    std::string filename = argv[1];
    std::filesystem::path pth = filename;

    if(!std::filesystem::exists(pth)){
        std::cout << "ERORR: File does not exist" << std::endl;
        return -1;
    }
  
  std::cout << filename << std::endl; 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> (filename, *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;
  //std::cerr << cloud->at(500).x << std::endl;
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> (filename + "inliers.pcd", *cloud_filtered, false);

  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> (filename + "outliers.pcd", *cloud_filtered, false);

  return (0);
}
