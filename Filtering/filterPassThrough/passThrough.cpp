#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

int
 main ()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  reader.read<pcl::PointXYZ> ("/home/dmrud/Git/pcl_tutor/data/downsample.pcd", *cloud);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 0.9);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  writer.write<pcl::PointXYZ> ("/home/dmrud/Git/pcl_tutor/data/tested.pcd", *cloud_filtered, false);
  
  return (0);
}
