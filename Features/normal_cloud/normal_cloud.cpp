#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <string>
#include <iostream>

int main(int argc, char** argv)
{
    if(argc < 2 || argc > 3){
        std::cout << "ERORR: wrong number of arguments (correct: 2)" << std::endl;
        return -1;
    }
    
    std::string filename1 = argv[1];
    std::string filename2;
    std::filesystem::path pth1 = filename1;
    std::filesystem::path pth2;

    if(!std::filesystem::exists(pth1)){
        std::cout << "ERORR: File " << filename1 << " does not exist" << std::endl;
        return -1;
    }

    if(argc == 3){
        filename2 = argv[2];
        pth2 = filename2;
        if(!std::filesystem::exists(pth2)){
            std::cout << "ERORR: File " << filename2 << " does not exist" << std::endl;
            return -1;
        }
    }



    std::cout << "Load file" << std::endl; 

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PCDReader reader;
    std::cout << "Load: " << filename1 << std::endl; 
    reader.read(filename1, *downsampled_cloud);

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (downsampled_cloud);
    
    if(argc == 3){
        std::cout << "Load: " << filename2 << std::endl;
        reader.read(filename2, *cloud);
        ne.setSearchSurface (cloud);
    }
    
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.05);

    std::cout << "Compute the features" << std::endl;
    // Compute the features
    ne.compute (*cloud_normals);

    std::cout << *cloud_normals << std::endl;
    
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(downsampled_cloud, cloud_normals);
    viewer.addPointCloud(downsampled_cloud);
    
    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
    }
    // for(auto& normal: *cloud_normals){
    //     std::cout << normal << std::endl;
    // }

    return 0;
    // cloud_normals->size () should have the same size as the input cloud->size ()
}
