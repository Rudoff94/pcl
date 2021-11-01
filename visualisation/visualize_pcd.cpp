#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <iostream>
#include <filesystem>

int main(int argc, char** argv){

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
    
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    
    pcl::visualization::CloudViewer viewer(filename);
    
    reader.read(filename, cloud);
    viewer.showCloud(cloud.makeShared());
    while(!viewer.wasStopped()){

    }
    return 0;
}