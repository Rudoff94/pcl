#include <iostream>
#include <filesystem>
#include <string>

#include <opencv2/opencv.hpp>

#include <librealsense2/rs.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/thread/thread.hpp>

#include "processing.h"

int main(int argc, char** argv){

    //parse filename
    if(argc != 2){
        std::cout << "ERROR: Too few arguments (correct - 2)";
        return -1;
    }
    else if(!std::filesystem::exists(argv[1]) ){
        std::cout << "ERROR: File does not exist" << std::endl;
        return -1;
    }

    std::string filename = argv[1];

    //setup realsense params
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::pointcloud pCloud;

    cfg.enable_device_from_file(filename);
    cfg.enable_stream(RS2_STREAM_DEPTH);
    rs2::pipeline_profile selection = pipe.start(cfg);

    bool loopFlag = true;

    pcl::visualization::CloudViewer viewer ("Cloud");

    while(loopFlag){
        //catch frame
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();
        
        //convert to PCL
        rs2::points rsPoints = pCloud.calculate(depth);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pclCloud = points_to_pcl(rsPoints);

        //filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
        filteredCloud = filtering(pclCloud);
        
        
        //Segmentation
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud = reg_grow_segmentation(filteredCloud);
        // std::cout << pclCloud << ' ' << filteredCloud << ' ' << coloredCloud << std::endl;

        if(coloredCloud != nullptr){
            viewer.showCloud(coloredCloud);
        }
        
        
        // while (!viewer.wasStopped ())
        // {
        //     // int key = cv::waitKey(0);
        //     // if(key == 'q'){ 
        //     //     loopFlag = false;
        //     // }
        //     std::cout << "here" << std::endl;
        // }
        if(viewer.wasStopped()){
            break;
        } 
    }
    

    

    return 0;
}