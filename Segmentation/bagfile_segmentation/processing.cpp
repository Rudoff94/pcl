#include "processing.h"


pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filtering(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsapledCloud (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr passCloud (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>());

    //pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    pcl::VoxelGrid<pcl::PointXYZ> gridFilter;
    gridFilter.setInputCloud(inputCloud);
    gridFilter.setLeafSize (0.01f, 0.01f, 0.01f);
    gridFilter.filter(*downsapledCloud);

    //passTr
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (downsapledCloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.2);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*passCloud);
    
    //outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(passCloud);
    sor.setMeanK(20);
    sor.setStddevMulThresh (1.0);
    sor.filter(*filteredCloud);
    
    return filteredCloud;
}

pcl::PointCloud <pcl::PointXYZRGB>::Ptr reg_grow_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud){
    //get normals
    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (downsampledCloud);
    //normal_estimator.setSearchSurface(rawCloud);
    normal_estimator.setKSearch (30);
    normal_estimator.compute (*normals);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::removeNaNFromPointCloud(*downsampledCloud, *indices);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (50);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (downsampledCloud);
    reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (8.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (0.5);
    
    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    coloredCloud = reg.getColoredCloud ();//(new pcl::PointCloud<pcl::PointXYZRGB>());
    //coloredCloud = reg.getColoredCloud ();
    return coloredCloud;
}