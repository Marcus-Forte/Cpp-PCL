#pragma once

#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>




template <class PointT>
class volumeEstimator {
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    

    public:

    volumeEstimator(double resolution_) : resolution(resolution_) {
        
    }

    inline void setInputCloud(PointCloudConstPtr cloud){
        this->cloud = cloud; // share ownership
    }

    inline void setGroundCloud(PointCloudConstPtr ground){
        this->ground = ground; // share owenership
    }


    double compute(){
        PointCloudPtr pc (new PointCloud);
        pcl::VoxelGrid<PointT> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(resolution,resolution,resolution);
        voxel.filter(*pc);



        
        return 0;

    }

    ~volumeEstimator(){
        
    }

    private:
    PointCloudConstPtr cloud;
    PointCloudConstPtr ground;
    double resolution;
};