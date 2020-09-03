#pragma once


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/console/print.h>

// Fill
#include <pcl/common/centroid.h>

#include <pcl/search/kdtree.h>
#include <pcl/octree/octree_search.h>


#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <memory>
// Classe utilitária




class PCUtils {
public:


    //TODO Make floor smart | Improve Herustics
    static void makeFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, float density);


    static void printPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, const std::string& name);

    //TODO fill clouds with points between floor and cloud top

    // TODO heuristics
    static void fillCloud(const  pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,int density = 2);


    // This method should inherit from pcl::filters..
    static void ROIFilter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in,const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& ROI);


    // 2.5D volume
    static float computeVolume(const  pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in,float res);


    // Mesh volume .. 


private:
    PCUtils(){}
    ~PCUtils(){}
};

