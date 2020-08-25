#pragma once


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/console/print.h>


#include <pcl/visualization/pcl_visualizer.h>

// Classe utilitária


class PCUtils {
public:
    static void makeFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, float density);

    static void quickView(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in);

    static void printPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, const std::string& name);


private:
    PCUtils();
    ~PCUtils();


};

