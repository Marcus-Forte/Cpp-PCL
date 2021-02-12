#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/passthrough.h>

#include "Features.h"

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer);

PointCloudT::Ptr cloud_input = pcl::make_shared<PointCloudT>();    //input
PointCloudT::Ptr edge_features = pcl::make_shared<PointCloudT>();  //input
PointCloudT::Ptr plane_features = pcl::make_shared<PointCloudT>(); //input

void KeyCallback(const pcl::visualization::KeyboardEvent &event)
{
    // cout<< "key";

    switch (event.getKeyCode())
    {
    case ('1'):
        viewer->removeAllPointClouds();
        viewer->addPointCloud(cloud_input, "cloud");
        // cout << "1";
        break;

    case ('2'):
        viewer->removeAllPointClouds();
        viewer->addPointCloud(cloud_input, "cloud");
        viewer->addPointCloud(edge_features, "edges");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "edges");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "edges");
        // cout << "2";
        break;

    case ('3'):
        viewer->removeAllPointClouds();
        viewer->addPointCloud(cloud_input, "cloud");
        viewer->addPointCloud(plane_features, "planes");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "planes");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "planes");
        // cout << "3";
        break;

    case ('a'):
        viewer->removeAllPointClouds();
        viewer->addPointCloud(cloud_input, "cloud");
        viewer->addPointCloud(edge_features, "edges");
        viewer->addPointCloud(plane_features, "planes");

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "edges");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "edges");

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "planes");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "planes");
        break;
    }
}

void printUsage()
{
    std::cout << "Usage : features [target.pcd]" << std::endl;
}

int main(int argc, char **argv)
{

    if (argc < 2)
    {
        printUsage();
        exit(-1);
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud_input) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file model \n");
        return (-1);
    }
    // Passthrough
    // Prefiltering
    pcl::PassThrough<pcl::PointXYZ> pass_through;
    // pass_through.setInputCloud(input_clzzz)
    pass_through.setInputCloud(cloud_input);
    pass_through.setFilterFieldName("x");
    pass_through.setFilterLimits(0, 0.05); // remove weird points close to origin
    pass_through.setNegative(true);
    pass_through.filter(*cloud_input);

    Features::EdgeDetection<pcl::PointXYZ>(*cloud_input, *edge_features, 5, 4);
    Features::ComputeSmoothness<pcl::PointXYZ>(*cloud_input, *plane_features, 10, 8);

    
    viewer->addPointCloud(cloud_input, "cloud");
    viewer->addPointCloud(edge_features, "edges");
    viewer->addPointCloud(plane_features, "planes");
    viewer->addCoordinateSystem(1);
    viewer->registerKeyboardCallback(KeyCallback);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "edges");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "edges");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "planes");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "planes");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}