#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/passthrough.h>

#include <pcl/common/random.h>

#include "Features.h"

using namespace std;

bool userContinue = false;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
pcl::visualization::PCLVisualizer::Ptr viewer;

PointCloudT::Ptr cloud_input = pcl::make_shared<PointCloudT>();    //input
PointCloudT::Ptr edge_features = pcl::make_shared<PointCloudT>();  //input
PointCloudT::Ptr plane_features = pcl::make_shared<PointCloudT>(); //input

void waitUser(const pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    while (userContinue == false)
    {
        viewer->spinOnce();
    }
    userContinue = false;

    if (viewer->wasStopped())
    {
        exit(0);
    }
}

void PointPickCallback(const pcl::visualization::PointPickingEvent& event){
    float x,y,z;
    event.getPoint(x,y,z);
    cout << "ID: " << event.getPointIndex() << endl;
    cout << "(" << x << "," << y << "," << z << ")" << endl;
    cout << "Assert:" << cloud_input->points[event.getPointIndex()] << endl;
    
}

void KeyCallback(const pcl::visualization::KeyboardEvent &event)
{
    // cout<< "key";
    if (!event.keyDown())
        return;

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

    case ('*'): // DEBUG
        userContinue = true;
        break;
    }
}

void printUsage()
{
    std::cout << "Usage : features [cloud.pcd] [partitions] [n_highest]" << std::endl;
}

int main(int argc, char **argv)
{

    if (argc < 4)
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

    viewer = pcl::make_shared<pcl::visualization::PCLVisualizer>("VIEWER");

    // Cloud Partitioning
    std::vector<PointCloudT> partitions;
    int N_partitions = atoi(argv[2]);
    int N_highest = atoi(argv[3]);
    Features::PartitionCloud(*cloud_input, partitions, N_partitions);

    // for(int i = 0;i < N_partitions;++i){
    //     cout << "Partition: " << i << endl;
    //     for(int j=0;j<partitions[i].size(); ++ j){
    //         cout << "Pt: " << partitions[i].points[j] << endl;
    //     }
    // }

    viewer->registerKeyboardCallback(KeyCallback);
    viewer->registerPointPickingCallback(PointPickCallback);

    PointCloudT::Ptr temp(new PointCloudT);
    PointCloudT::Ptr temp2(new PointCloudT);
    PointCloudT::Ptr temp3(new PointCloudT);

    for (int i = 0; i < N_partitions; ++i)
    {

        PointCloudT edges;
        PointCloudT planes;
        // Features::EdgeDetection<pcl::PointXYZ>(partitions[i], edges, 6, N_highest);
        Features::ComputeSmoothness<pcl::PointXYZ>(partitions[i], edges, planes, 6, N_highest, N_highest);

        *temp = partitions[i];
        *temp2 = edges;
        *temp3 = planes;
        viewer->addPointCloud(temp, "input" + to_string(i));
        viewer->addPointCloud(temp2, "edges" + to_string(i));
        viewer->addPointCloud(temp3, "planes" + to_string(i));
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10,"feat" + to_string(i));
        float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        float g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        float b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, "input" + to_string(i));
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "input" + to_string(i));

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "edges" + to_string(i));
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "edges" + to_string(i));

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "planes" + to_string(i));
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "planes" + to_string(i));

        waitUser(viewer);

        *edge_features += edges;
        *plane_features += planes;
    }

    viewer->removeAllPointClouds();

    viewer->addPointCloud(cloud_input, "cloud");
    viewer->addPointCloud(edge_features, "edges");
    viewer->addPointCloud(plane_features, "planes");
    viewer->addCoordinateSystem(1);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "edges");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "edges");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "planes");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "planes");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}