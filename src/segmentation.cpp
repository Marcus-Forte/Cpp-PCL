#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration_2d.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <thread>


using PointT = pcl::PointXYZ;

bool segment = false;

void keyCallback(const pcl::visualization::KeyboardEvent &event, void *cookie)
{
    if (event.keyDown() && event.getKeyCode() == 'a')
    {
        segment = true;
    }
}

int main(int argc, char **argv)
{

    if (argc < 3)
    {
        PCL_ERROR("Not enough arguments!\n");
        PCL_ERROR("segmentation [cloud.pcd] [dist_thres]");
        exit(-1);
    }

    // float voxel_res = atof(argv[2]);
    float dist_thres = atof(argv[2]);
    // std::cout << "dist = " << dist_thres << std::endl;

    std::string cloud_file = argv[1];
    pcl::PointCloud<PointT>::Ptr cloud = pcl::make_shared<pcl::PointCloud<PointT>>();

    if (pcl::io::loadPCDFile(cloud_file, *cloud) == -1)
    {
        PCL_ERROR("Error loading point cloud.");
        exit(-1);
    }

    PCL_INFO("N points -> %d\n", cloud->size());

    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_line = std::make_shared<pcl::SampleConsensusModelLine<pcl::PointXYZ>>(cloud);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_(model_line);
    ransac_.setDistanceThreshold(dist_thres);

    pcl::SACSegmentation<pcl::PointXYZ> ransac;
    ransac.setModelType(pcl::SACMODEL_LINE);
    ransac.setDistanceThreshold(dist_thres);
    ransac.setMethodType(pcl::SAC_PROSAC);
    ransac.setInputCloud(cloud);
    

    pcl::IndicesPtr inliers = std::make_shared<pcl::Indices>();
    pcl::PointIndicesPtr inliers_ = std::make_shared<pcl::PointIndices>();
    

    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud(cloud);
    

    pcl::PointCloud<PointT>::Ptr cloud_extract = pcl::make_shared<pcl::PointCloud<PointT>>();

    pcl::visualization::PCLVisualizer viewer("Viewer");
    viewer.registerKeyboardCallback(keyCallback);
    viewer.addPointCloud(cloud);
    int count = 0;
    pcl::ModelCoefficients model;

    

    while (!viewer.wasStopped())
    {
        if (segment)
        {
            std::cout << "Segmenting..." << std::endl;
            auto start = std::chrono::system_clock::now();
            model_line->setInputCloud(cloud);
            
            ransac_.computeModel();
            
            ransac_.getInliers(*inliers);
            extractor.setIndices(inliers);
            
            // ransac.setOptimizeCoefficients(true);
            // ransac.setNumberOfThreads(2);
            // ransac.segment(*inliers_,model);
            
            auto end = std::chrono::system_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << "Time: " << elapsed.count() << std::endl;
        
            extractor.setNegative(false);
            extractor.filter(*cloud_extract);
            
            extractor.setNegative(true);
            extractor.filter(*cloud); //Remaining
            
            PCL_INFO("Remaining -> %d\n",cloud->size());
            PCL_INFO("Extracted -> %d\n",cloud_extract->size());
            
            viewer.removeAllPointClouds();
            viewer.addPointCloud(cloud_extract,"extract");
            viewer.addPointCloud(cloud,"remain");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"extract");
            

            segment = false;
        }
        viewer.spinOnce(10);
    }

    // PCL_INFO("Inliers -> %d\n", inliers->size());
    // PCL_INFO("Extracted -> %d\n", cloud_extract->size());
    // PCL_INFO("Remain -> %d\n", cloud->size());

    return 0;
}