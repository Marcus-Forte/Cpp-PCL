#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <thread>

using PointT = pcl::PointXYZ;

int main(int argc, char **argv)
{

    if (argc < 4)
    {
        PCL_ERROR("Not enough arguments!\n");
        PCL_ERROR("segmentation [cloud.pcd] [voxel_res] [dist_thres]");
        exit(-1);
    }

    float voxel_res = atof(argv[2]);
    float dist_thres = atof(argv[3]);
    // std::cout << "dist = " << dist_thres << std::endl;

    std::string cloud_file = argv[1];
    pcl::PointCloud<PointT>::Ptr cloud = pcl::make_shared<pcl::PointCloud<PointT>>();
    pcl::PointCloud<PointT>::Ptr cloud_f = pcl::make_shared<pcl::PointCloud<PointT>>();

    if (pcl::io::loadPCDFile(cloud_file, *cloud) == -1)
    {
        PCL_ERROR("Error loading point cloud.");
        exit(-1);
    }

    pcl::visualization::PCLVisualizer viewer("Viewer");
    static int count = 0;
    std::cout << "PointCloud before filtering has: " << cloud->size() << " data points." << std::endl; //*

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(voxel_res, voxel_res, voxel_res);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(dist_thres);
    

    int i = 0, nr_points = (int)cloud_filtered->size();
    while (cloud_filtered->size() > 0.5 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> random(cloud_plane);
        viewer.addPointCloud(cloud_plane, random, "cloud" + std::to_string(count++));
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    
    viewer.addPointCloud(cloud_filtered,"cloud_filtered");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"cloud_filtered");
    viewer.spin();
    return 0;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.01); // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->push_back((*cloud_filtered)[*pit]); //*
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> random(cloud_cluster);
        viewer.addPointCloud(cloud_cluster, random, "cloud" + std::to_string(count++));
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud" + std::to_string(count - 1));
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        j++;
    }

    while (!viewer.wasStopped())
    {
        viewer.spin();
    }

    return (0);
}