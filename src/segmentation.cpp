#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration_2d.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <thread>

using PointT = pcl::PointXYZ;


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
    pcl::PointCloud<PointT>::Ptr output = pcl::make_shared<pcl::PointCloud<PointT>>();

    if (pcl::io::loadPCDFile(cloud_file, *cloud) == -1)
    {
        PCL_ERROR("Error loading point cloud.");
        exit(-1);
    }

    PCL_INFO("N points -> %d\n", cloud->size());

    clock_t start, elapsed;
    pcl::Indices inliers;
    start = clock();
    pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointT>(cloud));
    pcl::RandomSampleConsensus<PointT> ransac(model_p);
    ransac.setNumberOfThreads(2);
    ransac.setMaxIterations(500);
    ransac.setDistanceThreshold(dist_thres);
    ransac.computeModel();
    ransac.getInliers(inliers);
    elapsed = clock() - start;
    printf("Segmentation (%f secs)\n ", (float)elapsed / CLOCKS_PER_SEC);

    pcl::copyPointCloud(*cloud,inliers,*output);


    pcl::visualization::PCLVisualizer viewer("viewer");
    viewer.addPointCloud(cloud,"in");

    viewer.addPointCloud(output,"seg");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"seg");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"seg");


    while(!viewer.wasStopped()){
        viewer.spin();
    }




    // PCL_INFO("Inliers -> %d\n", inliers->size());
    // PCL_INFO("Extracted -> %d\n", cloud_extract->size());
    // PCL_INFO("Remain -> %d\n", cloud->size());

    return 0;
}