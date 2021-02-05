#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>

#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;

void printUsage()
{
    std::cout << "Usage : registration [target.pcd] [source.pcd] [voxel res] [iterations] [-v visualization]" << std::endl;
}

int main(int argc, char **argv)
{

    if (argc < 5)
    {
        printUsage();
        exit(-1);
    }

    PointCloudT::Ptr cloud_source = pcl::make_shared<PointCloudT>();  //Voxel
    PointCloudT::Ptr cloud_source_ = pcl::make_shared<PointCloudT>(); //Raw


    PointCloudT::Ptr cloud_target = pcl::make_shared<PointCloudT>();  // Voxel
    PointCloudT::Ptr cloud_target_ = pcl::make_shared<PointCloudT>(); //Raw

    PointCloudT::Ptr cloud_icp = pcl::make_shared<PointCloudT>();
    PointCloudT::Ptr cloud_gicp = pcl::make_shared<PointCloudT>();
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_nicp = pcl::make_shared<pcl::PointCloud<pcl::PointNormal>>();

    bool generateOutput = false;

    std::string output_name;
    if (pcl::console::parse_argument(argc, argv, "-o", output_name) != -1)
        generateOutput = true;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud_target_) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file model \n");
        return (-1);
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *cloud_source_) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file shape \n");
        return (-1);
    }

    // Prefiltering

    // Voxel
    float res = atof(argv[3]);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(res, res, res);
    voxel.setInputCloud(cloud_target_);
    voxel.filter(*cloud_target);

    voxel.setInputCloud(cloud_source_);
    voxel.filter(*cloud_source);

    // MeanK = 200, StdThresh = 5 for stockpile
    // std::cout << "Computing SOR..." << std::endl;
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // sor.setMeanK(50);
    // sor.setStddevMulThresh(0.1);

    // sor.setInputCloud(cloud_target);
    // sor.filter(*cloud_target);

    // sor.setInputCloud(cloud_source);
    // sor.filter(*cloud_source);

    
    
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal,pcl::PointNormal> n_icp;
    
    

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    std::cout << "Computing ICP..." << std::endl;
    int maxit = atoi(argv[4]);
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);
    icp.setMaximumIterations(maxit);
    icp.align(*cloud_icp);

    std::cout << "Score: " << icp.getFitnessScore() << std::endl;
    std::cout << "Has Converged : " << icp.hasConverged() << std::endl;



    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> g_icp;
    std::cout << "Computing GICP..." << std::endl;
    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> g_icp_ne_source;
    
    // pcl::PointCloud<pcl::Normal>::Ptr g_icp_source_normals = pcl::make_shared<pcl::PointCloud<pcl::Normal>>();
    // pcl::PointCloud<pcl::PointNormal>::Ptr g_icp_cloud_source_normals = pcl::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr g_icp_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // g_icp_tree->setInputCloud(cloud_source);
    // ne_source.setInputCloud(cloud_source);
    // ne_source.setSearchMethod(g_icp_tree);
    // ne_source.setKSearch(5);
    // ne_source.setViewPoint(0, 0, -1);

    // ne_source.compute(*source_normals);

    // pcl::concatenateFields(*cloud_source, *source_normals, *cloud_source_normals);


    g_icp.setMaxCorrespondenceDistance(0.1);
    g_icp.setRotationEpsilon(0.1);
    g_icp.setInputSource(cloud_source);
    g_icp.setInputTarget(cloud_target);
    g_icp.setMaximumIterations(maxit);
    g_icp.align(*cloud_gicp);

    std::cout << "Score: " << g_icp.getFitnessScore() << std::endl;
    std::cout << "Has Converged : " << g_icp.hasConverged() << std::endl;

    std::cout << "ICP Transformation" << std::endl
              << icp.getFinalTransformation() << std::endl;
    std::cout << "NICP Transformation" << std::endl
              << n_icp.getFinalTransformation() << std::endl;              
    std::cout << "GICP Transformation" << std::endl
              << g_icp.getFinalTransformation() << std::endl;


    pcl::visualization::PCLVisualizer viewer("Viewer");
    int v0, v1; //Viewports
    viewer.createViewPort(0, 0, 0.5, 1, v0);
    viewer.createViewPort(0.5, 0, 1, 1, v1);
    viewer.setCameraPosition(0, 0, -2, 0, -1, 0);
    viewer.addPointCloud(cloud_target, "cloud_target");
    viewer.addPointCloud(cloud_source, "cloud_source", v0);
    viewer.addPointCloud(cloud_icp, "cloud_icp", v1);
    viewer.addPointCloud(cloud_gicp, "cloud_gicp", v1);

    viewer.addCoordinateSystem(0.1, "ref");

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_source"); // R G B
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_target");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "cloud_icp");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud_gicp");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud_nicp");
    

    if (generateOutput)
    {
        PointCloudT::Ptr cloud_final = pcl::make_shared<PointCloudT>();
        std::cout << "Saving aligned point cloud in: '" << output_name << "'" << std::endl;
        pcl::transformPointCloud(*cloud_source_, *cloud_final, g_icp.getFinalTransformation());
        *cloud_final = *cloud_final + *cloud_target_;
        pcl::io::savePCDFile(output_name + ".pcd", *cloud_final);
    }

    // viewer.

    while (!viewer.wasStopped())
    {
        viewer.spin();
    }

    return 0;
}
