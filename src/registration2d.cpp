#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_2D.h>

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

    PointCloudT::Ptr cloud_target_ = pcl::make_shared<PointCloudT>(); //Raw
    PointCloudT::Ptr cloud_target = pcl::make_shared<PointCloudT>();  // Voxel

    PointCloudT::Ptr cloud_icp = pcl::make_shared<PointCloudT>();
    PointCloudT::Ptr cloud_gicp = pcl::make_shared<PointCloudT>();

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
    pcl::PassThrough<pcl::PointXYZ> pass_through;
    // pass_through.setInputCloud(input_clzzz)
    pass_through.setInputCloud(cloud_source_);
    pass_through.setFilterFieldName("x");
    pass_through.setFilterLimits(0, 0.05); // remove weird points close to origin
    pass_through.setNegative(true);
    pass_through.filter(*cloud_source_);

    pass_through.setInputCloud(cloud_target_);
    pass_through.setFilterFieldName("x");
    pass_through.setFilterLimits(0, 0.05); // remove weird points close to origin
    pass_through.setNegative(true);
    pass_through.filter(*cloud_target_);

    // Voxel
    float res = atof(argv[3]);
    int maxit = atoi(argv[4]);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(res, res, res);
    voxel.setInputCloud(cloud_target_);
    voxel.filter(*cloud_target);

    voxel.setInputCloud(cloud_source_);
    voxel.filter(*cloud_source);

    pcl::visualization::PCLVisualizer viewer("Viewer");
    
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);

    pcl::registration::TransformationEstimation2D<pcl::PointXYZ,pcl::PointXYZ>::Ptr te_2d (new pcl::registration::TransformationEstimation2D<pcl::PointXYZ,pcl::PointXYZ>);
    icp.setTransformationEstimation(te_2d);
    icp.setTransformationEpsilon(1e-8);
    icp.setMaximumIterations(maxit);
    // icp.setMaxCorrespondenceDistance(0.1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*cloud_aligned);
    
    int vp0,vp1;
    viewer.createViewPort(0,0,0.5,1,vp0);
    viewer.createViewPort(0.5,0,1,1,vp1);
    viewer.addPointCloud(cloud_target,"target",0);
    viewer.addPointCloud(cloud_source,"source",vp0);
    viewer.addPointCloud(cloud_aligned,"aligned",vp1);
    viewer.addCoordinateSystem(1);

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"target");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,"source");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,0,"aligned");

    while (!viewer.wasStopped()){
        viewer.spin();
    }

 

    if (generateOutput)
    {
        // PointCloudT::Ptr cloud_final = pcl::make_shared<PointCloudT>();
        // std::cout << "Saving aligned point cloud in: '" << output_name << "'" << std::endl;
        // pcl::transformPointCloud(*cloud_source_, *cloud_final, g_icp.getFinalTransformation());
        // *cloud_final = *cloud_final + *cloud_target_;
        // pcl::io::savePCDFile(output_name + ".pcd", *cloud_final);
    }

    // viewer.

    while (!viewer.wasStopped())
    {
        viewer.spin();
    }

    return 0;
}
