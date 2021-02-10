#include <iostream>
// #include "PCUtils.h"
#include "Features.h"

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

#include <pcl/filters/passthrough.h>

using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;

void PointPickCallback(const pcl::visualization::PointPickingEvent& event, void* cookie){
    float x,y,z;
    event.getPoint(x,y,z);
std::cout << "Pt ID: " << event.getPointIndex() << "(" << x << "," << y << "," << z << ")" << std::endl;

}

void partition_cloud(const PointCloudT &input, std::vector<PointCloudT> &partitions, int N)
{
    int total_size = input.size();
    int block_size = total_size / N;

    PointCloudT PC;
    int block_count = 0;
    int block_index = 0;
    for (int i = 0; i < total_size; ++i)
    {
        PC.push_back(input.points[i]);
        block_count++;
        if (block_count > block_size)
        {
            std::cout << "Pushing.. " << block_index << std::endl;
            partitions.push_back(PC);
            PC.clear();
            block_count = 0;
            block_index++;
        }
    }
    partitions.push_back(PC); // dumb
}

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

    // Partiticon clouds
    int N_Partitions = 1;
    std::vector<PointCloudT> cloud_source_partitions;
    partition_cloud(*cloud_source, cloud_source_partitions, N_Partitions);
    std::cout << "partition OK.." << cloud_source_partitions.size() << std::endl;

    std::vector<PointCloudT> cloud_target_partitions;
    partition_cloud(*cloud_target, cloud_target_partitions, N_Partitions);
    std::cout << "partition OK.." << cloud_target_partitions.size() << std::endl;


    //Feature Extraction
    PointCloudT::Ptr cloud_source_features(new PointCloudT); //edges
    PointCloudT::Ptr cloud_target_features(new PointCloudT);

    PointCloudT::Ptr cloud_source_features2(new PointCloudT); //planes
    PointCloudT::Ptr cloud_target_features2(new PointCloudT);

    for (int i = 0; i < N_Partitions; ++i)
    {
        int N = 7; // Neighboors
        
        // Features::EdgeDetection<pcl::PointXYZ>(cloud_source_partitions[i], *cloud_source_features2, N);
        // Features::EdgeDetection<pcl::PointXYZ>(cloud_target_partitions[i], *cloud_target_features2, N);

        // // Add Edges
        // *cloud_source_features += *cloud_source_features2;
        // *cloud_target_features += *cloud_target_features2;

        std::cout << "Source..." << std::endl;
        Features::ComputeSmoothness<pcl::PointXYZ>(cloud_source_partitions[i], *cloud_source_features2, N, 4);
        std::cout << "Target..." << std::endl;
        Features::ComputeSmoothness<pcl::PointXYZ>(cloud_target_partitions[i], *cloud_target_features2, N, 4);

        // Add Planes
        *cloud_source_features += *cloud_source_features2;
        *cloud_target_features += *cloud_target_features2;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_source_features);
    icp.setInputTarget(cloud_target_features);

    // pcl::registration::TransformationEstimation2D<pcl::PointXYZ,pcl::PointXYZ>::Ptr te_2d (new pcl::registration::TransformationEstimation2D<pcl::PointXYZ,pcl::PointXYZ>);
    // icp.setTransformationEstimation(te_2d);
    icp.setTransformationEpsilon(1e-8);
    icp.setMaximumIterations(maxit);
    // icp.setMaxCorrespondenceDistance(0.2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*cloud_aligned);
    Eigen::Matrix4f transform = icp.getFinalTransformation();

    PointCloudT::Ptr cloud_aligned_features(new PointCloudT);
    *cloud_aligned_features = *cloud_aligned;

    pcl::transformPointCloud(*cloud_source, *cloud_aligned, transform);

    pcl::visualization::PCLVisualizer viewer("Viewer");

    int vp0, vp1;
    viewer.createViewPort(0, 0, 0.5, 1, vp0);
    viewer.createViewPort(0.5, 0, 1, 1, vp1);
    viewer.addPointCloud(cloud_target, "target", 0);
    viewer.addPointCloud(cloud_source, "source", vp0);
    viewer.addPointCloud(cloud_aligned, "aligned", vp1);
    viewer.addCoordinateSystem(1, "ref", 0);
    // viewer.registerPointPickingCallback(boost::bind(PointPickCallback,_1,_2));
    viewer.registerPointPickingCallback(PointPickCallback);
    //Edges
    viewer.addPointCloud(cloud_target_features, "target features", 0);
    viewer.addPointCloud(cloud_source_features, "source features", vp0);
    viewer.addPointCloud(cloud_aligned_features, "aligned features", vp1);
    // Planes
    viewer.addPointCloud(cloud_target_features2, "target features 2", 0);
    viewer.addPointCloud(cloud_source_features2, "source features 2", vp0);
    // viewer.addPointCloud(cloud_aligned_features, "aligned features", vp1);

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "target");  //red
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "source");  //green
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "aligned"); // yellow

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "target features");  // red
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "source features");  // green
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "aligned features"); // yellow

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "target features 2"); // red
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "source features 2"); // green

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "source features");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "target features");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 14, "source features 2");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 14, "target features 2");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "aligned features");

    while (!viewer.wasStopped())
    {
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
