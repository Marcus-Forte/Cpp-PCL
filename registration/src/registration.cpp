#include <iostream>
#include <chrono>


#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

using ICP = pcl::IterativeClosestPoint<PointT, PointT>;
using NL_ICP = pcl::IterativeClosestPointNonLinear<PointT, PointT>;
using GICP = pcl::GeneralizedIterativeClosestPoint<PointT, PointT>;
using NDT = pcl::NormalDistributionsTransform<PointT,PointT>;

pcl::Registration<PointT, PointT>::Ptr Registration;


// TODO ALGORITHMS 
// TRIMMING
// POINT TO LINE (Overriding and using numerical diff from eigen)

using namespace std::chrono;

void printUsage()
{
    std::cout << "Usage : registration [target.pcd] [source.pcd] [voxel res] [iterations] [ransac iterations] [ransac threshold] [maxcorr dist] [method (ICP,NLICP,GICP)]" << std::endl;
}

int main(int argc, char **argv)
{

    if (argc < 9)
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

    pcl::IterativeClosestPoint<PointT,PointT> ycp;

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

    // Parameters
    float res = atof(argv[3]);
    int maxit = atoi(argv[4]);
    int ransac_it = atoi(argv[5]);
    int ranscac_thr = atof(argv[6]);
    float maxCorr = atof(argv[7]);

    if (res != 0.0)
    {
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setLeafSize(res, res, res);
        voxel.setInputCloud(cloud_target_);
        voxel.filter(*cloud_target);
        voxel.setInputCloud(cloud_source_);
        voxel.filter(*cloud_source);
    }
    else
    {
        *cloud_target = *cloud_target_;
        *cloud_source = *cloud_source_;
    }

    pcl::visualization::PCLVisualizer viewer("Viewer");

    std::string method_str = argv[8];

    if (method_str == "ICP")
    {
        Registration.reset(new ICP);
    }
    else if (method_str == "NLICP")
    {
        Registration.reset(new NL_ICP);
    }
    else if (method_str == "GICP")
    {
        Registration.reset(new GICP);
    }
    else if(method_str == "NDT"){
        Registration.reset(new NDT);
    }
    else
    {
        std::cerr << "Invalid Method." << std::endl;
        exit(-1);
    }

    Registration->setInputSource(cloud_source);
    Registration->setInputTarget(cloud_target);

    Registration->setTransformationEpsilon(1e-8);
    Registration->setMaximumIterations(maxit);
    Registration->setRANSACIterations(ransac_it);
    Registration->setRANSACOutlierRejectionThreshold(ranscac_thr);
    Registration->setMaxCorrespondenceDistance(maxCorr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    auto start = steady_clock::now();
    Registration->align(*cloud_aligned);
    auto end = steady_clock::now();


    std::cout << "Fitness Score: " << Registration->getFitnessScore() << " Computation time: " << duration_cast<microseconds>(end-start).count() << " us" << std::endl;

    int vp0, vp1;
    viewer.createViewPort(0, 0, 0.5, 1, vp0);
    viewer.createViewPort(0.5, 0, 1, 1, vp1);
    viewer.addPointCloud(cloud_target, "target", 0);
    viewer.addPointCloud(cloud_source, "source", vp0);
    viewer.addPointCloud(cloud_aligned, "aligned", vp1);
    viewer.addCoordinateSystem(1);

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "target");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "source");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "aligned");

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
