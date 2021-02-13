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

// TODO Implement Feature-to-plane correspondence

template <typename PointSource, typename PointTarget, typename Scalar = float>
class TransformationPointToLine : public pcl::registration::TransformationEstimation<PointSource, PointTarget, Scalar>
{

public:
    using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;

    using Ptr = shared_ptr<TransformationPointToLine<PointSource, PointTarget, Scalar>>;
    using ConstPtr = shared_ptr<const TransformationPointToLine<PointSource, PointTarget, Scalar>>;

    // called by ICP
    void
    estimateRigidTransformation(
        const pcl::PointCloud<PointSource> &cloud_src,
        const pcl::PointCloud<PointTarget> &cloud_tgt,
        const pcl::Correspondences &correspondences,
        Matrix4 &transformation_matrix) const override
    {
    }

private:
};

using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;

void PointPickCallback(const pcl::visualization::PointPickingEvent &event, void *cookie)
{
    float x, y, z;
    event.getPoint(x, y, z);
    std::cout << "Pt ID: " << event.getPointIndex() << "(" << x << "," << y << "," << z << ")" << std::endl;
}

// OK!

void printUsage()
{
    std::cout << "Usage : registration [target.pcd] [source.pcd] [voxel res] [iterations] [n_highest] [-v visualization]" << std::endl;
}

int main(int argc, char **argv)
{

    if (argc < 6)
    {
        printUsage();
        exit(-1);
    }

    PointCloudT::Ptr cloud_source = pcl::make_shared<PointCloudT>();     //Voxel
    PointCloudT::Ptr cloud_source_raw = pcl::make_shared<PointCloudT>(); //Raw

    PointCloudT::Ptr cloud_target = pcl::make_shared<PointCloudT>();     // Voxel
    PointCloudT::Ptr cloud_target_raw = pcl::make_shared<PointCloudT>(); //Raw

    PointCloudT::Ptr cloud_icp = pcl::make_shared<PointCloudT>();
    PointCloudT::Ptr cloud_gicp = pcl::make_shared<PointCloudT>();

    bool generateOutput = false;

    std::string output_name;
    if (pcl::console::parse_argument(argc, argv, "-o", output_name) != -1)
        generateOutput = true;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud_target_raw) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file model \n");
        return (-1);
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *cloud_source_raw) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file shape \n");
        return (-1);
    }

    // Prefiltering
    pcl::PassThrough<pcl::PointXYZ> pass_through;
    // pass_through.setInputCloud(input_clzzz)
    pass_through.setInputCloud(cloud_source_raw);
    pass_through.setFilterFieldName("x");
    pass_through.setFilterLimits(0, 0.05); // remove weird points close to origin
    pass_through.setNegative(true);
    pass_through.filter(*cloud_source_raw);

    pass_through.setInputCloud(cloud_target_raw);
    pass_through.setFilterFieldName("x");
    pass_through.setFilterLimits(0, 0.05); // remove weird points close to origin
    pass_through.setNegative(true);
    pass_through.filter(*cloud_target_raw);

    // Voxel
    float res = atof(argv[3]);
    int maxit = atoi(argv[4]);
    int n_highest = atoi(argv[5]);

    if (res != 0.0)
    {
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setLeafSize(res, res, res);
        voxel.setInputCloud(cloud_target_raw);
        voxel.filter(*cloud_target);

        voxel.setInputCloud(cloud_source_raw);
        voxel.filter(*cloud_source);
    }
    else
    {
        *cloud_target = *cloud_target_raw;
        *cloud_source = *cloud_source_raw;
    }

    // Partiticon clouds
    int N_Partitions = 4;
    std::vector<PointCloudT> cloud_source_partitions;
    Features::PartitionCloud(*cloud_source, cloud_source_partitions, N_Partitions);
    std::cout << "partition OK.." << cloud_source_partitions.size() << std::endl;

    // std::vector<PointCloudT> cloud_target_partitions;
    // Features::PartitionCloud(*cloud_target, cloud_target_partitions, N_Partitions);
    // std::cout << "partition OK.." << cloud_target_partitions.size() << std::endl;

    //Feature Extraction
    PointCloudT::Ptr cloud_source_features(new PointCloudT); //edges
    PointCloudT::Ptr cloud_source_local_features(new PointCloudT);

    PointCloudT::Ptr cloud_aligned_features(new PointCloudT);

    int N = 7; // Neighboors

    for (int i = 0; i < N_Partitions; ++i)
    {

        Features::EdgeDetection<pcl::PointXYZ>(cloud_source_partitions[i], *cloud_source_local_features, N, n_highest);

        // // Add Edges
        *cloud_source_features += *cloud_source_local_features;

        Features::ComputeSmoothness<pcl::PointXYZ>(cloud_source_partitions[i], *cloud_source_local_features, N, n_highest);

        // Add Smooth Points .. may vary
        *cloud_source_features += *cloud_source_local_features;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_source_features);
    // icp.setInputTarget(cloud_target_features);
    icp.setInputTarget(cloud_target_raw);

    pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>::Ptr te_2d(new pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>);
    icp.setTransformationEstimation(te_2d);
    icp.setTransformationEpsilon(1e-6);
    icp.setMaximumIterations(maxit);
    // icp.setCorr
    // icp.setMaxCorrespondenceDistance(0.2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*cloud_aligned_features);
    Eigen::Matrix4f transform = icp.getFinalTransformation();

    std::cout << "Final Transform" << transform << std::endl;

    pcl::transformPointCloud(*cloud_source, *cloud_aligned, transform);

    pcl::visualization::PCLVisualizer viewer("Viewer");

    int vp0, vp1;
    viewer.createViewPort(0, 0, 0.5, 1, vp0);
    viewer.createViewPort(0.5, 0, 1, 1, vp1);
    viewer.addPointCloud(cloud_target, "target", 0);
    viewer.addPointCloud(cloud_source, "source", vp0);
    viewer.addPointCloud(cloud_aligned, "aligned", vp1);
    viewer.addCoordinateSystem(1, "ref", 0);
    viewer.setCameraPosition(0, 0, 17, 0, 0, 0);
    viewer.setWindowBorders(true);
    viewer.setSize(1750, 880);
    viewer.setPosition(50, 120);
    // viewer.registerPointPickingCallback(boost::bind(PointPickCallback,_1,_2));
    viewer.registerPointPickingCallback(PointPickCallback);
    //Edges
    // viewer.addPointCloud(cloud_target_features, "target features", 0);
    viewer.addPointCloud(cloud_source_features, "source features", vp0);
    viewer.addPointCloud(cloud_aligned_features, "aligned features", vp1);

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "target");  //red
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "source");  //green
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "aligned"); // yellow

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "source features");  // green
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "aligned features"); // yellow

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "source features");
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
