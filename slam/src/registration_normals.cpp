#include <iostream>
#include <ctime>

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

using PointInT = pcl::PointXYZI;
using PointCloudIT = pcl::PointCloud<PointInT>;

// with normals
using PointT = pcl::PointXYZINormal;
using PointCloudT = pcl::PointCloud<PointT>;
//

using ICP = pcl::IterativeClosestPoint<PointT, PointT>;
using NL_ICP = pcl::IterativeClosestPointNonLinear<PointT, PointT>;
using GICP = pcl::GeneralizedIterativeClosestPoint<PointT, PointT>;
using NDT = pcl::NormalDistributionsTransform<PointT, PointT>;

pcl::Registration<PointT, PointT>::Ptr Registration;

// TODO ALGORITHMS
// TRIMMING
// POINT TO LINE (Overriding and using numerical diff from eigen)

using namespace std::chrono;

void printUsage()
{
    std::cout << "Usage : registration_normals [target.pcd] [source.pcd] [voxel res] [iterations] [maxcorr dist]" << std::endl;
}

int main(int argc, char **argv)
{

    if (argc < 6)
    {
        printUsage();
        exit(-1);
    }

    PointCloudIT::Ptr cloud_source = pcl::make_shared<PointCloudIT>();  //Voxel
    PointCloudIT::Ptr cloud_source_ = pcl::make_shared<PointCloudIT>(); //Raw

    PointCloudIT::Ptr cloud_target = pcl::make_shared<PointCloudIT>();  // Voxel
    PointCloudIT::Ptr cloud_target_ = pcl::make_shared<PointCloudIT>(); //Raw
    


    bool generateOutput = false;

    std::string output_name;
    if (pcl::console::parse_argument(argc, argv, "-o", output_name) != -1)
        generateOutput = true;

    if (pcl::io::loadPCDFile<PointInT>(argv[1], *cloud_target_) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file model \n");
        return (-1);
    }

    if (pcl::io::loadPCDFile<PointInT>(argv[2], *cloud_source_) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file shape \n");
        return (-1);
    }

    // Prefiltering
    pcl::PassThrough<PointInT> pass_through;
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
    float maxCorr = atof(argv[5]);

    std::cout <<"Res: " << res << std::endl;
    std::cout <<"maxit: " << maxit << std::endl;
    std::cout <<"maxCorr: " << maxCorr << std::endl;

    if (res != 0.0)
    {
        pcl::VoxelGrid<PointInT> voxel;
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

    PointCloudT::Ptr cloud_source_normals (new PointCloudT);
    PointCloudT::Ptr cloud_target_normals (new PointCloudT);

    PCL_INFO("Estimating normals...");
    pcl::NormalEstimation<PointInT, PointT> ne;
    // ne.setInputCloud(cloud_source);
    // ne.setKSearch(10);
    // ne.compute(*cloud_source_normals);

    ne.setInputCloud(cloud_target);
    ne.setKSearch(10);
    ne.compute(*cloud_target_normals);

    pcl::copyPointCloud<PointInT,PointT>(*cloud_source,*cloud_source_normals);
    pcl::copyPointCloud<PointInT,PointT>(*cloud_target,*cloud_target_normals);

    // pcl::concatenateFields(*cloud_source, *cloud_source_normals, *cloud_source_normals);
    // pcl::concatenateFields(*cloud_target, *cloud_target_normals, *cloud_target_normals);

    pcl::visualization::PCLVisualizer viewer("Viewer");

    std::cout << "Checking points" << std::endl;
    for(int i = 0;i < 3; ++i){
        std::cout << "src: " << cloud_source_normals->points[i].x << "| n: " << cloud_source_normals->points[i].normal_x << std::endl;
        std::cout << "src: " << cloud_target_normals->points[i].x << "| n: " << cloud_target_normals->points[i].normal_x << std::endl;
    
    }

    // PointCloudIT::Ptr aligned(new PointCloudIT);
    // pcl::IterativeClosestPoint<PointInT,PointInT> nicp;
    // nicp.setInputSource(cloud_source);
    // nicp.setInputTarget(cloud_target);
    // 0.00867128

    clock_t start = clock();
    PointCloudIT::Ptr aligned (new PointCloudIT);
    pcl::IterativeClosestPointWithNormals<PointInT,PointT> nicp;
    nicp.setInputSource(cloud_source);
    nicp.setInputTarget(cloud_target_normals);
    nicp.setUseReciprocalCorrespondences(false);
    pcl::registration::TransformationEstimationPointToPlaneLLS<PointInT,PointT>::Ptr trans_lls (new pcl::registration::TransformationEstimationPointToPlaneLLS<PointInT,PointT>);
    nicp.setTransformationEstimation(trans_lls);

    
    nicp.setMaximumIterations(maxit);
    nicp.setMaxCorrespondenceDistance(maxCorr);
    nicp.setTransformationEpsilon(1e-8);
    nicp.align(*aligned);
    clock_t end = clock();

    std::cout << "Compute time: " << (float)(end - start) / CLOCKS_PER_SEC << std::endl;

    std::cout << nicp.getFitnessScore() << std::endl;


    int vp0, vp1;
    viewer.createViewPort(0, 0, 0.5, 1, vp0);
    viewer.createViewPort(0.5, 0, 1, 1, vp1);
    viewer.addPointCloud<PointT>(cloud_target_normals,"tgt",0);
    viewer.addPointCloudNormals<PointT>(cloud_target_normals, 10, 0.1, "tgt_normals",0);

    viewer.addPointCloud<PointT>(cloud_source_normals,"src",vp0);
    viewer.addPointCloudNormals<PointT>(cloud_source_normals, 10, 0.1, "src_normals",vp0);

    viewer.addPointCloud<PointInT>(aligned,"aln",vp1);
    // viewer.addPointCloud<PointT>(aligned,"aln",vp1);
    // viewer.addPointCloudNormals<PointT>(cloud_source_normals, 10, 0.1, "src_normals");



    // viewer.addPointCloud(cloud_aligned, "aligned", vp1);

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "src");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "tgt");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "aln");

    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "tgt_normals");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "tgt_normals");

    


    // pcl::IterativeClosestPoint<PointT, PointT> icp;
    // icp.setInputSource(cloud_source);
    // icp.setInputTarget(cloud_target);
    // icp.setMaximumIterations(maxit);
    // icp.setMaxCorrespondenceDistance(maxCorr);
    // icp.setTransformationEpsilon(1e-8);
    // icp.align(*cloud_aligned);

    // pcl::visualization::PCLVisualizer viewer("Viewer");
    // printf("Visualizing ... ");
    // int vp0, vp1;
    // viewer.createViewPort(0, 0, 0.5, 1, vp0);make 
    // viewer.createViewPort(0.5, 0, 1, 1, vp1);
    // viewer.addPointCloud<PointT>(cloud_target, "target", 0);
    // viewer.addPointCloud<PointT>(cloud_source, "source", vp0);
    // viewer.addPointCloud<PointT>(cloud_aligned, "aligned", vp1);
    // viewer.addCoordinateSystem(1);

    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "target");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "source");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "aligned");

    while (!viewer.wasStopped())
    {
        viewer.spin();
    }

    return 0;
}