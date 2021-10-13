#include <pcl/registration/gicp6d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

using PointT = pcl::PointXYZRGBA;
using PointCloudT = pcl::PointCloud<PointT>;


void printError(char **argv)
{
    PCL_ERROR("useage : %s [src] [map] \n", argv[0]);
    exit(-1);
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        printError(argv);
    }

    pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_VERBOSE);

    PointCloudT::Ptr target_(new PointCloudT);
    PointCloudT::Ptr source_(new PointCloudT);

    if (pcl::io::loadPCDFile(argv[1], *source_) == -1)
    {
        PCL_ERROR("Error Opening source cloud\n");
        exit(-1);
    }

    if (pcl::io::loadPCDFile(argv[2], *target_) == -1)
    {
        PCL_ERROR("Error Opening map cloud\n");
        exit(-1);
    }

    // preprocess
    size_t filter_in, filter_out;
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(source_);
    voxel.setLeafSize(0.1,0.1,0.1);
    filter_in = source_->size();
    voxel.filter(*source_);
    filter_out = source_->size();
    PCL_INFO("Voxel source %d -> %d\n", filter_in,filter_out);

    voxel.setInputCloud(target_);
    filter_in = target_->size();
    voxel.filter(*target_);
    filter_out = target_->size();
    PCL_INFO("Voxel target %d -> %d\n", filter_in,filter_out);


    pcl::GeneralizedIterativeClosestPoint6D gicp6d;

    gicp6d.setInputSource(source_);
    gicp6d.setInputTarget(target_);
    gicp6d.setMaxCorrespondenceDistance(1);
    gicp6d.setTransformationEpsilon(1e-5);
    PointCloudT::Ptr output = pcl::make_shared<PointCloudT>();
    gicp6d.align(*output);


    Eigen::Matrix4f transform = gicp6d.getFinalTransformation();
    std::cout << transform << std::endl;


    pcl::visualization::PCLVisualizer viewer;
    int v0,v1;
    viewer.createViewPort(0,0,0.5,1,v0);
    viewer.createViewPort(0.5,0,1,1,v1);

    viewer.addPointCloud(target_,"tgt");
    viewer.addPointCloud(source_,"src",v0);
    viewer.addPointCloud(output,"out",v1);



    while (!viewer.wasStopped())
    {
        viewer.spin();
        /* code */
    }
    
    




    return 0;
}