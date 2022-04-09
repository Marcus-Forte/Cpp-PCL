#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/visualization/pcl_visualizer.h>

#ifndef DATASET_PATH
#error "DATASET PATH NOT DEFINED"
#endif

using PointT = pcl::PointXYZINormal;
using PointCloudT = pcl::PointCloud<PointT>;

using ColorHandler = pcl::visualization::PointCloudColorHandler<PointT>;
using ColorHandlerPtr = ColorHandler::Ptr;
using ColorHandlerConstPtr = ColorHandler::ConstPtr;

#define MAP_N_SCANS 25
#define SCAN_N 10

void draw_corrs(pcl::visualization::PCLVisualizer &viewer, const pcl::Correspondences &corrs, const PointCloudT::ConstPtr &source, const PointCloudT::ConstPtr &target, int detail = 5, int viewport = 0)
{
    for (int i = 0; i < corrs.size(); i += detail)
    {
        const PointT &src = source->points[corrs[i].index_query];
        const PointT &tgt = target->points[corrs[i].index_match];

        viewer.addLine(src, tgt, 0, 1, 0, "arrow" + std::to_string(i), viewport);
    }
}

int main(int argc, char **argv)
{
    PointCloudT::Ptr input_map = pcl::make_shared<PointCloudT>();
    PointCloudT::Ptr input_scan = pcl::make_shared<PointCloudT>();

    // Build map dataset
    for (int i = 1; i < MAP_N_SCANS; ++i)
    {
        PointCloudT map_;
        pcl::io::loadPCDFile(DATASET_PATH "map" + std::to_string(i) + ".pcd", map_);
        *input_map += map_;
    }

    // Pick scan
    pcl::io::loadPCDFile(DATASET_PATH "scan" + std::to_string(SCAN_N) + ".pcd", *input_scan);

    PCL_INFO("Loaded map : %ld points\n", input_map->size());
    PCL_INFO("Loaded scan : %ld points\n", input_scan->size());

    PointCloudT::Ptr input_map_downsampled(new PointCloudT);
    PointCloudT::Ptr input_scan_downsampled(new PointCloudT);

    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(0.05, 0.05, 0.05);

    voxel.setInputCloud(input_map);
    voxel.filter(*input_map_downsampled);

    voxel.setInputCloud(input_scan);
    voxel.filter(*input_scan_downsampled);

    PCL_INFO("Downsampled map : %ld points\n", input_map_downsampled->size());
    PCL_INFO("Downsampled scan : %ld points\n", input_scan_downsampled->size());

    pcl::visualization::PCLVisualizer viewer("viewer");
    int vp0, vp1;
    viewer.createViewPort(0, 0, 0.5, 1.0, vp0);
    // viewer.createViewPort(0.5,0,1.0,1,vp1);

    ColorHandlerPtr color_handler;
    //
    // pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler_white(1.0,1.0,1.0);
    color_handler.reset(new pcl::visualization::PointCloudColorHandlerGenericField<PointT>(input_map, "intensity"));
    viewer.addPointCloud<PointT>(input_map, *color_handler, "cloud", vp0);
    viewer.addPointCloud<PointT>(input_scan, "scan", vp0);

    // Estimate normals
    std::cout << "Computing normals...\n";
    pcl::NormalEstimation<PointT, PointT> ne;
    ne.setInputCloud(input_map);
    ne.setRadiusSearch(0.01);
    ne.compute(*input_map);
    std::cout << "done.\n";

    pcl::PointCloud<pcl::PFHSignature125>::Ptr map_features (new pcl::PointCloud<pcl::PFHSignature125>);

    std::cout << "Computing features...\n";
    pcl::PFHEstimation<PointT, PointT, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(input_map_downsampled);
    pfh.setInputNormals(input_map);
    pfh.setSearchSurface(input_map);
    pfh.setRadiusSearch(0.02);
    pfh.compute(*map_features);
    std::cout << "done.\n";

    pcl::registration::CorrespondenceEstimation<PointT, PointT> estimator;
    estimator.setInputSource(input_scan_downsampled);
    estimator.setInputTarget(input_map_downsampled);
    pcl::Correspondences corrs;
    estimator.determineCorrespondences(corrs);






    draw_corrs(viewer, corrs, input_scan_downsampled, input_map_downsampled, 2, vp0);

    while (!viewer.wasStopped())
    {
        viewer.spin();
    }
}