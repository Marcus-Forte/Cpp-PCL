#include "duna/feature_registration.h"
#include "pcl/registration/icp.h"

#include <pcl/filters/voxel_grid.h>
#include "pcl/features/normal_3d.h"
#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/visualization/registration_visualizer.h>

using PointT = pcl::PointXYZINormal;
using PointCloudT = pcl::PointCloud<PointT>;

int main(int argc, char **argv)
{

    pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_VERBOSE);

    PCL_DEBUG("DEBUG ON!");

    PointCloudT::Ptr source(new PointCloudT);
    PointCloudT::Ptr corner_map(new PointCloudT);
    PointCloudT::Ptr surface_map(new PointCloudT);
    PointCloudT::Ptr full_source(new PointCloudT);

    if (argc < 4)
    {
        PCL_ERROR("use : %s [src] [corner map] [surf map]\n", argv[0]);
        exit(-1);
    }
    if (pcl::io::loadPCDFile(argv[1], *full_source) == -1)
    {
        PCL_ERROR("Error Opening source cloud\n");
        exit(-1);
    }

    if (pcl::io::loadPCDFile(argv[2], *corner_map) == -1)
    {
        PCL_ERROR("Error Opening corner map cloud\n");
        exit(-1);
    }

    if (pcl::io::loadPCDFile(argv[3], *surface_map) == -1)
    {
        PCL_ERROR("Error Opening surface cloud\n");
        exit(-1);
    }

    PointCloudT::Ptr corners(new PointCloudT);
    PointCloudT::Ptr surfaces(new PointCloudT);

    // duna::extractFeatures<PointT>(*full_source,*corners, *surfaces);
    duna::Registration<PointT>::extractFeatures(*full_source, *corners, *surfaces);

    pcl::search::KdTree<PointT>::Ptr corners_tree(new pcl::search::KdTree<PointT>);
    pcl::search::KdTree<PointT>::Ptr surfaces_tree(new pcl::search::KdTree<PointT>);

    corners_tree->setInputCloud(corners);
    surfaces_tree->setInputCloud(surfaces);

    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(surface_map);
    voxel.setLeafSize(0.1, 0.1, 0.1);
    voxel.filter(*surface_map);

    pcl::NormalEstimation<PointT, PointT> ne;
    ne.setInputCloud(surface_map);
    ne.setKSearch(5);
    // ne.compute(*surface_map);

    duna::Registration<PointT> registration;
    registration.setMaxCorrDist(7);
    registration.setInputCorners(corners);
    registration.setInputSurfaces(surfaces);
    registration.setTargetCorners(corner_map);
    registration.setTargetSurfaces(surface_map);
    registration.setMaximumIterations(100);
    registration.setTargetCornersSearchMethod(corners_tree, true);
    registration.setTargetSurfacesSearchMethod(surfaces_tree, true);
    // Set Parameters
    Eigen::Matrix4f guess;
    guess << 1, 0, 0, 24, 0, 1, 0, 0.3, 0, 0, 1, -1.5, 0, 0, 0, 1;
    clock_t start = clock();
    registration.align(guess);
    clock_t elapsed = clock() - start;

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(surfaces);
    icp.setInputTarget(surface_map);
    icp.setMaximumIterations(100);
    icp.setMaxCorrespondenceDistance(5);
    icp.setTransformationEpsilon(1e-6);
    pcl::registration::TransformationEstimation<PointT, PointT>::Ptr transform_;
    transform_.reset(new pcl::registration::TransformationEstimationPointToPlaneLLS<PointT, PointT>);
    PointCloudT surface_tf_icp;
    // icp.align(surface_tf_icp, guess);
    Eigen::MatrixX4f final_transform_icp = icp.getFinalTransformation();

    Eigen::MatrixX4f final_transform = registration.getFinalTransformation();

    std::cout << "Align time: " << (float)elapsed / CLOCKS_PER_SEC << std::endl;
    std::cout << final_transform << std::endl;
    std::cout << final_transform_icp << std::endl;

    // FULL
    //     0.994109   -0.0927296     0.056173      28.2156
    //    0.0929155     0.995676 -0.000701644     0.619082
    //   -0.0558637   0.00592019     0.998421     -2.19996
    //            0            0            0            1

    // SURF
    //    0.99386 -0.0983837  0.0507122     28.359
    //  0.0977221   0.995098  0.0153708   0.560752
    // -0.0519744 -0.0103183   0.998596   -2.22317
    //          0          0          0          1

    pcl::visualization::PCLVisualizer viewer("viewer");

    PointCloudT::Ptr corners_tf(new PointCloudT);
    PointCloudT::Ptr surfaces_tf(new PointCloudT);

    // pcl::transformPointCloud(*corners,*corners_tf,final_transform);
    pcl::transformPointCloud(*surfaces, *surfaces_tf, final_transform_icp);

    viewer.addPointCloud<pcl::PointXYZINormal>(corner_map, "corner_map");
    viewer.addPointCloud<pcl::PointXYZINormal>(surface_map, "surface_map");

    // viewer.addPointCloud<pcl::PointXYZINormal>(corners_tf,"corners_tf");
    viewer.addPointCloud<pcl::PointXYZINormal>(surfaces_tf, "surfaces_tf");
    
    viewer.addPointCloudNormals<pcl::PointXYZINormal>(surface_map, 1, 0.1, "surface_normals");
    // viewer.addPointCloud<pcl::PointXYZINormal>(corner_map,"corner_map");

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "corner_map");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "surface_map");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "corner_map");

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "surfaces_tf");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "surfaces_tf");
    
    viewer.spin();

    return 0;
}