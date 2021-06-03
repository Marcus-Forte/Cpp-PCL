#include <iostream>

#include <pcl/registration/registration.h>
// #include <pcl/registration/icp.h>
// #include <pcl/registration/icp_nl.h>
// #include <pcl/registration/ndt.h>
// #include <pcl/registration/pyramid_feature_matching.h>
// #include <pcl/registration/gicp.h>
// #include <pcl/registration/gicp6d.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_3point.h>
#include <pcl/console/print.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>

#include <pcl/features/normal_3d.h>

#include <pcl/io/pcd_io.h>

// #include "my/my_transform.hpp"
#include "marcus/transform_normals.h"
#include "marcus/icp.h"


#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/registration_visualizer.h>

#include <ctime>

using PointT = pcl::PointXYZINormal;
using PointCloudT = pcl::PointCloud<PointT>;

void keyCallback(const pcl::visualization::KeyboardEvent &event, void *cookie)
{
    pcl::visualization::PCLVisualizer *viewer = (pcl::visualization::PCLVisualizer *)cookie;
    pcl::visualization::Camera c0;
    viewer->getCameraParameters(c0);

    // orientation
    float dx = c0.focal[0] - c0.pos[0];
    float dy = c0.focal[1] - c0.pos[1];
    float dz = c0.focal[2] - c0.pos[2];

    float norm = sqrtf(dx * dx + dy * dy + dz * dz);
    dx /= norm;
    dy /= norm;
    dz /= norm;
    // std::cout << dx << "," << dy << "," << dz << std::endl;
    float vel = 1;

    std::string keyName = event.getKeySym();

    if (keyName == "Up" && event.keyDown())
    {
        c0.pos[0] += dx * vel;
        c0.pos[1] += dy * vel;
        c0.pos[2] += dz * vel;

        c0.focal[0] = c0.pos[0] + dx * 2;
        c0.focal[1] = c0.pos[1] + dy * 2;
        c0.focal[2] = c0.pos[2] + dz * 2;

        viewer->setCameraParameters(c0);
    }
    else if (keyName == "Down" && event.keyDown())
    {
        c0.pos[0] -= dx * vel;
        c0.pos[1] -= dy * vel;
        c0.pos[2] -= dz * vel;

        c0.focal[0] = c0.pos[0] + dx * 2;
        c0.focal[1] = c0.pos[1] + dy * 2;
        c0.focal[2] = c0.pos[2] + dz * 2;
        viewer->setCameraParameters(c0);
    }
    else if (keyName == "Left" && event.keyDown())
    {
    }
    else if (keyName == "Right" && event.keyDown())
    {
    }
}

int main(int argc, char **argv)
{

    pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_VERBOSE);

    PCL_DEBUG("DEBUG ON!");

    PointCloudT::Ptr source(new PointCloudT);
    PointCloudT::Ptr target(new PointCloudT);
    PointCloudT::Ptr full_source(new PointCloudT);

    if (argc < 5)
    {
        PCL_ERROR("use : %s [src] [tgt] [nearestK] [ran samp factor]\n", argv[0]);
        exit(-1);
    }
    if (pcl::io::loadPCDFile(argv[1], *full_source) == -1)
    {
        PCL_ERROR("Error Opening source cloud\n");
        exit(-1);
    }

    if (pcl::io::loadPCDFile(argv[2], *target) == -1)
    {
        PCL_ERROR("Error Opening target cloud\n");
        exit(-1);
    }

    // Pre Filtering
    float rand_samp_factor = atof(argv[4]);
    pcl::RandomSample<PointT> ransamp;
    ransamp.setInputCloud(full_source);
    ransamp.setSample(rand_samp_factor * full_source->size());
    ransamp.filter(*source);

    int nearestK = atoi(argv[3]);

    PCL_INFO("src pts : %d\n", source->size());
    PCL_INFO("tgt pts : %d\n", target->size());

    clock_t start, elapsed;

    PointCloudT::Ptr aligned_gn(new PointCloudT); // my transform
    PointCloudT::Ptr aligned(new PointCloudT);    // pcl transform

    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZINormal>);

    tree->setInputCloud(target);

    pcl::NormalEstimation<PointT, PointT> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(target);
    ne.setKSearch(nearestK);
    ne.compute(*target);

    // this registration computes normals internally and updates the target
    MyRegistration<PointT, PointT> reg(nearestK);
    reg.setSearchMethodTarget(tree, true);
    reg.setInputSource(source);
    reg.setInputTarget(target);
    reg.setMaxCorrespondenceDistance(5);
    reg.setMaximumIterations(200);
    reg.setTransformationEpsilon(1e-6);

    pcl::registration::TransformationEstimation<PointT, PointT>::Ptr transform_;

    Eigen::Matrix4f guess;
    Eigen::Matrix4f aligned_gn_transform;
    Eigen::Matrix4f aligned_transform;
    guess << 1, 0, 0, 24, 0, 1, 0, 0.3, 0, 0, 1, -1.5, 0, 0, 0, 1;
    // guess.setIdentity();
    // std::cout << guess << std::endl;

    // start = clock();
    // transform_.reset(new pcl::registration::TransformationEstimationSVD<PointT, PointT>);
    // reg.setTransformationEstimation(transform_);
    // reg.align(*aligned, guess);
    // elapsed = clock() - start;
    // PCL_INFO("Alignment time SVD: %f\n", (float)elapsed / CLOCKS_PER_SEC);
    // PCL_INFO("Fitness: %f\n", reg.getFitnessScore());

    // start = clock();
    // transform_.reset(new MyTransform<PointT, PointT>);
    // reg.setTransformationEstimation(transform_);
    // reg.align(*aligned_gn);
    // elapsed = clock() - start;
    // PCL_INFO("Alignment time G-N: %f\n", (float)elapsed / CLOCKS_PER_SEC);
    // PCL_INFO("Fitness: %f\n", reg.getFitnessScore());

    start = clock();
    transform_.reset(new pcl::registration::TransformationEstimationPointToPlane<PointT, PointT>);
    reg.setTransformationEstimation(transform_);
    reg.align(*aligned, guess);
    elapsed = clock() - start;
    PCL_INFO("Alignment time LM Normals: %f\n", (float)elapsed / CLOCKS_PER_SEC);
    PCL_INFO("Fitness: %f\n", reg.getFitnessScore());

    start = clock();
    transform_.reset(new pcl::registration::TransformationEstimationPointToPlaneLLS<PointT, PointT>);
    reg.setTransformationEstimation(transform_);
    reg.align(*aligned, guess);
    elapsed = clock() - start;
    aligned_transform = reg.getFinalTransformation();
    PCL_INFO("Alignment time: Normals LLS %f\n", (float)elapsed / CLOCKS_PER_SEC);
    PCL_INFO("Fitness: %f\n", reg.getFitnessScore());

    start = clock();
    transform_.reset(new pcl::registration::TransformationEstimationDualQuaternion<PointT, PointT>);
    reg.setTransformationEstimation(transform_);
    reg.align(*aligned, guess);
    elapsed = clock() - start;
    PCL_INFO("Alignment time: Dual Quat %f\n", (float)elapsed / CLOCKS_PER_SEC);
    PCL_INFO("Fitness: %f\n", reg.getFitnessScore());

    start = clock();
    transform_.reset(new MyTransformNormals<PointT, PointT>);
    reg.setTransformationEstimation(transform_);
    reg.align(*aligned_gn, guess);
    aligned_gn_transform = reg.getFinalTransformation();
    elapsed = clock() - start;
    PCL_INFO("Alignment time G-N Normals: %f\n", (float)elapsed / CLOCKS_PER_SEC);
    PCL_INFO("Fitness: %f\n", reg.getFitnessScore());

    reg.hasConverged() ? PCL_INFO("Converged\n") : PCL_INFO("Not Converged\n");

    // Visualize
    pcl::transformPointCloud(*full_source, *aligned, aligned_transform);
    pcl::transformPointCloud(*full_source, *aligned_gn, aligned_gn_transform);

    pcl::visualization::PCLVisualizer viewer("Viewer");
    pcl::visualization::PCLVisualizer viewer_big("Viewer_big");
    // std::cin.get();

    int vp0, vp1, vp2;
    viewer.createViewPort(0, 0, 0.33, 1, vp0);
    viewer.createViewPort(0.33, 0, 0.66, 1, vp1);
    viewer.createViewPort(0.66, 0, 1, 1, vp2);
    viewer.addPointCloud<PointT>(target, "target", 0);
    viewer.addPointCloudNormals<PointT>(target, 50, 0.1, "target_n", 0);

    pcl::transformPointCloud(*full_source, *full_source, guess);
    viewer.addPointCloud<PointT>(full_source, "source", vp0);
    viewer.addPointCloud<PointT>(aligned, "aligned", vp1);
    viewer.addPointCloud<PointT>(aligned_gn, "aligned_gn", vp2);

    viewer.addCoordinateSystem(1);

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "target");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "source");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "aligned");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "aligned_gn");

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "aligned");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "aligned_gn");

    viewer.registerKeyboardCallback(keyCallback, &viewer);

    viewer_big.addPointCloud<PointT>(full_source, "source");
    viewer_big.addPointCloud<PointT>(aligned_gn, "aligned_gn");
    viewer_big.addPointCloud<PointT>(target, "target");

    viewer_big.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "target");
    viewer_big.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "source");
    viewer_big.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "aligned_gn");
    viewer_big.registerKeyboardCallback(keyCallback,&viewer_big);


    while (!viewer.wasStopped() && !viewer_big.wasStopped())
    {
        viewer.spinOnce(500);
        viewer_big.spinOnce(500);
    }
}