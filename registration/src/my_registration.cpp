#include <iostream>

#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/pyramid_feature_matching.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_3point.h>
#include <pcl/console/print.h>

#include <pcl/features/normal_3d.h>

#include <pcl/io/pcd_io.h>

#include "my/my_transform.hpp"
#include "my/my_transform_normals.hpp"
#include "my/my_icp.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/registration_visualizer.h>

#include <ctime>

using PointT = pcl::PointXYZINormal;
using PointCloudT = pcl::PointCloud<PointT>;

class Parent
{
public:
protected:
    int prot_m;
};

class Child : public Parent
{

    // using Parent::prot_m;
public:
    int getProt()
    {
        return prot_m;
    }

protected:
};



int main(int argc, char **argv)
{

    pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_VERBOSE);

    PCL_DEBUG("DEBUG ON!");

    PointCloudT::Ptr source(new PointCloudT);
    PointCloudT::Ptr target(new PointCloudT);

    if (argc < 4)
    {
        PCL_ERROR("use : %s [src] [tgt] [nearestK]\n", argv[0]);
        exit(-1);
    }
    if (pcl::io::loadPCDFile(argv[1], *source) == -1)
    {
        PCL_ERROR("Error Opening source cloud\n");
        exit(-1);
    }

    if (pcl::io::loadPCDFile(argv[2], *target) == -1)
    {
        PCL_ERROR("Error Opening target cloud\n");
        exit(-1);
    }

    int nearestK = atoi(argv[3]);

    PCL_INFO("src pts : %d\n", source->size());
    PCL_INFO("tgt pts : %d\n", target->size());

    clock_t start, elapsed;

    PointCloudT::Ptr aligned_gn(new PointCloudT);
    PointCloudT::Ptr aligned(new PointCloudT);

    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZINormal>);

    tree->setInputCloud(target);
    pcl::NormalEstimation<PointT, PointT> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(target);
    ne.setKSearch(nearestK);
    ne.compute(*target);
 

    MyRegistration<PointT,PointT> reg(nearestK);
    reg.setSearchMethodTarget(tree,true);
    
    // reg.setSearchMethodTarget(tree,true);
    // pcl::IterativeClosestPoint<PointT, PointT> reg;
    // pcl::NormalDistributionsTransform<PointT,PointT> reg;
    // MyICP<PointT, PointT> reg;
    // PCL_INFO("%s\n", reg.reg_name_);
    // pcl::GeneralizedIterativeClosestPoint<PointT, PointT> reg;
    // pcl::registration::FPCSInitialAlignment<PointT,PointT> reg;

    reg.setInputSource(source);
    reg.setInputTarget(target);
    // visualizer.setRegistration(icp);
    reg.setMaxCorrespondenceDistance(0.5);
    reg.setMaximumIterations(100);
    reg.setTransformationEpsilon(1e-8);

    // reg_vis.setInputSource(source);
    // reg_vis.setInputTarget(target);
    // visualizer.setRegistration(reg);
    // reg_vis.setMaxCorrespondenceDistance(0.5);
    // reg_vis.setMaximumIterations(100);
    // reg_vis.setTransformationEpsilon(1e-8);
    // reg.setEuclideanFitnessEpsilon(1e-8);

    pcl::registration::TransformationEstimation<PointT, PointT>::Ptr transform_;

    // pcl::RegistrationVisualizer<PointT, PointT> visualizer;
    // visualizer.startDisplay();
    // visualizer.setMaximumDisplayedCorrespondences(1000);

    // visualizer.setRegistration(reg);

    // start = clock();
    // transform_.reset(new pcl::registration::TransformationEstimationLM<PointT, PointT>);
    // reg.setTransformationEstimation(transform_);
    // reg.align(*aligned);
    // elapsed = clock() - start;
    // PCL_INFO("Alignment time LM: %f\n", (float)elapsed / CLOCKS_PER_SEC);
    // PCL_INFO("Fitness: %f\n", reg.getFitnessScore()); //0.005

    start = clock();
    transform_.reset(new pcl::registration::TransformationEstimationSVD<PointT, PointT>);
    reg.setTransformationEstimation(transform_);
    reg.align(*aligned);
    elapsed = clock() - start;
    PCL_INFO("Alignment time SVD: %f\n", (float)elapsed / CLOCKS_PER_SEC);
    PCL_INFO("Fitness: %f\n", reg.getFitnessScore());

    // start = clock();
    // transform_.reset(new MyTransform<PointT, PointT>);
    // reg.setTransformationEstimation(transform_);
    // reg.align(*aligned_gn);
    // elapsed = clock() - start;
    // PCL_INFO("Alignment time G-N: %f\n", (float)elapsed / CLOCKS_PER_SEC);
    // PCL_INFO("Fitness: %f\n", reg.getFitnessScore());

    // start = clock();
    // transform_.reset(new pcl::registration::TransformationEstimationPointToPlane<PointT, PointT>);
    // reg.setTransformationEstimation(transform_);
    // reg.align(*aligned);
    // elapsed = clock() - start;
    // PCL_INFO("Alignment time LM Normals: %f\n", (float)elapsed / CLOCKS_PER_SEC);
    // PCL_INFO("Fitness: %f\n", reg.getFitnessScore());

    start = clock();
    transform_.reset(new pcl::registration::TransformationEstimationPointToPlaneLLS<PointT, PointT>);
    reg.setTransformationEstimation(transform_);
    reg.align(*aligned);
    elapsed = clock() - start;
    PCL_INFO("Alignment time: Normals LLS %f\n", (float)elapsed / CLOCKS_PER_SEC);
    PCL_INFO("Fitness: %f\n", reg.getFitnessScore());

    // start = clock();
    // transform_.reset(new pcl::registration::TransformationEstimationDualQuaternion<PointT, PointT>);
    // reg.setTransformationEstimation(transform_);
    // reg.align(*aligned);
    // elapsed = clock() - start;
    // PCL_INFO("Alignment time: Dual Quat %f\n", (float)elapsed / CLOCKS_PER_SEC);
    // PCL_INFO("Fitness: %f\n", reg.getFitnessScore());

    start = clock();
    transform_.reset(new MyTransformNormals<PointT, PointT>);
    reg.setTransformationEstimation(transform_);
    reg.align(*aligned_gn);
    elapsed = clock() - start;
    PCL_INFO("Alignment time G-N Normals: %f\n", (float)elapsed / CLOCKS_PER_SEC);
    PCL_INFO("Fitness: %f\n", reg.getFitnessScore());

    reg.hasConverged() ? PCL_INFO("Converged\n") : PCL_INFO("Not Converged\n");

    pcl::visualization::PCLVisualizer viewer("Viewer");
    // std::cin.get();

    int vp0, vp1, vp2;
    viewer.createViewPort(0, 0, 0.33, 1, vp0);
    viewer.createViewPort(0.33, 0, 0.66, 1, vp1);
    viewer.createViewPort(0.66, 0, 1, 1, vp2);
    viewer.addPointCloud<PointT>(target, "target", 0);
    viewer.addPointCloudNormals<PointT>(target,100,0.05,"target_n",0);

    viewer.addPointCloud<PointT>(source, "source", vp0);
    viewer.addPointCloud<PointT>(aligned, "aligned", vp1);
    viewer.addPointCloud<PointT>(aligned_gn, "aligned_gn", vp2);

    
    viewer.addCoordinateSystem(1);

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "target");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "source");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "aligned");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "aligned_gn");

    while (!viewer.wasStopped())
    {
        viewer.spin();
    }

    return 0;
}