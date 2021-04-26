#include <iostream>

#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_3point.h>
#include <pcl/console/print.h>

#include <pcl/features/normal_3d.h>

#include <pcl/io/pcd_io.h>

#include "my_transform.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/registration_visualizer.h>

#include <ctime>

using PointT = pcl::PointXYZINormal;
using PointCloudT = pcl::PointCloud<PointT>;

class Parent {
    public:

    protected:
    int prot_m;
};

class Child : public Parent {


    // using Parent::prot_m;
    public:

    int getProt(){
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

    if (argc < 3)
    {
        PCL_ERROR("use : %s [src] [tgt]\n", argv[0]);
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

    PCL_INFO("src pts : %d\n", source->size());
    PCL_INFO("tgt pts : %d\n", target->size());

    PointCloudT::Ptr aligned(new PointCloudT);


    pcl::NormalEstimation<PointT,PointT> ne;
    ne.setInputCloud(target);
    ne.setKSearch(10);
    ne.compute(*target);

    pcl::IterativeClosestPoint<PointT,PointT> reg;
    // pcl::GeneralizedIterativeClosestPoint<PointT, PointT> reg;
    // pcl::registration::FPCSInitialAlignment<PointT,PointT> reg;

    pcl::RegistrationVisualizer<PointT, PointT> visualizer;
    
    reg.setInputSource(source);
    reg.setInputTarget(target);
    // visualizer.setRegistration(icp);
    reg.setMaxCorrespondenceDistance(0.5);
    reg.setMaximumIterations(50);
    // reg.setRANSACOutlierRejectionThreshold(0.15);
    // reg.setRANSACIterations(5);
    // reg.setRotationEpsilon(0.2);
    // reg.setCorrespondenceRandomness(12);
    // icp.getCorrespondenceRejectors()
    // 0.000654 vs 0.000877
    // pcl::registration::TransformationEstimation3Point<PointT,PointT>::Ptr transform_(new pcl::registration::TransformationEstimation3Point<PointT,PointT>);
    // pcl::registration::TransformationEstimationDualQuaternion<PointT,PointT>::Ptr transform_(new pcl::registration::TransformationEstimationDualQuaternion<PointT,PointT>);
    MyTransform<PointT, PointT>::Ptr transform_(new MyTransform<PointT, PointT>);
    // pcl::registration::TransformationEstimationLM<PointT,PointT>::Ptr transform_(new pcl::registration::TransformationEstimationLM<PointT,PointT>);
    // pcl::registration::TransformationEstimationPointToPlaneLLS<PointT,PointT>::Ptr transform_(new pcl::registration::TransformationEstimationPointToPlaneLLS<PointT,PointT>);
    // pcl::registration::TransformationEstimationPointToPlane<PointT,PointT>::Ptr transform_ (new pcl::registration::TransformationEstimationPointToPlane<PointT,PointT>);
    reg.setTransformationEstimation(transform_);
    reg.setTransformationEpsilon(1e-7);
    // reg.setRotationEpsilon(1e-8);
    
    


    // visualizer.setRegistration(reg);
    // visualizer.startDisplay();
    // visualizer.setMaximumDisplayedCorrespondences(100);
    clock_t start = clock();
    reg.align(*aligned);
    clock_t elapsed = clock() - start;
    PCL_INFO("Alignment time: %f\n", (float) elapsed /CLOCKS_PER_SEC);

    PCL_INFO("Fitness: %f\n", reg.getFitnessScore());
  

    reg.hasConverged() ? PCL_INFO("Converged\n") : PCL_INFO("Not Converged\n");

    pcl::visualization::PCLVisualizer viewer("Viewer");
    // std::cin.get();

    int vp0, vp1;
    viewer.createViewPort(0, 0, 0.5, 1, vp0);
    viewer.createViewPort(0.5, 0, 1, 1, vp1);
    viewer.addPointCloud<PointT>(target, "target", 0);
    viewer.addPointCloud<PointT>(source, "source", vp0);
    viewer.addPointCloud<PointT>(aligned, "aligned", vp1);
    viewer.addCoordinateSystem(1);

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "target");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "source");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "aligned");

    while (!viewer.wasStopped())
    {
        viewer.spin();
    }

    return 0;
}