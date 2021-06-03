#include "duna/feature_registration.h"


#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"


using PointT = pcl::PointXYZINormal;
using PointCloudT = pcl::PointCloud<PointT>;

int main(int argc, char** argv){

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
    duna::Registration<PointT>::extractFeatures(*full_source,*corners, *surfaces);
    duna::Registration<PointT> registration;


    pcl::search::KdTree<PointT>::Ptr corners_tree (new pcl::search::KdTree<PointT>);
    pcl::search::KdTree<PointT>::Ptr surfaces_tree (new pcl::search::KdTree<PointT>);
    
    
    corners_tree->setInputCloud(corners);
    surfaces_tree->setInputCloud(surfaces);

    
    registration.setTargetCornersSearchMethod(corners_tree,false);
    registration.setTargetSurfacesSearchMethod(surfaces_tree,false);

    registration.setInputCorners(corners);
    registration.setInputSurfaces(surfaces);

    registration.setTargetCorners(corner_map);
    registration.setTargetSurfaces(surface_map);

    // Set Parameters
    Eigen::Matrix4f guess;
    guess << 1, 0, 0, 24, 0, 1, 0, 0.3, 0, 0, 1, -1.5, 0, 0, 0, 1;
    
    clock_t start = clock();
    registration.align(guess);
    clock_t elapsed = clock() - start;

    std::cout << "Align time: " << (float) elapsed / CLOCKS_PER_SEC << std::endl;

    for(const auto& point : surface_map->points){
        // std::cout << point.normal_x << " ";
    }
    // Visualization

    pcl::visualization::PCLVisualizer viewer("viewer");

    PointCloudT::Ptr corners_tf (new PointCloudT);
    PointCloudT::Ptr surfaces_tf (new PointCloudT);


    viewer.addPointCloud<pcl::PointXYZINormal>(corner_map,"corner_map");
    viewer.addPointCloud<pcl::PointXYZINormal>(surface_map,"surface_map");

    viewer.addPointCloud<pcl::PointXYZINormal>(corners_tf,"corners_tf");
    viewer.addPointCloud<pcl::PointXYZINormal>(surfaces_tf,"surfaces_tf");


    viewer.addPointCloudNormals<pcl::PointXYZINormal>(surface_map,1,0.1,"surface_normals");
    // viewer.addPointCloud<pcl::PointXYZINormal>(corner_map,"corner_map");

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0,"corner_map");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0,"surface_map");

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"corner_map");



    viewer.spin();
    



    return 0;

}