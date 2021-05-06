#include <iostream>

#include <pcl/surface/grid_projection.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/ear_clipping.h>
#include <pcl/surface/gp3.h>

#include "PCUtils.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>

using PointT = pcl::PointNormal;
using PointCloudT = pcl::PointCloud<PointT>;

int main(int argc, char **argv)
{

    PointCloudT::Ptr cloud(new PointCloudT);

    PCUtils::readFile(argv[1], *cloud);

    pcl::NormalEstimation<PointT, PointT> ne;
    ne.setInputCloud(cloud);
    ne.setKSearch(50);
    ne.setViewPoint(0, 0, 100);

    ne.compute(*cloud);

    PCL_INFO("init pts: %d\n", cloud->size());
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.001, 0.001, 0.001);
    voxel.filter(*cloud);
    PCL_INFO("voxel pts: %d\n", cloud->size());

    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

    // pcl::MarchingCubesRBF<PointT> recon;
    // recon.setIsoLevel(0.0);
    // recon.setInputCloud(cloud);
    // recon.setGridResolution(30,30,30);
    // recon.setOffSurfaceDisplacement(0.01);

    pcl::MarchingCubesHoppe<PointT> recon;
    recon.setInputCloud(cloud);
    recon.setGridResolution(100, 100, 100);
    recon.setIsoLevel(0.00);
    recon.setPercentageExtendGrid(0.0);

    // pcl::ConvexHull<PointT> recon;
    // recon.setInputCloud(cloud);
    // recon.reconstruct(*mesh);

    // pcl::Poisson<PointT> recon;
    // recon.setInputCloud(cloud);
    // recon.setManifold(true);
    // // recon.setIsoDivide(2);
    // // recon.setScale(0.001);

    // recon.reconstruct(*mesh);

    // pcl::GreedyProjectionTriangulation<PointT> recon;
    // recon.setInputCloud(cloud);
    // // recon.setMaximumNearestNeighbors(10);
    // recon.setMu(15);
    // recon.setSearchRadius(0.1);
    // // recon.setMaximumSurfaceAngle(0.5);
    // recon.setNormalConsistency(true);
    // // recon.setMaximumAngle(0.5);
    // recon.reconstruct(*mesh);

    recon.reconstruct(*mesh);

    pcl::visualization::PCLVisualizer viewer;

    int vp0, vp1;
    viewer.createViewPort(0, 0, 0.5, 1, vp0);
    viewer.createViewPort(0.5, 0, 1, 1, vp1);
    viewer.addPointCloud<PointT>(cloud, "cloud", vp0);
    viewer.addPointCloudNormals<PointT>(cloud, 10, 0.01, "normals", vp0);
    viewer.addPolygonMesh(*mesh, "mesh", vp1);

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "normals");

    pcl::io::savePLYFile("surface.ply", *mesh);

    while (viewer.wasStopped() == false)
    {
        viewer.spin();
    }

    return 0;
}