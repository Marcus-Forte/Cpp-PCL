#pragma once

#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/octree.h>
#include <unordered_set>

#include <pcl/registration/gicp.h>

template <class PointT>
class volumeEstimator
{
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

public:
    volumeEstimator(double resolution_) : resolution(resolution_), debug(false), registration(false)
    {
    }

    inline void setDebug(bool debug_)
    {
        debug = debug_;
    }

    inline void SetRegistration(bool registration_)
    {
        registration = registration_;
    }

    inline void setInputCloud(PointCloudConstPtr cloud)
    {
        this->cloud = cloud; // share ownership to avoid copy
    }

    inline void setGroundCloud(PointCloudConstPtr ground)
    {
        this->ground = ground; // share owenership to avoid copy
    }

    double compute()
    {
        pcl::octree::OctreePointCloudSearch<PointT> octree(resolution);
        PointCloudPtr oct_cloud(new PointCloud);
        if (!cloud)
        {
            PCL_ERROR("NO CLOUD");
            return 0;
        }

        *oct_cloud = *cloud;
        if (ground.get() != nullptr)
        {

            if (registration)
            {
                pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
                
                pcl::VoxelGrid<PointT> voxel;

                PointCloudPtr ground_ds(new PointCloud);
                PointCloudPtr cloud_ds(new PointCloud);
                PointCloudPtr aligned_ground(new PointCloud);

                voxel.setInputCloud(ground);
                voxel.setLeafSize(resolution, resolution, resolution);
                voxel.filter(*ground_ds);

                voxel.setInputCloud(cloud);
                voxel.filter(*cloud_ds);

                gicp.setInputSource(ground_ds);
                gicp.setInputTarget(cloud_ds);
                gicp.setMaximumOptimizerIterations(10);
                std::cout << "Registering..." << std::endl;
                gicp.align(*aligned_ground);
                std::cout << "Done..." << std::endl;

                Eigen::Matrix4f gicp_transform = gicp.getFinalTransformation();
                pcl::transformPointCloud(*ground, *aligned_ground, gicp_transform);

                *oct_cloud += *aligned_ground;
            }
            else
            {
                *oct_cloud += *ground;
            }
        }
        else
        {
            PCL_ERROR("NO GROUND BUT OK ");
        }

        octree.setInputCloud(oct_cloud);
        octree.addPointsFromInputCloud();

        PointT min, max;
        pcl::getMinMax3D(*oct_cloud, min, max);

        PointT box_min(min.x, min.y, min.z);
        PointT box_max(min.x + resolution, min.y + resolution, max.z);

        // std::cout << min << std::endl;
        // std::cout << max << std::endl;

        // std::cout << box_min << std::endl;
        // std::cout << box_max << std::endl;

        pcl::IndicesPtr indices(new pcl::Indices);
        pcl::ExtractIndices<PointT> extractor;
        extractor.setIndices(indices);
        extractor.setInputCloud(oct_cloud);

        PointCloudPtr box_cloud(new PointCloud);

        // pcl::PointCloud debug_cloud;

        pcl::visualization::PCLVisualizer::Ptr viewer;

        if (debug)
        {
            viewer.reset(new pcl::visualization::PCLVisualizer);
            viewer->addCoordinateSystem(1);
            viewer->addPointCloud(oct_cloud);
        }

        float sum_z = 0;

        while (box_max.y < max.y)
        {

            while (box_max.x < max.x)
            {
                octree.boxSearch(box_min.getVector3fMap(), box_max.getVector3fMap(), *indices);

                if (indices->size())
                {

                    extractor.filter(*box_cloud);
                    // std::cout << "Extracted: " << box_cloud->size() << std::endl;

                    PointT min_, max_;
                    pcl::getMinMax3D(*box_cloud, min_, max_);
                    bool paint_blue = false;
                    if ((max_.z - min_.z) > 0.4)
                    { // filter
                        sum_z += max_.z - min_.z;
                        paint_blue = true;
                    }

                    if (debug)
                    {
                        viewer->removePointCloud("box");
                        // PointCloudPtr box_ptr(&box_cloud);
                        viewer->addPointCloud(box_cloud, "box");
                        if (paint_blue)
                            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "box");
                        else
                            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "box");
                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "box");
                        viewer->spinOnce(25);
                    }

                    std::cout << "Volume: " << sum_z * resolution * resolution << std::endl;
                }

                box_min.x += resolution;
                box_max.x += resolution;
            }

            //Next box row
            box_min.x = min.x;
            box_max.x = min.x + resolution;

            box_min.y += resolution;
            box_max.y += resolution;
        }

        return sum_z * resolution * resolution;
    }

    ~volumeEstimator()
    {
    }

private:
    PointCloudConstPtr cloud;
    PointCloudConstPtr ground;
    double resolution;
    bool debug;
    bool registration;
};