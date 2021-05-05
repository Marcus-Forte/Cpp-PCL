#pragma once

#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/octree.h>
#include <unordered_set>

#include <pcl/features/normal_3d.h>
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

    inline void setResolution(double res)
    {
        resolution = res;
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

    double compute(PointCloud &output_cloud)
    {
        pcl::octree::OctreePointCloudSearch<PointT> octree(resolution);
        pcl::octree::OctreePointCloudSearch<PointT> octree_ground(resolution);

        PointCloudPtr oct_cloud(new PointCloud);
        PointCloudPtr oct_cloud_gnd(new PointCloud);
        output_cloud.clear();

        PCL_INFO("Computing ... %d, %d\n", ground->size(), cloud->size());
        if (!cloud)
        {
            PCL_ERROR("NO CLOUD");
            return 0;
        }

        if (!ground)
        {
            PCL_ERROR("NO GROUND  ");
            return 0;
        }

        if (registration)
        {
            PointCloudPtr src(new PointCloud);
            PointCloudPtr tgt(new PointCloud);

            *oct_cloud = *cloud;
            *oct_cloud_gnd = *ground;
        }
        else
        {
            *oct_cloud = *cloud;
            *oct_cloud_gnd = *ground;
        }

        octree_ground.setInputCloud(oct_cloud_gnd);
        octree_ground.addPointsFromInputCloud();
        octree.setInputCloud(oct_cloud);
        octree.addPointsFromInputCloud();

        PointT min, max;
        pcl::getMinMax3D(*oct_cloud, min, max);

        PointT box_min(min.x, min.y, min.z);
        PointT box_max(min.x + resolution, min.y + resolution, max.z);

        pcl::IndicesPtr indices(new pcl::Indices);
        pcl::IndicesPtr indices_gnd(new pcl::Indices);

        pcl::ExtractIndices<PointT> extractor;
        pcl::ExtractIndices<PointT> extractor_gnd;

        extractor.setIndices(indices);
        extractor.setInputCloud(oct_cloud);

        extractor_gnd.setIndices(indices_gnd);
        extractor_gnd.setInputCloud(oct_cloud_gnd);

        PointCloudPtr box_cloud(new PointCloud);
        PointCloudPtr box_cloud_gnd(new PointCloud);

        // pcl::PointCloud debug_cloud;

        PCL_INFO("Loop\n");

        pcl::visualization::PCLVisualizer::Ptr viewer;

        if (debug)
        {
            viewer.reset(new pcl::visualization::PCLVisualizer);
            // viewer->addCoordinateSystem(1);
            viewer->addPointCloud(oct_cloud, "cloud");
            viewer->addPointCloud(oct_cloud_gnd, "ground");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "ground");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");
        }

        float sum_z = 0;

        while (box_max.y < max.y)
        {

            while (box_max.x < max.x)
            {
                // PCL_INFO("searching box\n");
                octree.boxSearch(box_min.getVector3fMap(), box_max.getVector3fMap(), *indices);
                octree_ground.boxSearch(box_min.getVector3fMap(), box_max.getVector3fMap(), *indices_gnd);

                if (indices->size())
                {
                    // PCL_INFO("Extracting box\n");
                    extractor.filter(*box_cloud);
                    extractor_gnd.filter(*box_cloud_gnd);
                    // std::cout << "Extracted: " << box_cloud->size() << std::endl;

                    // PCL_INFO("Extracting min max\n");
                    PointT min_, max_;
                    PointT min_gnd, max_gnd;
                    pcl::getMinMax3D(*box_cloud, min_, max_);
                    pcl::getMinMax3D(*box_cloud_gnd, min_gnd, max_gnd);

                    bool paint_blue = false;
                    if ((max_.z - max_gnd.z) > 0.02)
                    { // filter
                        sum_z += max_.z - min_.z;
                        output_cloud += *box_cloud + *box_cloud_gnd;
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
                        viewer->spinOnce(15);
                        std::this_thread::sleep_for(std::chrono::milliseconds(15));
                        std::cout << "Volume: " << sum_z * resolution * resolution << std::endl;
                    }
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