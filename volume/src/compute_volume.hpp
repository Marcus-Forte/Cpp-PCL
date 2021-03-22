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
        this->cloud = cloud; // share ownership
    }

    inline void setGroundCloud(PointCloudConstPtr ground)
    {
        this->ground = ground; // share owenership
    }

    double compute()
    {
        pcl::octree::OctreePointCloudSearch<PointT> octree(resolution);
        PointCloudPtr subtracted(new PointCloud);
        if (registration)
        {
            pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;

            PointCloudPtr cloud_ds (new PointCloud);
            PointCloudPtr ground_ds (new PointCloud);

            pcl::VoxelGrid<PointT> voxel;
            voxel.setInputCloud(cloud);
            voxel.setLeafSize(resolution,resolution,resolution);
            voxel.filter(*cloud_ds);

            voxel.setInputCloud(ground);
            voxel.filter(*ground_ds);

            gicp.setInputSource(ground_ds);
            gicp.setInputTarget(cloud_ds);
            gicp.setMaxCorrespondenceDistance(0.1);
            gicp.setMaximumOptimizerIterations(5);
            std::cout << "Aligning..." << std::endl;
            gicp.align(*subtracted);
            std::cout << "Done" << std::endl;

            pcl::visualization::PCLVisualizer viewer;
            viewer.addCoordinateSystem();
            
            viewer.addPointCloud(ground_ds);
            viewer.addPointCloud(ground_ds);
        }

        // *subtracted = *cloud;
        octree.setInputCloud(subtracted);
        octree.addPointsFromInputCloud();

        PointCloudPtr downsampled(new PointCloud);

        pcl::VoxelGrid<PointT> voxel;
        voxel.setInputCloud(ground);
        voxel.setLeafSize(resolution, resolution, resolution);
        voxel.filter(*downsampled);

        std::cout << "Downsampled points: " << downsampled->size();

        pcl::IndicesPtr indices(new pcl::Indices);

        pcl::ExtractIndices<PointT> extractor;
        extractor.setIndices(indices);
        extractor.setInputCloud(subtracted);
        extractor.setNegative(true);

        std::unordered_set<int> lista;

        for (const auto &it : downsampled->points)
        {
            PointT min(it.x - resolution / 2, it.y - resolution / 2, it.z - resolution / 2);
            PointT max(it.x + resolution / 2, it.y + resolution / 2, it.z + resolution / 2);
            pcl::Indices curr_indices;
            octree.boxSearch(min.getVector3fMap(), max.getVector3fMap(), curr_indices);
            lista.insert(curr_indices.begin(), curr_indices.end()); //mark for removal
        }

        std::cout << "pontos removidos: " << lista.size() << std::endl;

        for (const auto &it : lista)
        {
            indices->push_back(it);
        }

        // (*indices).insert(lista.begin(),lista.end());
        extractor.filter(*subtracted);

        if (debug)
        {
            pcl::visualization::PCLVisualizer viewer;
            int v0, v1, v2;
            viewer.addCoordinateSystem(1);
            viewer.createViewPort(0, 0, 0.33, 1, v0);
            viewer.createViewPort(0.33, 0, 0.66, 1, v1);
            viewer.createViewPort(0.66, 0, 01, 1, v2);

            viewer.addPointCloud(cloud, "cloud", v0);
            viewer.addPointCloud(ground, "ground", v1);
            viewer.addPointCloud(subtracted, "sub", v2);

            // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,1,"cloud");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "ground");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sub");

            // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5,"ground");
            // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5,"sub");

            while (!viewer.wasStopped())
            {
                viewer.spin();
            }
        }

        return 0;
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