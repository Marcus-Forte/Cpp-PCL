#pragma once

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>



class Features
{

    typedef std::pair<float, int> mypair; //curvature + index

public:
    template <class T>
    static void EdgeDetection(const pcl::PointCloud<T> &input_cloud, pcl::PointCloud<T> &features_cloud, int N)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr window_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr input_buffer(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr feature_points(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::copyPointCloud(input_cloud, *input_buffer);

        // Main Loop
        for (int i = N; i < input_cloud.size() - N; i++)
        {

            window_cloud->clear();
            window_cloud->push_back(input_cloud.points[i]);
            for (int j = 1; j < N; ++j)
            {
                window_cloud->push_back(input_cloud.points[i + j]);
                window_cloud->push_back(input_cloud.points[i - j]);
            }

            T centroid;
            pcl::computeCentroid(*window_cloud, centroid);
            // cout << "Query Point ->" << window_cloud->points[0] << endl;
            // cout << "Centroid -> " << centroid << endl;

            float resolution = 9999;
            float dist;
            for (auto it = window_cloud->begin() + 1; it < window_cloud->end(); ++it)
            {
                // cout << *it << endl;
                dist = pcl::euclideanDistance(window_cloud->points[0], (*it));
                if (dist < resolution)
                    resolution = dist;
            }

            float isEdge = (centroid.getVector3fMap() - window_cloud->points[0].getVector3fMap()).norm();
            float lambda = 6;
            if (isEdge > lambda * resolution)
            {
                // cout << "Edge!" << endl;
                feature_points->push_back(*(window_cloud->points.end() - 2));
                feature_points->push_back(*(window_cloud->points.end() - 4)); //last two
                i += 2 * N;
            }
        }

        pcl::copyPointCloud(*feature_points, features_cloud);
    }

    // Must preallocate
    template <class T>
    static void ComputeSmoothness(const pcl::PointCloud<T> &input_cloud, pcl::PointCloud<T> &features_cloud, int N, int N_planar)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr window_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr feature_points(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Main Loop
        int count = 0;

        std::vector<mypair> curvatures;

        for (int i = N; i < input_cloud.size() - N; i++)
        {

            window_cloud->clear();
            window_cloud->push_back(input_cloud.points[i]);
            for (int j = 1; j < N; ++j)
            {
                window_cloud->push_back(input_cloud.points[i + j]);
                window_cloud->push_back(input_cloud.points[i - j]);
            }

            T centroid;
            pcl::computeCentroid(*window_cloud, centroid);

            // less means more smooth
            // float smoothness = (centroid.getVector3fMap() - window_cloud->points[0].getVector3fMap()).norm();

            Eigen::Vector3f sum = Eigen::Vector3f::Zero();
            for (int k = 1; k < window_cloud->size(); ++k)
            {
                sum += (window_cloud->points[0].getVector3fMap() - window_cloud->points[k].getVector3fMap());
            }

            // from LOAM paper
            float smoothness = sum.norm() / (window_cloud->points[0].getVector3fMap().norm() * window_cloud->size());
            mypair curv;
            curv.first = smoothness;
            curv.second = i;
            curvatures.push_back(curv);
            // curvature[count] = smoothness;
            // count++;
        }

        std::sort(curvatures.begin(), curvatures.end(), comparator);

        std::ofstream file;
        file.open("log.txt");
        for ( auto it : curvatures){
            // std::cout << "c = " << it.first << "," << "i = " << it.second << std::endl;
            file << "c = " << it.first << "," << "i = " << it.second << std::endl;
        }
        file.close();

        int index;
        int index_low;
        for (int j = 0; j < N_planar; ++j)
        {                                             // pick top ten
            index = (curvatures.begin() + j)->second; // index
            index_low = (curvatures.end() - 1 - j)->second;
            feature_points->points.push_back(input_cloud.points[index]); // Smooth 
            // feature_points->points.push_back(input_cloud.points[index_low]); // Not Smooth
        }

        pcl::copyPointCloud(*feature_points, features_cloud);
    }

private:
    Features() {}
    ~Features() {}

    static bool comparator(const mypair &l, const mypair &r)
    {
        return l.first < r.first;
    }
};
