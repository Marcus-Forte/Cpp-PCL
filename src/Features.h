#pragma once

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

using namespace std;

class Features
{

    typedef std::pair<float, int> query_point; //curvature + index

public:
    template <class T>
    static void PartitionCloud(const pcl::PointCloud<T> &input, std::vector<pcl::PointCloud<T>> &partitions, int N_partitions)
    {
        int total_size = input.size();
        int block_size = total_size / N_partitions;
        int remainder = total_size % N_partitions;

        // std::cout << "Points: " << total_size << std::endl;
        // std::cout << "Number of Partitions: " << N << std::endl;
        // std::cout << "Block Size: " << block_size << std::endl;
        // std::cout << "Remainder: " << remainder << std::endl;

        pcl::PointCloud<T> PC;
        partitions.resize(N_partitions);

        T pt;
        for (int i = 0; i < N_partitions; i++)
        {
            // cout << "Partition: " << i << endl;
            for (int j = 0; j < block_size; j++)
            {
                pt = input.points[i * block_size + j];
                // cout << "index = " << i*block_size + j << endl;
                partitions[i].points.push_back(pt);
            }
        }

        for (int i = 0; i < remainder; ++i)
        {

            pt = input.points[N_partitions * block_size + i];
            // cout << "Remainder index: " << N*block_size + i << endl;
            partitions[N_partitions - 1].points.push_back(pt);
        }
    }

    //
    template <typename T>
    static void EdgeDetection(const pcl::PointCloud<T> &input_cloud, pcl::PointCloud<T> &features_cloud, int window_size, int n_highest)
    {
        typename pcl::PointCloud<T>::Ptr window_cloud(new pcl::PointCloud<T>);
        typename pcl::PointCloud<T>::Ptr feature_points(new pcl::PointCloud<T>);

        std::vector<query_point> edge_list;

        window_cloud->points.resize(window_size + 1); // Query point + window_size

        // Main Loop
        for (int i = window_size; i < input_cloud.size() - window_size; i++)
        {
            // WE must preserve centrality over query point i
            window_cloud->points[0] = input_cloud.points[i];
            int count = 1;
            for (int j = 1; j <= window_size / 2; ++j)
            { // Error here
                window_cloud->points[count] = input_cloud.points[i + j];
                count++;
                window_cloud->points[count] = input_cloud.points[i - j];
                count++;
            }

            // cout << "Window: " << i << endl;
            // for (auto it : window_cloud->points)
            // {
            //     cout << it << endl;
            // }

            // cout << endl;

            // [i] , [i+j] , [i + 2j] , [i + 2j] , [i - 2j], ...

            T centroid;
            pcl::computeCentroid(*window_cloud, centroid);

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
                query_point p;
                p.first = isEdge;
                p.second = i;
                edge_list.push_back(p);
                // feature_points->push_back(window_cloud->points[0]);
                // feature_points->push_back(*(window_cloud->points.end() - 4)); //
                // i += 2 * window_size;
            }
        }
        // Sort highest edges

        if(edge_list.size() > n_highest){
        std::sort(edge_list.begin(), edge_list.end(), comparator); // low to high

        int index;
        for (int i = 0; i < n_highest; ++i)
        {
            index = (edge_list.end() - 1 - i)->second;
            feature_points->push_back(input_cloud.points[index]);
        }

        // pcl::copyPointCloud(*feature_points, features_cloud);
        features_cloud = *feature_points;
        // cout << "Features -> " << features_cloud.size() << endl;
        }
    }

    // Must preallocate
    template <typename T>
    static void ComputeSmoothness(const pcl::PointCloud<T> &input_cloud, pcl::PointCloud<T> &features_cloud, int window_size, int n_highest)
    {
        typename pcl::PointCloud<T>::Ptr window_cloud(new pcl::PointCloud<T>);
        typename pcl::PointCloud<T>::Ptr feature_points(new pcl::PointCloud<T>);

        // Main Loop
        int count = 0;

        std::vector<query_point> curvatures;

        for (int i = window_size; i < input_cloud.size() - window_size; i++)
        {

            window_cloud->clear();
            window_cloud->push_back(input_cloud.points[i]);
            for (int j = 1; j < window_size; ++j)
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
            query_point curv;
            curv.first = smoothness;
            curv.second = i;
            curvatures.push_back(curv);
            // curvature[count] = smoothness;
            // count++;
        }


        if(curvatures.size() > n_highest ) {
        std::sort(curvatures.begin(), curvatures.end(), comparator);

        std::ofstream file;
        file.open("log.txt");
        for (auto it : curvatures)
        {
            // std::cout << "c = " << it.first << "," << "i = " << it.second << std::endl;
            file << "c = " << it.first << ","
                 << "i = " << it.second << std::endl;
        }
        file.close();

        int index;
        int index_low;
        for (int j = 0; j < n_highest; ++j)
        {                                             // pick top ten
            index = (curvatures.begin() + j)->second; // index
            index_low = (curvatures.end() - 1 - j)->second;
            feature_points->points.push_back(input_cloud.points[index]); // Smooth
            // feature_points->points.push_back(input_cloud.points[index_low]); // Not Smooth - requires further filtering
        }

        // pcl::copyPointCloud(*feature_points, features_cloud);
        features_cloud = *feature_points;
        }
    }

private:
    Features() {}
    ~Features() {}

    static bool comparator(const query_point &l, const query_point &r)
    {
        return l.first < r.first;
    }
};
