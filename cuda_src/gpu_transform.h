#ifndef TRANSFORM_
#define TRANSFORM_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cuda_runtime.h>

// class point3
// {
//     public:
//     float x;
//     float y;
//     float z;
// };

// class t_matrix
// {
//     public:
//     float r11, r12, r13, tx;
//     float r21, r22, r23, ty;
//     float r31, r32, r33, tz;
// };

namespace mygpu
{
    class Transform
    {

        using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;
        using PointT = pcl::PointXYZ;

    public:
        Transform(const PointCloudT &cloud_in, const Eigen::Matrix4f &tf)
        {
            n_pts = cloud_in.size();
            block_size = 512;
            grid_size = (n_pts + block_size - 1) / block_size;

            std::cout << "Blk Size:" << block_size << std::endl;
            std::cout << "N Grids:" << grid_size << std::endl;
            std::cout << "N Pts:" << n_pts << std::endl;

            // input points
            cudaError_t err;
            err = cudaMalloc(&pts_d, n_pts * sizeof(pcl::PointXYZ));
            if (err != cudaSuccess)
            {
                std::cout << "cudaMalloc Failure" << std::endl;
                exit(EXIT_FAILURE);
            }

            err = cudaMemcpy(pts_d, cloud_in.points.data(), n_pts * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);
            if (err != cudaSuccess)
            {
                std::cout << "GPU memCpy error " << std::endl;
                exit(EXIT_FAILURE);
            }

                        //transform matrix
            err = cudaMalloc(&matrix_d, sizeof(Eigen::Matrix4f));

            if (err != cudaSuccess)
            {
                std::cout << "cudaMalloc Failure" << std::endl;
                exit(EXIT_FAILURE);
            }


            err = cudaMemcpy(matrix_d, &tf, sizeof(Eigen::Matrix4f), cudaMemcpyHostToDevice);
            if (err != cudaSuccess)
            {
                std::cout << "GPU memCpy error " << std::endl;
                exit(EXIT_FAILURE);
            }

            //transformed points
            err = cudaMalloc(&tf_pts_d, n_pts * sizeof(pcl::PointXYZ));
            if (err != cudaSuccess)
            {
                std::cout << "cudaMalloc Failure" << std::endl;
                exit(EXIT_FAILURE);
            }





            std::cout << "Alloc points OK" << std::endl;
        }

        // Each kernel is to compute transform of each point
        void Apply(PointCloudT &cloud_transformed);

    private:
        int n_pts; // N# of points
        int block_size;
        int grid_size;

        // pcl::PointXYZ* pts_h;
        pcl::PointXYZ *pts_d;

        Eigen::Matrix4f *matrix_d;
        pcl::PointXYZ *tf_pts_d;

        // point3 *pts_h; //host address
        // t_matrix *matrix_h; // host address
        // point3 *t_pts_h; //host address

        // point3 *pts; //dev address
        // t_matrix *matrix; // dev address
        // point3 *t_pts; //dev address
    };
}

#endif