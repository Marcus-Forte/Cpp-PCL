

#include "gpu_transform.h"

// __global__ void matMultKernel(const float* A, const float*B, float* C, int m, int n, int k)

__global__ void GPUTransform(const pcl::PointXYZ *pts, pcl::PointXYZ *cloud_out_pts, const Eigen::Matrix4f *matrix, int n_pts)
{

        int row = blockIdx.y * blockDim.y + threadIdx.y;
        int col = blockIdx.x * blockDim.x + threadIdx.x;
        float sum = 0;  

        if (col < n_pts && row < 4)
        {
                
                printf("hi ");
                // sum += (*matrix)(row, 0) * pts[col].x;
                // sum += (*matrix)(row, 1) * pts[col].y;
                // sum += (*matrix)(row, 2) * pts[col].z;
                // sum += (*matrix)(row, 3); //translation
                // printf("OK! -> npts: %d, r: %d , c: %d. sum: %f\n", n_pts, row,col,sum);
                // cloud_out_pts[col].data[row] = sum;           
        }
        
}

namespace gpu
{

        void TransformUnified(const pcl::PointCloud<pcl::PointXYZ> *cloud, pcl::PointCloud<pcl::PointXYZ> *cloud_out, const Eigen::Matrix4f *transform)
        {

                int n_pts = cloud->size();
                    if(cloud_out->size() != n_pts){
                            printf("resizing ... \n");
                       cloud_out->resize(n_pts);
                    }

                cudaError_t err;

                const pcl::PointXYZ *pts = cloud->points.data();

                for(int i =0; i < cloud->size(); ++ i){
                        printf("pt[%d]: %f %f %f\n",i,pts[i].x, pts[i].y, pts[i].z);
                }

                int m = 4;
                int n = 4;
                int k = n_pts;

                int threads_per_block = 4; //
                unsigned int grid_rows = (m + threads_per_block - 1) / threads_per_block;
                unsigned int grid_cols = (k + threads_per_block - 1) / threads_per_block;
                dim3 dimGrid(grid_cols, grid_rows);
                dim3 dimBlock(threads_per_block, threads_per_block);
                printf("dimGrid (%d,%d,%d)\n",dimGrid.x,dimGrid.y,dimGrid.z);
                printf("dimBlock (%d,%d,%d)\n",dimBlock.x,dimBlock.y,dimBlock.z);
                GPUTransform<<<dimGrid, dimBlock>>>(cloud->points.data(), cloud_out->points.data(), transform, n_pts);

                err = cudaGetLastError();
                if (err != cudaSuccess)
                        std::cout << "Kernel launch error: " << cudaGetErrorString(err) << std::endl;

                // err = cudaMemcpy(cloud_out->points.data(), d_tf_pts, n_pts * sizeof(pcl::PointXYZ), cudaMemcpyDeviceToHost);
                // if (err != cudaSuccess)
                //         std::cout << "cudaMemcpy cloud out Failure" << std::endl;


        }

        void Transform(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &cloud_out, const Eigen::Matrix4f &transform)
        {
                pcl::PointXYZ *d_pts;
                Eigen::Matrix4f *d_transform_matrix;
                pcl::PointXYZ *d_tf_pts;

                int n_pts = cloud.points.size();

                if (cloud_out.points.size() != n_pts)
                        cloud_out.points.resize(n_pts);

                cudaError_t err;

                err = cudaMalloc(&d_pts, n_pts * sizeof(pcl::PointXYZ));
                if (err != cudaSuccess)
                        std::cout << "cudaMalloc Failure" << std::endl;

                err = cudaMalloc(&d_transform_matrix, sizeof(Eigen::Matrix4f));
                if (err != cudaSuccess)
                        std::cout << "cudaMalloc Failure" << std::endl;

                err = cudaMalloc(&d_tf_pts, n_pts * sizeof(pcl::PointXYZ));
                if (err != cudaSuccess)
                        std::cout << "cudaMalloc Failure" << std::endl;

                std::cout << "GPU Mem Used: " << (2 * n_pts * sizeof(pcl::PointXYZ) + sizeof(Eigen::Matrix4f)) / 1000000 << " MB" << std::endl;

                err = cudaMemcpy(d_pts, cloud.points.data(), n_pts * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);
                if (err != cudaSuccess)
                        std::cout << "cudaMalloc Failure" << std::endl;

                err = cudaMemcpy(d_transform_matrix, &transform, sizeof(Eigen::Matrix4f), cudaMemcpyHostToDevice);
                if (err != cudaSuccess)
                        std::cout << "cudaMalloc Failure" << std::endl;

                // Kernel Call
                int m = 4;
                int n = 4;
                int k = n_pts;

                int threads_per_block = 4; //
                unsigned int grid_rows = (m + threads_per_block - 1) / threads_per_block;
                unsigned int grid_cols = (k + threads_per_block - 1) / threads_per_block;
                dim3 dimGrid(grid_cols, grid_rows);
                dim3 dimBlock(threads_per_block, threads_per_block);
                // printf("dimGrid (%d,%d,%d)\n",dimGrid.x,dimGrid.y,dimGrid.z);
                // printf("dimBlock (%d,%d,%d)\n",dimBlock.x,dimBlock.y,dimBlock.z);
                GPUTransform<<<dimGrid, dimBlock>>>(d_pts, d_tf_pts, d_transform_matrix, n_pts);

                err = cudaGetLastError();
                if (err != cudaSuccess)
                        std::cout << "Kernel launch error: " << cudaGetErrorString(err) << std::endl;

                err = cudaMemcpy(cloud_out.points.data(), d_tf_pts, n_pts * sizeof(pcl::PointXYZ), cudaMemcpyDeviceToHost);
                if (err != cudaSuccess)
                        std::cout << "cudaMemcpy Failure" << std::endl;

                cudaFree(d_pts);
                cudaFree(d_tf_pts);
                cudaFree(d_pts);
        }

}
