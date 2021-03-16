

#include "gpu_transform.h"



// Each kernel computes the entire matrix multiplication PER point
__global__ void GPUTransform(pcl::PointXYZ* pts, pcl::PointXYZ* tf_pts, const Eigen::Matrix4f* matrix, int n_pts){
        
        int tid = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;

        // very very stupid kernel usage
        // if(tid < n_pts){
        //         tf_pts[tid].x =  (*matrix)(0,0) * pts[tid].x  + (*matrix)(0,1) * pts[tid].y + (*matrix)(0,2) * pts[tid].z + (*matrix)(0,3);
        //         tf_pts[tid].y =  (*matrix)(1,0) * pts[tid].x  + (*matrix)(1,1) * pts[tid].y + (*matrix)(1,2) * pts[tid].z + (*matrix)(1,3);
        //         tf_pts[tid].z =  (*matrix)(2,0) * pts[tid].x  + (*matrix)(2,1) * pts[tid].y + (*matrix)(2,2) * pts[tid].z + (*matrix)(2,3);

        // }

        for(int i=tid;i<n_pts;i += stride){
                tf_pts[tid].x =  (*matrix)(0,0) * pts[tid].x  + (*matrix)(0,1) * pts[tid].y + (*matrix)(0,2) * pts[tid].z + (*matrix)(0,3);
                tf_pts[tid].y =  (*matrix)(1,0) * pts[tid].x  + (*matrix)(1,1) * pts[tid].y + (*matrix)(1,2) * pts[tid].z + (*matrix)(1,3);
                tf_pts[tid].z =  (*matrix)(2,0) * pts[tid].x  + (*matrix)(2,1) * pts[tid].y + (*matrix)(2,2) * pts[tid].z + (*matrix)(2,3);

        }

}

// A smater kernel would 


void mygpu::Transform::Apply(PointCloudT& cloud_transformed){
        
        GPUTransform<<<block_size,grid_size>>>(pts_d,tf_pts_d,matrix_d,n_pts);
        cudaDeviceSynchronize();
        // Get Results
        cloud_transformed.resize(n_pts);
        cudaError_t err = cudaMemcpy(cloud_transformed.points.data(), tf_pts_d, n_pts * sizeof(pcl::PointXYZ), cudaMemcpyDeviceToHost);
        if(err != cudaSuccess){
        std::cout << "GPU memCpy error " << std::endl;
        exit(EXIT_FAILURE);
        
        }
        



        
}

// __global__ void vectorAddGPU(float *A, const float *B, int numElements) {
// int tid = blockIdx.x * blockDim.x + threadIdx.x;
// int stride = blockDim.x * gridDim.x;



// for(int i=tid;i<numElements;i += stride){
// 		A[i] = A[i] * B[i];
// }

// }



