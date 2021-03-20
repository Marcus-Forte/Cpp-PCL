#include <iostream>
#include <chrono>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
// #include <pcl/features/normal_3d.h>

// #include <pcl/filters/statistical_outlier_removal.h>

// #include <pcl/gpu/features/features.hpp>
// #include <pcl/gpu/containers/initialization.h>
// #include <pcl/gpu/containers/device_array.h>
// #include <pcl/gpu/containers/kernel_containers.h>
// #include <pcl/io/pcd_io.h>

// #include <pcl/visualization/pcl_visualizer.h>

#include <cuda_runtime.h>


#include "gpu_transform.h"

using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;
using PointT = pcl::PointXYZ;

using namespace std::chrono;

void PrintGPUInfo();

int main(int argc,char **argv){

	if(argc < 2 ){
		std::cerr << "usage: gputransform [N elements] " << std::endl;
		exit(-1);
	}

	int N = atoi(argv[1]);	

	PrintGPUInfo();
	

	PointCloudT cloud;
	cloud.resize(N);
	for(int i=0;i<N;++i){

		cloud.points[i].x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/10));
		cloud.points[i].y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/10));
		cloud.points[i].z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/10));

	}


	std::cout << "Number of Points : " << cloud.size() << std::endl; // Capping @ 1050000
	// std::cout << "MBytes Used: " << (cloud.size()*sizeof(float)*4) / 1048576.0f << " MBytes" << std::endl;

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	float theta = M_PI/4;
	transform(0,0) = cos(theta);
	transform(0,1) = -sin(theta);
	transform(1,0) = sin(theta);
	transform(1,1) = cos(theta);
	transform(0,3) = 5; //yz
	transform(1,3) = 5; //ty
	transform(2,3) = 5; //tz

	PointCloudT transformed_cloud;
	auto start = high_resolution_clock::now();
	transformPointCloud(cloud,transformed_cloud,transform);
	auto cpu_time = duration_cast<microseconds>(high_resolution_clock::now() - start).count();
	

	//print first and last 5
	for(int i=0;i<10;++i){
		std::cout << cloud.points[i] << " || " << cloud.points[N-i] << std::endl;		
	}

	std::cout << std::endl << std::endl;
		//print first and last 5
	for(int i=0;i<10;++i){
		std::cout << transformed_cloud.points[i] << " || " << transformed_cloud.points[N-i] << std::endl;		
	}

	// CLEAR
	for(auto& it : transformed_cloud){
		it.x = 0;
		it.y = 0;
		it.z = 0;
	}

	// > 530 000 pts bugs
	std::cout << std::endl;
	mygpu::Transform CTransform(cloud,transform);	
	start = high_resolution_clock::now();
	CTransform.Apply(transformed_cloud);
	auto gpu_time = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

	std::cout << std::endl << std::endl;
	//print first and last 5
	for(int i=0;i<10;++i){
		std::cout << transformed_cloud.points[i] << " || " << transformed_cloud.points[N-i] << std::endl;		
	}

	std::cout << "CPU TIME: " << cpu_time  << " us" << std::endl;
	std::cout << "GPU TIME: " << gpu_time  << " us" << std::endl;


    
}





void PrintGPUInfo(){
	int nDevices;
cudaGetDeviceCount(&nDevices);
for (int i = 0;i<nDevices;++i){
cudaDeviceProp prop;
cudaGetDeviceProperties(&prop,i);
std::cout << "Device Number: " << i << std::endl;
std::cout << "Memory Bus Width: " << prop.memoryBusWidth << std::endl;
std::cout << "Memory Clock Rate: " << prop.memoryClockRate << std::endl;
std::cout << "Compute Mode: " << prop.computeMode << std::endl;
std::cout << "Clock Rate: " << prop.clockRate << std::endl;
std::cout << "Total Global Memory: " << prop.totalGlobalMem / 1048576.0f << " MBytes" << std::endl;
std::cout << "Total Shared Memory / Block : " << prop.sharedMemPerBlock << " Bytes" << std::endl;
std::cout << "Warp Size: " << prop.warpSize << std::endl;
std::cout << "MultiProcessor Count: " << prop.multiProcessorCount << std::endl;
std::cout << "Max Threads per multiprocessor: " << prop.maxThreadsPerMultiProcessor << std::endl;
std::cout << "Max Threads per block: " << prop.maxThreadsPerBlock << std::endl;
std::cout << "Max Dimension of thread block(x,y,z): (" << prop.maxThreadsDim[0] << "," << prop.maxThreadsDim[1] << "," << prop.maxThreadsDim[2] << ")" << std::endl;
std::cout << "Max Dimension of grid size(x,y,z): (" << prop.maxGridSize[0] << "," << prop.maxGridSize[1] << "," << prop.maxGridSize[2] << ")" << std::endl;


}
}