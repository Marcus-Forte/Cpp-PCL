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

int main(int argc,char **argv){

	if(argc < 2 ){
		std::cerr << "Error " << std::endl;
	}

	int N = atoi(argv[1]);	
	

	PointCloudT cloud;
	cloud.resize(N);
	for(int i=0;i<N;++i){

		cloud.points[i].x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/10));
		cloud.points[i].y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/10));
		cloud.points[i].z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/10));

	}


	std::cout << "Size: " << cloud.size() << std::endl;

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
	transformPointCloud(cloud,transformed_cloud,transform);

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
	CTransform.Apply(transformed_cloud);

	std::cout << std::endl << std::endl;
	//print first and last 5
	for(int i=0;i<10;++i){
		std::cout << transformed_cloud.points[i] << " || " << transformed_cloud.points[N-i] << std::endl;		
	}


    
}