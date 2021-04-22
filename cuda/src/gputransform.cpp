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

#include "gpu_transform.h"

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

template <typename T>
struct mallocator
{
	using value_type = T;

	mallocator() = default;
	template <class U>
	mallocator(const mallocator<U> &) {}

	T *allocate(std::size_t n)
	{
		std::cout << "allocate(" << n << ") = ";
		if (n <= std::numeric_limits<std::size_t>::max() / sizeof(T))
		{
			void* ptr;
			cudaMallocManaged(&ptr,sizeof(T));
			// if (auto ptr = std::malloc(n * sizeof(T)))
			// {
			// 	return static_cast<T *>(ptr);
			// }
			return (T*)ptr;
		}
		return nullptr;
		// throw std::bad_alloc();
	}
	void deallocate(T *ptr, std::size_t n)
	{
		cudaFree(ptr);
		// std::free(ptr);
	}
};

class Managed
{
public:
	void *operator new(size_t len)
	{
		void *ptr;
		printf("Calling managed.");
		cudaMallocManaged(&ptr, len);
		cudaDeviceSynchronize();
		return ptr;
	}

	void operator delete(void *ptr)
	{
		cudaDeviceSynchronize();
		cudaFree(ptr);
	}
};

using namespace std::chrono;

bool isSamePoint(const PointT &p0, const PointT &p1)
{
	static const float approx = 0.001;

	if (fabs(p0.x - p1.x) > approx)
		return false;
	if (fabs(p0.y - p1.y) > approx)
		return false;
	if (fabs(p0.z - p1.z) > approx)
		return false;

	return true;
}

int main(int argc, char **argv)
{

	if (argc < 2)
	{
		std::cerr << "usage: gputransform [N elements] " << std::endl;
		exit(-1);
	}

	int nDevices;

	cudaGetDeviceCount(&nDevices);

	if (nDevices == 0)
	{
		std::cerr << "No CUDA devices found." << std::endl;
		exit(-1);
	}

	cudaError_t err;

	int N = atoi(argv[1]);

	class testType : public Managed
	{
	public:
		float a;
		float b;
		// std::vector<float, mallocator<float>> vec;
		float* vec;
	};

	testType *obj = new testType;
	
	cudaMallocManaged(&obj->vec,sizeof(float));
	// obj->vec.resize(25);


	cudaMallocManaged(&obj, sizeof(testType));

	cudaPointerAttributes attributes;
	err = cudaPointerGetAttributes(&attributes, obj);
	std::cout << "Type: " << attributes.type << std::endl;

	err = cudaPointerGetAttributes(&attributes, obj->vec);
	std::cout << "Member Type: " << attributes.type << std::endl;

	return 0;

	PointCloudT *cloud_;
	// float* f;
	cudaMallocManaged(&cloud_, sizeof(PointCloudT)); // nao adiata .. o vector "points" precisa ser managed tambem

	PointCloudT &cloud = (*cloud_);
	cloud.resize(N);
	for (int i = 0; i < N; ++i)
	{

		cloud.points[i].x = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 10));
		cloud.points[i].y = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 10));
		cloud.points[i].z = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 10));
	}

	std::cout << "Number of Points : " << cloud.size() << std::endl; // C

	Eigen::Matrix4f *transform_;

	cudaMallocManaged(&transform_, sizeof(Eigen::Matrix4f));

	Eigen::Matrix4f &transform = (*transform_);

	transform = Eigen::Matrix4f::Identity();

	// Arbitrary transform
	float theta = 0; // M_PI / 4;
	transform(0, 0) = cos(theta);
	transform(0, 1) = -sin(theta);
	transform(1, 0) = sin(theta);
	transform(1, 1) = cos(theta);
	transform(0, 3) = 0; //yz
	transform(1, 3) = 0; //ty
	transform(2, 3) = 0; //tz

	cudaEvent_t start, stop;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);

	PointCloudT transformed_cloud;

	cudaEventRecord(start, 0);
	transformPointCloud(cloud, transformed_cloud, transform);
	cudaEventRecord(stop, 0);
	cudaEventSynchronize(stop);
	float elapsed_cpu_ms;
	cudaEventElapsedTime(&elapsed_cpu_ms, start, stop);

	std::cout << "CPU Time: " << elapsed_cpu_ms << " ms" << std::endl;

	PointCloudT *gpu_transformed_cloud_;
	cudaMallocManaged(&gpu_transformed_cloud_, sizeof(PointCloudT));

	PointCloudT &gpu_transformed_cloud = (*gpu_transformed_cloud_);
	// gpu_transformed_cloud_->resize(N);

	// pcl::PointXYZ *ptr = gpu_transformed_cloud.points.data();

	cudaEventRecord(start, 0);
	// gpu::Transform(cloud, gpu_transformed_cloud, transform);
	gpu::TransformUnified(cloud_, gpu_transformed_cloud_, transform_);
	cudaDeviceSynchronize();
	cudaEventRecord(stop, 0);
	cudaEventSynchronize(stop);
	float elapsed_gpu_ms;
	cudaEventElapsedTime(&elapsed_gpu_ms, start, stop);

	std::cout << "GPU Time: " << elapsed_gpu_ms << " [ms]" << std::endl;

	// Check Results
	bool allGood = true;
	for (int i = 0; i < transformed_cloud.points.size(); ++i)
	{
		if (!isSamePoint(transformed_cloud.points[i], gpu_transformed_cloud.points[i]))
		{
			allGood = false;
			std::cout << "Wrong Results @ i=" << i << std::endl;
			std::cout << transformed_cloud.points[i] << std::endl;
			std::cout << gpu_transformed_cloud.points[i] << std::endl;
			break;
		}
	}

	if (allGood)
		std::cout << "All Good!" << std::endl;
}
