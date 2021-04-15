#ifndef TRANSFORM_
#define TRANSFORM_

#include <cuda_runtime.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace gpu
{
   void TransformUnified(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &cloud_out, const Eigen::Matrix4f &transform);
   void Transform(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &cloud_out, const Eigen::Matrix4f &transform);
}

#endif
