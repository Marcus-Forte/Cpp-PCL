#include <iostream>
#include <pcl/point_types.h>

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/statistical_outlier_removal.h>

// Utilidades (Classe)
#include "PCUtils.h"
#include "delabella.h"
#include "compute_volume.hpp"

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudT2D = pcl::PointCloud<pcl::PointXY>;

void PrintUsage(const char *progName)
{
	std::cout << "\n\nUsage: " << progName << " cloudfile [-r resolution] [-g ground]\n\n";
}

int main(int argc, char **argv)
{

	if (argc < 2)
	{
		PrintUsage(argv[0]);
		return 0;
	}
	float resolution = 0.1;
	std::string ground_file;

	pcl::console::parse_argument(argc, argv, "-r", resolution);
	pcl::console::parse_argument(argc, argv, "-g", ground_file);
	bool debug_mode = false;
	if (pcl::console::find_switch(argc, argv, "-d"))
	{
		debug_mode = true;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZ>); // Cloud to be processed

	PCUtils::readFile(argv[1], *cloud);

	// Remove useless ground
	PCL_INFO("Removing noise\n");
	pcl::PassThrough<pcl::PointXYZ> passthrough;
	passthrough.setInputCloud(cloud);
	passthrough.setFilterFieldName("z");
	passthrough.setFilterLimits(0.07, 1);
	passthrough.filter(*cloud);

	volumeEstimator<PointT> estimator(resolution);
	estimator.setInputCloud(cloud);
	estimator.setDebug(debug_mode);
	estimator.SetRegistration(false);

	double volume = 0;
	PCL_INFO("Res: %f\n", resolution);

	if (!ground_file.size())
	{
		PCL_ERROR("No Ground points");
		exit(-1);
	}

	PCL_INFO("Ground file found.\n");

	PCUtils::readFile(ground_file, *ground);

	passthrough.setInputCloud(ground);
	passthrough.filter(*ground);

	estimator.setGroundCloud(ground);
	volume = estimator.compute(*processed_cloud);

	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(processed_cloud);
	sor.setMeanK(10);
	sor.setStddevMulThresh(0.4);
	// sor.filter(*processed_cloud);

	PCL_INFO("Volume: %f \n", volume);
	PCL_INFO("Processed Pts: %d \n", processed_cloud->size());

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color(cloud, "z");
	// std::cout << "cloud" << *cloud << std::endl;
	// std::cout << "processing_cloud" << *processing_cloud << std::endl;

	pcl::visualization::PCLVisualizer viewer;
	int v1, v2;
	viewer.createViewPort(0, 0, 0.5, 1, v1);
	viewer.createViewPort(0.5, 0, 1, 1, v2);
	viewer.addPointCloud(cloud, "cloud", v1);
	viewer.addPointCloud(ground, "ground", v1);
	viewer.addPointCloud(processed_cloud, "diff_cloud", v2);
	// viewer.addPointCloud(processing_cloud, color, "interpolated", v2);
	// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "interpolated");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "ground");

	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "ground");
	// viewer.addCoordinateSystem(1);

	while (!viewer.wasStopped())
	{
		viewer.spin();
	}

	return 0;
}
