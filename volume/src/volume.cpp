#include <iostream>
#include <pcl/point_types.h>

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>

// Utilidades (Classe)
#include "PCUtils.h"
#include "delabella.h"
#include "compute_volume.hpp"


using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudT2D = pcl::PointCloud<pcl::PointXY>;



void PrintUsage(const char *progName)
{
	std::cout << "\n\nUsage: " << progName << " cloudfile [r resolution] [-g ground]\n\n";
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

	pcl::console::parse_argument(argc,argv,"-r",resolution);
	pcl::console::parse_argument(argc,argv,"-g",ground_file);


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr processing_cloud(new pcl::PointCloud<pcl::PointXYZ>); // Cloud to be processed

	PCUtils::readFile(argv[1], *cloud);

	volumeEstimator<PointT> estimator(resolution);

	if(ground_file.size()){
		PCL_INFO("Ground file found.\n");
		PointCloudT::Ptr ground (new PointCloudT);
		PCUtils::readFile(ground_file,*ground);


	} else {

		estimator.setInputCloud(cloud);
		double volume = estimator.compute();
		
	}



	return 0;

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color(processing_cloud, "z");

	// std::cout << "cloud" << *cloud << std::endl;
	// std::cout << "processing_cloud" << *processing_cloud << std::endl;

	pcl::visualization::PCLVisualizer viewer;
	int v1, v2;
	viewer.createViewPort(0, 0, 0.5, 1, v1);
	viewer.createViewPort(0.5, 0, 1, 1, v2);
	viewer.addPointCloud(cloud, "cloud");
	// viewer.addPointCloud(processing_cloud, "interpolated", v2);
	viewer.addPointCloud(processing_cloud, color, "interpolated", 2);
	// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "interpolated");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "interpolated");
	viewer.addCoordinateSystem(1);

	while (!viewer.wasStopped())
	{
		viewer.spin();
	}


	return 0;
}


