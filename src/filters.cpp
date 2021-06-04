#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

#include <unistd.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

#include "PCUtils.h"

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

int main(int argc, char **argv)
{

	PointCloudT::Ptr cloud(new PointCloudT);

	if (argc < 5)
	{
		std::cerr << "Usage : filters cloud.pcd [res] [sor neighboorhood] [sor deviation]" << std::endl;
		std::cerr << "Recommended values : res = 0.1, sor neighboorhood = 50, sor deviation = 1" << std::endl;
		exit(-1);
	}

	std::cout << "Loading input point clouds..." << std::endl;
	if (PCUtils::readFile(argv[1], *cloud) != 0)
		exit(-1);

	std::cout << "Loaded: " << cloud->size() << " points." << std::endl;

	PointCloudT::Ptr cloud_filtered(new PointCloudT);
	PointCloudT::Ptr cloud_filtered2(new PointCloudT); //Voxel Grid
	PointCloudT::Ptr cloud_filtered3(new PointCloudT); // SOR Filter

	clock_t start = start, end;

	float res = atof(argv[2]);
	std::cout << "res = " << res << std::endl;
	start = clock();
	std::cout << "Computing Grid Minimum..." << std::endl;
	pcl::GridMinimum<PointT> grid(res); //resolution
	grid.setInputCloud(cloud);
	grid.filter(*cloud_filtered);
	end = clock();
	std::cout << "OK! Number of points: " << cloud_filtered->size() << "( " << (float)(end - start) / CLOCKS_PER_SEC << " s)" << std::endl;
	//

	start = clock();
	std::cout << "Computing Voxel Grid..." << std::endl;
	pcl::VoxelGrid<PointT> voxel;
	voxel.setInputCloud(cloud);
	voxel.setLeafSize(res, res, res);
	voxel.filter(*cloud_filtered2);
	end = clock();
	std::cout << "OK! Number of points: " << cloud_filtered2->size() << "( " << (float)(end - start) / CLOCKS_PER_SEC << " s)" << std::endl;
	//
	// MeanK = 200, StdThresh = 5 for stockpile
	if (atoi(argv[3]) != 0.0)
	{
		start = clock();
		std::cout << "Computing SOR..." << std::endl;
		pcl::StatisticalOutlierRemoval<PointT> sor;
		sor.setInputCloud(cloud);
		sor.setMeanK(atoi(argv[3]));
		sor.setStddevMulThresh(atof(argv[4]));
		sor.filter(*cloud_filtered3);
		end = clock();
		std::cout << "OK! Number of points: " << cloud_filtered3->size() << "( " << (float)(end - start) / CLOCKS_PER_SEC << " s)" << std::endl;
	}

	// cloudname
	std::string arg(argv[1]);
	std::string grid_cloudname, voxel_cloudname, sor_cloudname;
	int slash = arg.find_last_of('/'); //path
	int dot = arg.find_last_of('.');   //extension
	grid_cloudname = arg.substr(slash + 1, dot - slash - 1) + "_" + "gridded.pcd";
	voxel_cloudname = arg.substr(slash + 1, dot - slash - 1) + "_" + "voxeled.pcd";
	sor_cloudname = arg.substr(slash + 1, dot - slash - 1) + "_" + "sored.pcd";

	std::cout << "Wrting output clouds..." << std::endl;
	pcl::io::savePCDFileBinary(grid_cloudname, *cloud_filtered);
	PCL_INFO("subsampled cloud '%s' saved successfully \n", grid_cloudname.c_str());
	pcl::io::savePCDFileBinary(voxel_cloudname, *cloud_filtered2);
	PCL_INFO("subsampled cloud '%s' saved successfully \n", voxel_cloudname.c_str());
	// pcl::io::savePCDFileBinary(sor_cloudname, *cloud_filtered3);
	// PCL_INFO("subsampled cloud '%s' saved successfully \n", sor_cloudname.c_str());

	return 0;
}
