#pragma once

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/console/print.h>

// Fill
#include <pcl/common/centroid.h>

#include <pcl/search/kdtree.h>
#include <pcl/octree/octree_search.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <memory>
#include <fstream>
// Classe utilitária

// TODO migrate header-implemented methods to .cpp

class PCUtils
{
public:
static void setColorMap(int n,int max,pcl::PointXYZRGB& pt){
		float min_color = 0;
		float max_color = 240;
		//Normaliza
		float t = n;
		float T = 4*max;

		// Blue sine (lowest) começa com 90 graus
		float b = ((std::sin(2*M_PI*t/T + M_PI_2)))*max_color; // ineficiente
		//Green sine (mid) // 45 graus
		float g = ((std::sin(3*M_PI*t/T + M_PI_4)))*max_color; 
		// Red sine (highest) // 0 graus
		float r = ((std::sin(2*M_PI*t/T)))*max_color;
		pt.r = r;
		pt.g = g;
		pt.b = b;
		// std::cout << "RGB: " << r << "," << g << "," << b << std::endl;

}
	// This method loads either .pcd or .txt point cloud files.
	template <typename pointT>
	static inline int readFile(const std::string &cloudfile, pcl::PointCloud<pointT> &cloud)
	{

		//								std::cout << "READING FILE" << std::endl;
		int index = cloudfile.find_last_of('.');
		std::string extension = cloudfile.substr(index + 1);

		if (extension.compare("pcd") == 0)
		{
			if (pcl::io::loadPCDFile(cloudfile, cloud) == -1)
			{
				PCL_ERROR("Cloud not open .pcd file.\n");
				exit(-1);
			}
		}
		else if (extension.compare("txt") == 0)
		{

			if (readTxt(cloudfile, cloud) == -1)
			{
				PCL_ERROR("Cloud not open .txt file.\n");
				exit(-1);
			}
		} else if (extension.compare("ply") == 0) {

			if(pcl::io::loadPLYFile(cloudfile,cloud) == -1) {
					PCL_ERROR("Could not open .ply file. \n");
					exit(-1);
			}

		}

		else
		{

			PCL_ERROR("format not supported!\n");
			exit(-1);
		}

		return 0;
	}

	//TODO Make floor smart | Improve Herustics
	static void makeFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, float density);

	// TODO make default stuff
	template <typename pointT>
	static void printPoints(const pcl::PointCloud<pointT> &cloud_in, const std::string &name);

	//TODO fill clouds with points between floor and cloud top

	// TODO heuristics
	static void fillCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, int density = 2);
	// 2.5D volume
	static float computeVolume(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_in, float res);

	// Mesh volume ..

private:
	PCUtils() {}
	~PCUtils() {}

	template <typename pointT>
	static inline int readTxt(const std::string &filename, pcl::PointCloud<pointT> &cloud)
	{
		std::ifstream file;
		file.open(filename);

		std::string line;
		// std::cout << "read file" << std::endl;
		pointT pt;
		while (std::getline(file, line))
		{

			// std::cout << "LINE : " << line << std::endl;
			if (line.find("#") != std::string::npos)
			{
				// std::cout << "DEBUG : COMMENTARIO" << std::endl;
				continue;
			}
			else if (line.size() > 1) //non empty
			{
				sscanf(line.c_str(), "%f %f %f", &pt.x, &pt.y, &pt.z);
				// std::cout << line.size() << std::endl;
				cloud.push_back(pt);
			}
		}
		return 0;
	}
};
