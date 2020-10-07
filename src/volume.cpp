#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <thread>

// Utilidades (Classe)
#include "PCUtils.h"


void PrintUsage(const char *progName)
{
	std::cout << "\n\nUsage: " << progName << " cloudfile resolution\n\n";
}

int main(int argc, char **argv)
{

	if (argc < 3)
	{
		PrintUsage(argv[0]);
		return 0;
	}

	std::cout.precision(10);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr processing_cloud(new pcl::PointCloud<pcl::PointXYZ>); // Cloud to be processed

	PCUtils::readFile(argv[1],*cloud);

	float v_res = atof(argv[2]);

	if (v_res < 0.001)
	{
		PCL_ERROR("Please insert a valid (>0.001) volume resolution!");
		PrintUsage(argv[0]);
		return 0;
	}

	float vol = PCUtils::computeVolume(cloud, v_res);
	std::cout << "volume = " << vol << std::endl;
	// PCUtils::startThreadedViewer();
	// while(1);

	return 0;

}
