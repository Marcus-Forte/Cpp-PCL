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

#include <pcl/filters/extract_indices.h>

// Utilidades (Classe)
#include "PCUtils.h"
#include "delaunator.hpp"

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudT2D = pcl::PointCloud<pcl::PointXY>;


void Interpolation(const PointCloudT &input, PointCloudT &interpolated, double resolution)
{
	pcl::PointXYZ min,max;
	pcl::getMinMax3D(input,min,max);


	



}

// Ideia -> gerar uma malha 2D de resolução 'res'; pra cada celular da malha, pegar os ponhos de maior altura e calcular o volume do paralelepipedo
float computeVolume(const PointCloudT::ConstPtr &cloud_in, float res);

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

	// std::cout.precision(10);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr processing_cloud(new pcl::PointCloud<pcl::PointXYZ>); // Cloud to be processed

	PCUtils::readFile(argv[1], *cloud);

	float v_res = atof(argv[2]);

	if (v_res < 0.001)
	{
		PCL_ERROR("Please insert a valid (>0.001) volume resolution!");
		PrintUsage(argv[0]);
		return 0;
	}

	Interpolation(*cloud, *processing_cloud, v_res);

	cout << "Normal V: " << computeVolume(cloud, v_res) << std::endl;
	cout << "Interpolated V: " << computeVolume(processing_cloud, v_res) << std::endl;

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

	// float vol = PCUtils::computeVolume(cloud, v_res);
	// std::cout << "volume = " << vol << std::endl;
	// PCUtils::startThreadedViewer();
	// while(1);

	return 0;
}

float computeVolume(const PointCloudT::ConstPtr &cloud_in, float res)
{

	pcl::PointXYZ min, max;
	pcl::getMinMax3D(*cloud_in, min, max);
	// std::cout << min << max << std::endl;
	int size_x = int((max.x + res - min.x) / res);
	int size_y = int((max.y + res - min.y) / res);
	Eigen::MatrixXf Cells;

	Cells.resize(size_x, size_y);

	Cells = Eigen::MatrixXf::Zero(size_x, size_y);

	// std::cout << Cells << std::endl;
	// std::cout << "size = "
	// 		  << "(" << size_x << "," << size_y << ")" << std::endl;

	pcl::IndicesPtr pointIdxVec = pcl::make_shared<pcl::Indices>();

	pcl::ExtractIndices<pcl::PointXYZ> extractor;
	extractor.setInputCloud(cloud_in);
	extractor.setIndices(pointIdxVec);

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(res);

	pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	octree.setInputCloud(cloud_in);

	// std::cout << "cloud size : " << cloud_in->size() << std::endl;
	octree.addPointsFromInputCloud();

	std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> c_list;

	pcl::PointXYZ box_max(min.x + res, min.y + res, max.z);
	pcl::PointXYZ box_min = min;
	box_max.z = max.z;

	int i = 0;
	int j = 0;

	pcl::PointXYZ centroid;

	pcl::PointXYZ _min, _max;
	double ground = min.z;

	while (box_max.y < max.y + res)
	{

		j = 0;

		while (box_max.x < max.x + res)
		{

			octree.boxSearch(box_min.getVector3fMap(), box_max.getVector3fMap(), *pointIdxVec);
			extractor.filter(*box_cloud);

			if (box_cloud->size())
			{
				pcl::getMinMax3D(*box_cloud, _min, _max);
				pcl::computeCentroid(*box_cloud, centroid);

				// Maximum Height
				Cells(j, i) = _max.z - ground; // GROUND

				// Centroid

				// FLOOR
			}
			else
			{
				Cells(j, i) = 0; // Interpolar aqui. Marcar células p/ interpolar. (média dos adjacentes termos )
			}

			box_max.x += res;
			box_min.x += res;

			j++;
		}

		box_min.y += res;
		box_max.y += res;

		box_max.x = min.x + res;
		box_min.x = min.x;

		i++;
	}

	return Cells.sum() * res * res;
}