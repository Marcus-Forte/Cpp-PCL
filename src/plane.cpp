#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <thread>
#include <chrono>

#include <Eigen/Dense>

#include "vtkPlaneSource.h"

vtkSmartPointer<vtkPolyData> createPlane(const pcl::ModelCoefficients &coefficients, double x, double y, double z, double scale);



int main(int argc, char** argv){
	//cloud = ReadTxt("pontos.txt");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	if(argc < 2){
		std:cerr << "insira o arquivo para carregar" << std::endl;
		return -1;
	}

	std::string filename (argv[1]);
	// std::cout << filename << std::endl;

	if ( pcl::io::loadPCDFile(filename,*cloud) == -1){
		PCL_ERROR("Nao deu pra abrir arquivo \n");
		return -1;
	} 

	std::cout << "Size = " << cloud->size() << std::endl;
	pcl::PointXYZ min,max;
	pcl::getMinMax3D(*cloud,min,max);
	std::cout << "max x = " << max.x << std::endl;
	std::cout << "min x = " << min.x << std::endl;
	//Filtro "PassThrough"

	pcl::PointIndices filtered_indices;
	std::vector<int>indices;

	pcl::PointIndices innn;

	pcl::PassThrough<pcl::PointXYZ> pfilter;
	pfilter.setInputCloud(cloud);
	pfilter.setFilterFieldName("x");
	pfilter.setFilterLimits(max.x-2,max.x+2);
	pfilter.filter(*cloud_filtered);

	
	pcl::ModelCoefficients coefficients;
	pcl::PointIndices inliers;
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(1);


	// seg.setInputCloud(cloud);
	seg.setInputCloud(cloud_filtered); // vem do passthrough
	seg.segment(inliers,coefficients);




  if (inliers.indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients.values[0] << " " 
                                      << coefficients.values[1] << " "
                                      << coefficients.values[2] << " " 
                                      << coefficients.values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers.indices.size () << std::endl;
  for (std::size_t i = 0; i < inliers.indices.size (); ++i){
	  cloud_inliers->push_back(cloud->points[inliers.indices[i]]);


    // std::cerr << inliers.indices[i] << "    " << cloud->points[inliers.indices[i]].x << " "
    //                                            << cloud->points[inliers.indices[i]].y << " "
    //                                            << cloud->points[inliers.indices[i]].z << std::endl;


  }

double x0 = cloud_inliers->points[0].x;
double y0 = cloud_inliers->points[0].y;
double z0 = cloud_inliers->points[0].z;

pcl::ModelCoefficients coeff_test;
coeff_test.values.resize(4);
coeff_test.values[0] = 1;
coeff_test.values[1] = 1;
coeff_test.values[2] = 1;
coeff_test.values[3] = 0;

pcl::PointXYZ p1;
p1.x = cloud_inliers->points[0].x + coefficients.values[0];
p1.y = cloud_inliers->points[0].y + coefficients.values[1];
p1.z = cloud_inliers->points[0].z + coefficients.values[2];


// vtkSmartPointer<vtkPolyData> plane = createPlane(coefficients,x0,y0,z0,2); // 1 = cumprimento do eixo


pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
viewer->setBackgroundColor(0,0,0);
 viewer->addPointCloud<pcl::PointXYZ> (cloud, "input cloud");
 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_inliers, 0, 255, 0); //pinta de verde
 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_(cloud_filtered, 255, 0, 0); //pinta de vermelho
viewer->addPointCloud<pcl::PointXYZ> (cloud_inliers,single_color, "final cloud");
//viewer->addPointCloud<pcl::PointXYZ> (cloud_filtered,single_color_, "passthrough");
viewer->addPlane(coefficients,"plane");
//viewer->addPlane(coeff_test,"plane2");
//viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");
//viewer->addArrow(p1,cloud_inliers->points[0],1,0,0,"arr");
//viewer->addModelFromPolyData(plane,"myplane");



 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input cloud");
 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "final cloud");
 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "passthrough");
 
 viewer->addCoordinateSystem (1.0);
 viewer->initCameraParameters();
 viewer->setCameraPosition(0,0,10,0,0,0,1);

 pcl::io::savePCDFile("filtered.pcd",*cloud_filtered,true);
 pcl::io::savePCDFile("inliers.pcd",*cloud_inliers,true);
 
   


	while(!viewer->wasStopped()){
	viewer->spinOnce (100);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	}
	return 0;
}



vtkSmartPointer<vtkPolyData> createPlane(const pcl::ModelCoefficients &coefficients, double x, double y, double z, double scale = 1)
{
	vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();


	double norm_sqr = 1.0 / (coefficients.values[0] * coefficients.values[0] +
		coefficients.values[1] * coefficients.values[1] +
		coefficients.values[2] * coefficients.values[2]);

	plane->SetNormal(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
	double t = x * coefficients.values[0] + y * coefficients.values[1] + z * coefficients.values[2] + coefficients.values[3];
	x -= coefficients.values[0] * t * norm_sqr;
	y -= coefficients.values[1] * t * norm_sqr;
	z -= coefficients.values[2] * t * norm_sqr;

	//gerar dois vetores ortogonais à normal e entre si

	Eigen::Vector3d p1,p2;
	Eigen::Vector3d n;
	n.x() = x + coefficients.values[0];
	n.y() = y + coefficients.values[1];
	n.z() = z + coefficients.values[2];
	
	p1.x() = x+1.0; //pega um x arbritrário
	p1.y() = y;
	p1.z() = (coefficients.values[3] - coefficients.values[0]*p1.x() - coefficients.values[1]*p1.y())/coefficients.values[2];

	p2 = n.cross(p1);

	p1.normalize();
	p2.normalize();

	p1 = p1*scale;
	p2 = p2*scale;
	

	
	double point1[3];
	double point2[3];

	point1[0] = p1.x();
	point1[1] = p1.y();
	point1[2] = p1.z();

	point2[0] = p2.x();
	point2[1] = p2.y();
	point2[2] = p2.z();

	std::cout << "p1 = " << p1 << std::endl;
	std::cout << "p2 = " << p2 << std::endl;
	

	plane->SetOrigin(x, y, z);
	plane->SetPoint1(point1);
	plane->SetPoint2(point2);

	plane->Update();


	return (plane->GetOutput());
}
