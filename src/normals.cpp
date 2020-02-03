#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/cloud_viewer.h>


#define WORKINGPATH "/home/projeto/Workspace/Coding/C++/CMake/PCL/"

void PrintCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);


int main()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 if( pcl::io::loadPCDFile<pcl::PointXYZ> (WORKINGPATH "cloud_test.pcd",*cloud) == -1){
	PCL_ERROR("Deu ruim p carregar");
	return -1;
 }

	PrintCloud(cloud);
 

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.5);

  // Compute the features
  ne.compute (*cloud_normals);

  float nx = 0 ,ny = 0 ,nz = 0;
	
	std::cout << "size = " << cloud_normals->size() << std::endl;
	for(int i=0;i<cloud_normals->size();++i){

	nx += cloud_normals->points[i].normal_x;
	ny += cloud_normals->points[i].normal_y;
	nz += cloud_normals->points[i].normal_y;

	}

	std::cout << "area = " << nx << " " << ny << " " << nz << std::endl;

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointNormal>);
 pcl::concatenateFields(*cloud,*cloud_normals,*cloud_out);
 
	  pcl::io::savePCDFileASCII(WORKINGPATH "normals_output_ascii.pcd",*cloud_out);
  pcl::io::savePCDFileBinary(WORKINGPATH "normals_output_bin.pcd",*cloud_out);

  return 0;

//  pcl::visualization::CloudViewer viewer("Cloud Viewer");
//  viewer.showCloud(cloud);

//  while(!viewer.wasStopped()){

//  }

  return 0;


  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
}



void PrintCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud){
int size = cloud->points.size();

for(unsigned int i=0;i<size;++i){
	std::cout << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;

}

}
