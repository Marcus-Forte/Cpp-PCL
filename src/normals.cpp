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
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//  tree->setInputCloud(cloud);
  ne.setInputCloud (cloud);
  ne.setSearchMethod (tree);
//  ne.setKSearch(20);
  ne.setRadiusSearch (0.4);
  ne.compute (*normals);
  

  float nx,ny,nz;
  float normals_sum = 0;
	unsigned int valid_normals = 0;
	std::cout << "size = " << normals->size() << std::endl;
	for(int i=0;i<normals->size();++i){
//	std::cout << "x normal = " << normals->points[i].normal_x << std::endl;

	nx = normals->points[i].normal_x;
	ny = normals->points[i].normal_y;
	nz = normals->points[i].normal_z;

	std::cout << nx << "	" << ny << "	" << nz << std::endl;
	
	if(!std::isnan(nx) && !std::isnan(ny) && !std::isnan(nz)) {
	normals_sum += sqrt(nx*nx + ny*ny + nz*nz);
	valid_normals++;

	}
	}

		std::cout << "area = " << normals_sum << "valid normals" << valid_normals << std::endl;

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointNormal>);
 pcl::concatenateFields(*cloud,*normals,*cloud_out);
 
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
