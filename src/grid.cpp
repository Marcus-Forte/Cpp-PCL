#include <pcl/filters/grid_minimum.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

#define WORKINGPATH "./"

int main(){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 if( pcl::io::loadPCDFile<pcl::PointXYZ> (WORKINGPATH "pilha_menor.pcd",*cloud) == -1){
	PCL_ERROR("Deu ruim p carregar");
	return -1;
 }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
 
 std::cout << "Digite Resolução: ";
 float res;
 std::cin >> res;
 pcl::GridMinimum<pcl::PointXYZ> grid(res); //resolution
 grid.setInputCloud(cloud);
 grid.filter(*cloud_filtered);

 
 pcl::io::savePCDFileBinary( WORKINGPATH "gridded.pcd", *cloud_filtered);
 PCL_INFO("Salvo com sucesso!\n");



}
