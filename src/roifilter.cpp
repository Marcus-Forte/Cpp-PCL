#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/surface/convex_hull.h>

#include "PCUtils.h"

static void extrude(const pcl::PointCloud<pcl::PointXYZ>& cloud_in,const pcl::PointCloud<pcl::PointXYZ>& crop_zone,pcl::PointCloud<pcl::PointXYZ>& crop_zone3D){
	pcl::PointXYZ min,max;
	pcl::getMinMax3D(cloud_in,min,max);
	crop_zone3D = crop_zone; // Copy ?
	
	for(int i=0;i<crop_zone.size();++i){
		pcl::PointXYZ newpt(crop_zone.points[i].x,crop_zone.points[i].y,max.z+10);
		crop_zone3D.points[i].z = min.z-10;
		crop_zone3D.push_back(newpt);
	}

}

int main(int argc,char** argv){

	//CloudType::Ptr cloud (new CloudType);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr crop_zone(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr crop_zone3D(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	

	if (argc < 3)
	{
		std::cerr << "Usage : roifilter cloud.pcd cropbox.pcd ..." << std::endl;
		exit(-1);
	}


	
	std::string filename = argv[1]; //Cloud name
	size_t final = filename.find_last_of(".");
	size_t init = filename.find_last_of("/");

	
	std::string newfilename;
	newfilename  = filename.substr(init+1,final-init-1); // Filtered ROI filename


	std::cout << newfilename << std::endl;

	PCUtils::readFile(argv[1],*cloud_in);
	PCUtils::readFile(argv[2],*crop_zone);


	extrude(*cloud_in,*crop_zone,*crop_zone3D);

	pcl::ConvexHull<pcl::PointXYZ> convexhull;
	std::vector<pcl::Vertices> polygons;
	pcl::PolygonMesh mesh;
	convexhull.setInputCloud(crop_zone3D);
	convexhull.setDimension(3);

	convexhull.reconstruct(polygons);
	convexhull.reconstruct(mesh);

	pcl::CropHull<pcl::PointXYZ> cropper;
	cropper.setInputCloud(cloud_in);
	cropper.setHullCloud(crop_zone3D);
	cropper.setHullIndices(polygons);
	cropper.setDim(2);
	cropper.setCropOutside(true);
	cropper.filter(*cropped_cloud);
	




// Cropbox funciona muito bem
//   pcl::CropBox<pcl::PointXYZ> crop_box;
//  
//   Eigen::Vector4f min,max;
//   pcl::getMinMax3D(*crop_zone,min,max);
//  
//   crop_box.setMin(min);
//   crop_box.setMax(max);
//   crop_box.setInputCloud(cloud_in);
//   crop_box.filter(*cloud_out);



std::cout << "cloud out size: " << cropped_cloud->size() << std::endl;
pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_(crop_zone, 0, 255, 0); //green
// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_(cloud_out, 255, 0, 0); //red
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_(cropped_cloud, 0, 0, 255); //blue
viewer->addPointCloud(cloud_in);
viewer->addPointCloud(crop_zone,green_,"zone");
//viewer->addPointCloud(cloud_out,red_,"output");
viewer->addPointCloud(cropped_cloud,blue_,"cropped");
viewer->addPolygonMesh(mesh,"mesh");
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "zone");
// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "output");
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cropped");
viewer->addCoordinateSystem(3,"ref");


pcl::io::savePCDFileBinary(newfilename + "_filtered.pcd",*cropped_cloud);
std::cout << "ROI filtered cloud saved successfully." << std::endl;



while (!viewer->wasStopped()){
    viewer->spin();
}





return 0;


}
