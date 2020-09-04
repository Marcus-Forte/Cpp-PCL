#include <pcl/point_cloud.h>
#include "PCUtils.h"
#include <pcl/filters/crop_hull.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/surface/convex_hull.h>

int main(int argc,char** argv){

	//CloudType::Ptr cloud (new CloudType);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr crop_zone(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr crop_zone2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	

	if (argc < 2)
	{
		std::cerr << "Usage : vizualise cloud.pcd ..." << std::endl;
		exit(-1);
	}

	if (pcl::io::loadPCDFile(argv[1], *cloud_in) != 0) //TODO verificar ausencia do field rgb com try catch
		exit(-1);


    float vertex = 0.25;
    pcl::PointXYZ pt;
    pt.x = vertex;
    pt.y = vertex;
    pt.z = 1.5;
    crop_zone->push_back(pt);
    pt.x = vertex;
    pt.y = -vertex;    
    crop_zone->push_back(pt);
    pt.x = -vertex;
    pt.y = vertex;
    crop_zone->push_back(pt);
    pt.x = -vertex;
    pt.y = -vertex;
    crop_zone->push_back(pt);

    pt.x = vertex;
    pt.y = vertex;
    pt.z = -1;
    crop_zone->push_back(pt);
    pt.x = vertex;
    pt.y = -vertex;    
    crop_zone->push_back(pt);
    pt.x = -vertex;
    pt.y = vertex;
    crop_zone->push_back(pt);
    pt.x = -vertex;
    pt.y = -vertex;
    crop_zone->push_back(pt);
    

pcl::ConvexHull<pcl::PointXYZ> convex_hull;
std::vector<pcl::Vertices> indices;
convex_hull.setInputCloud(crop_zone);
// convex_hull.setDimension(3);
convex_hull.setComputeAreaVolume(true);
convex_hull.reconstruct(indices);
std::cout << "volume = " << convex_hull.getTotalVolume() << std::endl;

pcl::CropHull<pcl::PointXYZ> cropper;
cropper.setInputCloud(cloud_in);
cropper.setHullCloud(crop_zone);
//cropper.setHullIndices(crop_zone->);
cropper.setDim(3);
cropper.setCropOutside(true);
cropper.filter(*cloud_out);

std::cout << "cloud out size: " << cloud_out->size() << std::endl;


// Cropbox funciona legal!
// pcl::CropBox<pcl::PointXYZ> crop_box;

// Eigen::Vector4f min,max;
// min[0] = -0.25;
// min[1] = -0.25;
// min[2] = -100;
// min[3] = 1;

// max[0] = 0.25;
// max[1] = 0.25;
// max[2] = 100;
// max[3] = 1;


// crop_box.setMin(min);
// crop_box.setMax(max);
// crop_box.setInputCloud(cloud_in);
// crop_box.filter(*cloud_out);



pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(crop_zone, 0, 255, 0); //green
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud_out, 255, 0, 0); //red
viewer->addPointCloud(cloud_in);
viewer->addPointCloud(crop_zone,single_color,"zone");
viewer->addPointCloud(cloud_out,single_color2,"output");
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "zone");
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "output");

// viewer->addCoordinateSystem(1,"Ref");



while (!viewer->wasStopped()){
    viewer->spin();
}



return 0;


}