#include <iostream>
#include <pcl/ModelCoefficients.h>
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
#include "txt2pc.h"



int main(int argc, char** argv){
	//cloud = ReadTxt("pontos.txt");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
	
	if ( pcl::io::loadPCDFile("project.pcd",*cloud) == -1){
		PCL_ERROR("Nao deu pra abrir arquivo \n");
		return -1;
	} 
	
	
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 50.0);
  pass.filter (*cloud_filtered);
    std::cerr << "PointCloud after filtering has: "
            << cloud_filtered->points.size () << " data points." << std::endl;
	
 pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  std::cerr << "PointCloud after segmentation has: "
            << inliers->indices.size () << " inliers." << std::endl;

  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setIndices (inliers);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  std::cerr << "PointCloud after projection has: "
            << cloud_projected->points.size () << " data points." << std::endl;
	
		

	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	
	
	//Concave and Convex Hull
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>); //Chull
	
	pcl::PolygonMesh mesh;
	pcl::PolygonMesh concave_mesh;
	/*pcl::ConcaveHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cloud);
	chull.setAlpha(0.1);
	chull.reconstruct(*cloud_hull);
	*/
	
	pcl::ConvexHull<pcl::PointXYZ> convex;
	convex.setComputeAreaVolume(true);
	convex.setInputCloud(cloud);
	convex.reconstruct(mesh);
	
	pcl::ConcaveHull<pcl::PointXYZ> concave;
	concave.setInputCloud(cloud);
	concave.setAlpha(5);
	concave.reconstruct(concave_mesh);
	
	
	
	
	std::cout << "ConvHull Volume = " << convex.getTotalVolume() << std::endl;
	//viewer.showCloud(cloud);
	
	pcl::io::savePLYFile("convex.ply",mesh);
	std::cout << "Mesh saved" << std::endl;
	
		pcl::io::savePLYFile("concave.ply",concave_mesh);
	std::cout << "Concave Mesh saved" << std::endl;
   pcl::io::savePCDFileASCII("cloud_projected.pcd",*cloud_projected);
   


	//while(!viewer.wasStopped()){

//	}
	return 0;
}
