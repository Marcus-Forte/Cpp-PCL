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
#include "txt2pc.h"

// Transformações
#include <pcl/common/transforms.h>


std::atomic<bool> done{false};



float signedVolumeOfTriangle(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3) 
{
		float v321 = p3.x*p2.y*p1.z;
		float v231 = p2.x*p3.y*p1.z;
		float v312 = p3.x*p1.y*p2.z;
		float v132 = p1.x*p3.y*p2.z;
		float v213 = p2.x*p1.y*p3.z;
		float v123 = p1.x*p2.y*p3.z;
		return (1.0f/6.0f)*(-v321 + v231 + v312 - v132 - v213 + v123);
}


float volumeOfMesh(const pcl::PolygonMesh::ConstPtr& mesh)
{
    float vols = 0.0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::fromPCLPointCloud2(mesh->cloud,*cloud);

    for(int triangle=0;triangle<mesh->polygons.size();triangle++)
    {
        pcl::PointXYZ pt1 = cloud->points[mesh->polygons[triangle].vertices[0]];
        pcl::PointXYZ pt2 = cloud->points[mesh->polygons[triangle].vertices[1]];
        pcl::PointXYZ pt3 = cloud->points[mesh->polygons[triangle].vertices[2]];
        vols += signedVolumeOfTriangle(pt1, pt2, pt3);
    }
    return abs(vols);
}


void concHULL(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in, pcl::PolygonMesh::Ptr mesh, float alpha){

		std::cout << "starting thread" << std::endl;
		pcl::ConcaveHull<pcl::PointXYZ> hull;
		hull.setInputCloud(cloud_in);
		hull.setAlpha(alpha);
		hull.reconstruct(*mesh);
		std::cout << "finished thread" << std::endl;
		std::cout << "Alpha : " << alpha << "| Mesh size: " << mesh->polygons.size() << std::endl;

		done = true;	

}



int main(int argc, char** argv){
		//cloud = ReadTxt("pontos.txt");

		if(argc < 2){
				std::cerr << "Please give an .pcd argument" << std::endl;
				exit(-1);
		}

		std::string cloudfile = argv[1];

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);


		if ( pcl::io::loadPCDFile(cloudfile,*cloud) == -1){
				PCL_ERROR("Nao deu pra abrir arquivo \n");
				return -1;
		} 


		// pcl::PassThrough<pcl::PointXYZ> pass;
		// pass.setInputCloud (cloud);
		// pass.setFilterFieldName ("z");
		// pass.setFilterLimits (0, 50.0);
		// pass.filter (*cloud_filtered);
		// std::cerr << "PointCloud after filtering has: "
		// 		<< cloud_filtered->points.size () << " data points." << std::endl;

		// pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		// pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// // Create the segmentation object
		// pcl::SACSegmentation<pcl::PointXYZ> seg;
		// // Optional
		// seg.setOptimizeCoefficients (true);
		// // Mandatory
		// seg.setModelType (pcl::SACMODEL_PLANE);
		// seg.setMethodType (pcl::SAC_RANSAC);
		// seg.setDistanceThreshold (0.01);

		// seg.setInputCloud (cloud_filtered);
		// seg.segment (*inliers, *coefficients);
		// std::cerr << "PointCloud after segmentation has: "
		// 		<< inliers->indices.size () << " inliers." << std::endl;

		// // Project the model inliers
		// pcl::ProjectInliers<pcl::PointXYZ> proj;
		// proj.setModelType (pcl::SACMODEL_PLANE);
		// proj.setIndices (inliers);
		// proj.setInputCloud (cloud_filtered);
		// proj.setModelCoefficients (coefficients);
		// proj.filter (*cloud_projected);
		// std::cerr << "PointCloud after projection has: "
		// 		<< cloud_projected->points.size () << " data points." << std::endl;



		//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
		pcl::visualization::PCLVisualizer viewer("3D Viewer");




		//Concave and Convex Hull
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>); //Chull


		//		pcl::PolygonMesh concave_mesh;
		/*pcl::ConcaveHull<pcl::PointXYZ> chull;
		  chull.setInputCloud(cloud);
		  chull.setAlpha(0.1);
		  chull.reconstruct(*cloud_hull);
		  */

		pcl::PolygonMesh::Ptr convex_mesh (new pcl::PolygonMesh);
		pcl::ConvexHull<pcl::PointXYZ> convex;
		convex.setComputeAreaVolume(true);
		convex.setInputCloud(cloud);
		convex.reconstruct(*convex_mesh);
		std::cout << "Convex Hull Volume " << convex.getTotalVolume() << std::endl;
		std::cout << "Number of meshes : " <<convex_mesh->polygons.size() << std::endl;
		std::cout << "My volume = " << volumeOfMesh(convex_mesh) << std::endl;
		pcl::io::savePLYFile("convex.ply",*convex_mesh);
		std::cout << "Mesh saved" << std::endl;


		

		// 		pcl::ConcaveHull<pcl::PointXYZ> concave;
		// 		concave.setInputCloud(cloud);

		//float a = concave.getTotalVolume();	

		//		std::cout << "ConvHull Volume = " << convex.getTotalVolume() << std::endl;

		//	viewer.showCloud(cloud);
		//viewer2.showCloud(cloud);

		// 
		// 
		// 		pcl::io::savePLYFile("concave.ply",concave_mesh);
		// 		std::cout << "Concave Mesh saved" << std::endl;
		// 		pcl::io::savePCDFileASCII("cloud_projected.pcd",*cloud_projected);

		float alpha = 0.001;
		viewer.setBackgroundColor(0,0,0);
		// viewer.addPointCloud(cloud,"nuvem");
		pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
		// pcl::PointCloud<pcl::PointXYZ>::Ptr tf_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	
		// Eigen::Affine3f transform;
		// Eigen::Vector3f vec;
		// vec[0] = 1;
		// vec[1] = 1;
		// vec[2] = 1;
		// transform.translation() = vec;
		// pcl::transformPointCloud(*cloud,*tf_cloud,transform);
		// viewer.addPointCloud(tf_cloud,"nuvem2");

		// bool is false up to here
		//		concHULL(cloud,mesh,alpha);

		std::thread t (concHULL,cloud,mesh,alpha);
		t.join();
		while(!viewer.wasStopped()){
				viewer.spinOnce(100);

				if(done){


						viewer.removePolygonMesh("mesh");
						viewer.addPolygonMesh(*mesh,"mesh");
						alpha += 0.001;
						std::thread t(concHULL,cloud,mesh,alpha);
						t.detach();
						done = false;
				}


				// std::cout <<"Loop" << std::endl;
				//				sleep(1);

		}
		return 0;
}
