#include <iostream>
#include <pcl/common/io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/extract_indices.h>

//#include <pcl/kdtree/kdtree_flann.h>

int main(int argc,char**argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if(argc<4){
		std::cout << "Usage: normals [.pcd] [normal estimation radius] [alpha]" << std::endl;
		std::cout << "Alpha is the 'walking step' for the algorithm " << std::endl;
		std::cout << "Typical alpha values -> 0.01 for small clouds " << std::endl;
		exit(-1);
	}

	

	std::string filename = argv[1];

	float ne_radius = atof(argv[2]);
	float alpha = atof(argv[3]);


	int dot = filename.find_last_of('.');
	std::string extension = filename.substr(dot+1);

	if(extension.compare("pcd") == 0) {
	if(pcl::io::loadPCDFile(filename,*cloud) == -1){
		PCL_ERROR("Error loading point clouds.\n");
		exit(-1);
	}

	} else if(extension.compare("ply") == 0 ) {

	if(pcl::io::loadPLYFile(filename,*cloud) == -1){
		PCL_ERROR("Error loading point clouds.\n");
		exit(-1);
	}

	} else {
		PCL_ERROR("format not supported. \n");
		exit(-1);
	}

	// Estimate normals.. must be tuned to the application. This kinds of dictates how everything else works
	std::cout << "Estimating Normals..." << std::endl;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(ne_radius);  // Depends on the input cloud. Maybe related to our 'alpha' ?
	ne.compute(*cloud_normals);

	
	int start_point_index; //
	int end_point_index; //


	//Kdtree search (just for grabbing points that belong to the cloud)
	std::vector<int> index_vector(1);
	std::vector<float> dummy_arg; //Just for calling kdtree.

	//Start
	pcl::PointXYZ start_point;
	start_point.x = -4.07;
	start_point.y = 44.701;  
	start_point.z = -61.75;  

	//End
	pcl::PointXYZ end_point;
	end_point.x = -40; //12 | 1
	end_point.y = -140; //-21 | 0.5
	end_point.z = -59; //-142 | 0.5

	tree->nearestKSearch(start_point,1,index_vector,dummy_arg);
	start_point_index = index_vector[0];

	tree->nearestKSearch(end_point,1,index_vector,dummy_arg);
	end_point_index = index_vector[0];


	std::cout << "Top point index -> " << start_point_index << std::endl;
	std::cout << "Top Point " << cloud->points[start_point_index] << std::endl;
	std::cout << "Top Point Normal" << cloud_normals->points[start_point_index] << std::endl;

	std::cout << "Side point index -> " << end_point_index << std::endl;
	std::cout << "Side Point " << cloud->points[end_point_index] << std::endl;
	

	//Visualize stuff
	pcl::visualization::PCLVisualizer viewer;
	//Highlight the points
	viewer.addSphere(cloud->points[start_point_index],alpha,1,0,0,"top_pt");
	viewer.addSphere(cloud->points[end_point_index],alpha,1,0,0,"side_pt");

	//Plot more stuff
	// viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,cloud_normals,10,0.05,"cloud_normals");
	// while(1){
	// 	viewer.spin();
	// }
	viewer.addPointCloud<pcl::PointXYZ>(cloud,"cloud");
	viewer.addCoordinateSystem(1,"ref");
	viewer.addArrow(end_point,start_point,0,1,0,"arrow");


	// Basically, we want to "walk" from the start to an end point along a surface.

	// First, we get a vector that "points" to the walking direction;
	Eigen::Vector3f eig_start = cloud->points[start_point_index].getVector3fMap();
	Eigen::Vector3f eig_end = cloud->points[end_point_index].getVector3fMap();

	// Some algebraic st
	Eigen::Vector3f direction;
	Eigen::Vector3f normal;
	Eigen::Vector3f projection;
	float projection_length;


	Eigen::Vector3f current_point = eig_start;
	Eigen::Vector3f search_direction;
	int index = start_point_index;
	int next_index;
	int iteration = 0;

	pcl::IndicesPtr walk_indexes (new pcl::Indices);

	while(1){

	//Direction
	 direction = eig_end - current_point;
	std::cout << "'error': " << direction.norm() << std::endl;
	if(direction.norm() < 0.01) // We're there! May vary..
		break;

	 normal = cloud_normals->points[index].getNormalVector3fMap();

	// Now, we project the direction vector onto de plane normal of the point
	// Ref -> https://www.maplesoft.com/support/help/Maple/view.aspx?path=MathApps/ProjectionOfVectorOntoPlane
 	projection_length = direction.dot(normal);
	projection = direction - projection_length*normal/normal.norm();
	
	std::cout << "projection: " << projection << std::endl;
	
	// Normalize projection, otherwise if the walk gets too close to the target it slows down
	projection.normalize();
	
	// We add the projection to the current point
	search_direction = current_point + alpha*projection;

	// Search for a point belonging to the input cloud that is closest to our iteration walk
	pcl::PointXYZ next_xyz(search_direction[0],search_direction[1],search_direction[2]);
	std::cout << "search direction: " << search_direction << std::endl << std::endl;
	tree->nearestKSearch(next_xyz,1,index_vector,dummy_arg);
	

	index = index_vector[0];
	walk_indexes->push_back(index); //Save our  "walk" indexes
	viewer.addSphere(cloud->points[index],alpha,0,1,0,"walk_pt" + std::to_string(iteration++)); //Plots

	current_point = cloud->points[index].getArray3fMap(); // iterate
	
	
	viewer.spinOnce(80); //Delay for visualization only

	}

	// Now we extract the saved indices from the estimated normals
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr output_normals (new pcl::PointCloud<pcl::Normal>);

	pcl::ExtractIndices<pcl::Normal> normal_filter;
	//Extract normals
	normal_filter.setInputCloud(cloud_normals);
	normal_filter.setIndices(walk_indexes);
	normal_filter.filter(*output_normals);

	pcl::ExtractIndices<pcl::PointXYZ> point_filter;
	//Extract the corresponding point to the normal cloud
	// (Im only doint that because i dont know to to plot normals ONLY haha)
	point_filter.setInputCloud(cloud);
	point_filter.setIndices(walk_indexes);
	point_filter.filter(*output_cloud);
	// filter.filter(*output_cloud);

	std::cout << "Number of walked points/normals: " << walk_indexes->size() << std::endl;
	
	// Plotting both
	viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(output_cloud,output_normals,1,0.1,"cloud_normals");

	pcl::PointCloud<pcl::PointNormal> cloud_path;
	pcl::concatenateFields(*output_cloud,*output_normals,cloud_path);
	pcl::io::savePLYFile("cloud_path.ply",cloud_path);

	while(!viewer.wasStopped()){

		viewer.spin();
	}

	return 0;

	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
}
