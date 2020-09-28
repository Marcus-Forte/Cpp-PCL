#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>



typedef pcl::PointCloud<pcl::PointXYZ> CloudType;


int main(int argc,char** argv){

		CloudType::Ptr cloud (new CloudType);

		if(argc < 3){
				std::cerr << "Usage : subsample cloud.pcd [resolution]" << std::endl;
				exit(-1);
		}

		std::cout << "Loading Input point cloud..." << std::endl;
		if ( pcl::io::loadPCDFile(argv[1],*cloud) != 0 )
				exit(-1);

		// cloudname 
		std::string arg(argv[1]);
		std::string voxel_cloudname;
		int slash =   arg.find_last_of('/'); //path
		int dot = arg.find_last_of('.'); //extension
		voxel_cloudname = arg.substr(slash+1,dot - slash - 1) + "_" + "voxeled.pcd";



		CloudType::Ptr cloud_filtered (new CloudType);
		

		float res = atof(argv[2]);
		std::cout << "res = " << res << std::endl;

		// 
		// 
		pcl::VoxelGrid<pcl::PointXYZ> voxel;
		voxel.setInputCloud(cloud);
		voxel.setLeafSize(res,res,res);
		voxel.filter(*cloud_filtered);
		// 
		//  
		PCL_INFO("saving to '%s'  \n",voxel_cloudname.c_str());
		pcl::io::savePCDFileBinary( voxel_cloudname, *cloud_filtered);
		PCL_INFO("subsampled cloud '%s' saved successfully \n",voxel_cloudname.c_str());

	
		return 0;



}
