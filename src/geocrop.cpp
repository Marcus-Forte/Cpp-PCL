#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/surface/convex_hull.h>

#include "PCUtils.h"

#include "crop2pcd.h"

#include <boost/property_tree/json_parser.hpp> // .json parse
#include <boost/property_tree/exceptions.hpp>

#include <unistd.h>

void printUsage(){
std::cout << "Usage:" << std::endl << " geocrop cloud.pcd crop1.crop crop2.crop ...  [origin.json] [-D dir] -[v]" << std::endl << std::endl;
std::cout << "Crop point clouds clouds with geographical coordinates" << std::endl << std::endl;
std::cout << "First parameter must be the '.pcd' cloud to be cropped." << std::endl;
std::cout << "'.json' files are optional, but are required if no origin is given in the .crop files. They must have a node called 'origem' with at latitude and longitude specified" << std::endl;
std::cout << "Options:" << std::endl;
std::cout << "-D	specify cropped cloud save directory " << std::endl;
std::cout << "-v	specify cropped cloud save directory " << std::endl;
std::cout << "-v 	optional visualization interface" << std::endl;

}

//TODO
// gerar arquivos de saída com nomes padronizados (parsear timestamp do arquivo origem e juntar nomes nas saídas)

static void extrude(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, const pcl::PointCloud<pcl::PointXYZ> &crop_zone, pcl::PointCloud<pcl::PointXYZ> &crop_zone3D)
{
	pcl::PointXYZ min, max;
	pcl::getMinMax3D(cloud_in, min, max);
	crop_zone3D = crop_zone; // Copy ?

	for (int i = 0; i < crop_zone.size(); ++i)
	{
		pcl::PointXYZ newpt(crop_zone.points[i].x, crop_zone.points[i].y, max.z + 10);
		crop_zone3D.points[i].z = min.z - 10;
		crop_zone3D.push_back(newpt);
	}
}

static void crop(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_in, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &crop_zone, pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr crop_zone3D(new pcl::PointCloud<pcl::PointXYZ>);

	extrude(*cloud_in, *crop_zone, *crop_zone3D);

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
}

int main(int argc, char **argv)
{
	if (argc < 3)
	{
		printUsage();
		exit(-1);
	}

	char opt;
	bool hasVisualization = false;
	std::string custom_dir = "./";

	while ((opt = getopt(argc, argv, "vD:h")) != -1)
	{

			switch(opt){
					case 'v':
							hasVisualization = true;
							break;
					case 'D':
							custom_dir = std::string(optarg) + "/";
//							std::cout << "custom dir : " << custom_dir << std::endl;
							break;
					case '?':
							std::cerr << "Specify the requried directory!" << std::endl;
							exit(-1);
							break;

					case 'h':
					printUsage();
					exit(0);
					break;


			}


	}



	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);

	bool hasJsonOrigin = false;
	bool hasPCD = false;
	std::string raw_prefix;

	for(int i = optind;i<argc;++i){
		std::string filename = argv[i];
		if (filename.find(".pcd") != std::string::npos) {// Check if .crop file
		hasPCD = true;
	std::cout << "Loading RAW point cloud..." << std::endl;
	if (pcl::io::loadPCDFile(argv[optind], *cloud_raw) == -1)
	{
		std::cerr << "Error loading raw cloud.Exiting." << std::endl;
		exit(-1);
	}
	std::cout << "RAW point cloud loaded!" << std::endl;
	std::string rawfilename = argv[optind];
	size_t final_raw = rawfilename.find_last_of("."); //Substring extraction
	size_t init_raw = rawfilename.find_last_of("/");
	raw_prefix = rawfilename.substr(init_raw + 1, final_raw - init_raw - 1);
	break; // Looks only for a single .pcd
		}
	}

	if(!hasPCD){
		std::cerr << "No .pcd file given! Exiting.";
		exit(-1);
	}


	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> crop_clouds;

	int cloud_i = 0;
	NavSatFix Origin = {0}; // Set to zero to indicate abscence of data

	boost::property_tree::ptree pt;
	// look for json origin
	for (int i = optind; i < argc; ++i)
	{
		std::string filename = argv[i];
		if (filename.find(".json") != std::string::npos)
		{
			// parse .json, check origin
			hasJsonOrigin = true;
			std::cout << "Loading .json metadata: " << filename << std::endl;
			try{
			boost::property_tree::read_json(filename, pt); // May throw
			} catch(boost::property_tree::ptree_error e) {
				std::cerr << "error loading .json: " << e.what() << std::endl;
				hasJsonOrigin = false;
			}

			try{
			std::string value;
			value = pt.get<std::string>("origem"); // may throw. 
			sscanf(value.c_str(), "%lf,%lf,%*lf", &Origin.latitude, &Origin.longitude);
			}
			 catch (boost::property_tree::ptree_error e) {
				std::cerr << "Warning: " << e.what() << std::endl;
				hasJsonOrigin = false;	
			}
			// std::cout << "Json Origin: " << value << std::endl;
			

			
		}
	}

	std::vector<std::string> outputfilenames;

	for (int i = optind; i < argc; ++i)
	{
		std::string filename = argv[i];
		if (filename.find(".crop") != std::string::npos) // Check if .crop file
		{
			// parse .crops, check origin
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			int r = readCropfile(filename, cloud, Origin);
			if (r == 1 && hasJsonOrigin == false)
			{ // Some .crop has no origin
				std::cerr << "Warning : '" << filename << "' has NO Origin and no '.json' origin was found. Ignoring crop." << std::endl;
				// exit(-1);
				continue;
			}

			if ( r == -1){
				exit(-1);
			}

			// Generate the name of output file
			size_t final = filename.find_last_of(".");
			size_t init = filename.find_last_of("/");
			std::string newfilename;
			newfilename = filename.substr(init + 1, final - init - 1);

			std::cout << "Cropping '" << newfilename << "' ..." << std::endl;
			crop(cloud_raw, cloud, cropped_cloud); // Crop happens here
			crop_clouds.push_back(cropped_cloud);
			if(cropped_cloud->size() != 0){
			pcl::io::savePCDFileBinary(custom_dir + raw_prefix + "_" + newfilename + ".pcd", *cropped_cloud); // TODO adotar padrão aqui
			std::cout << "Saved: " << newfilename << ".pcd" << std::endl;
			} else {
				std::cout << "Warning: Cropped zone '" << newfilename << "' has no points!" << std::endl;
			}

		}
	}




	if (!hasVisualization)
		return 0;

	std::cout << "Opening crop viewer..." << std::endl;

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
	viewer->addPointCloud(cloud_raw, "raw");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "raw");

	int count = 0;

	for (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = crop_clouds.begin(); it != crop_clouds.end(); ++it)
	{
		std::string crop_name = "crop" + std::to_string(count++);
		pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> color(*it);
		viewer->addPointCloud(*it, color, crop_name);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, crop_name);
	}

	while (!viewer->wasStopped())
	{
		viewer->spin();
	}

	return 0;
}
