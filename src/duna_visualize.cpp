#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_picking_event.h>
#include <thread>
#include <mutex>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>

#include "PCUtils.h"
#include <vector>

using PointCloudT = pcl::PointCloud<pcl::PointXYZRGB>;
std::vector<PointCloudT::Ptr> cloud_vector;

bool animate;
pcl::visualization::Camera c0, c1;

static bool compare_z(pcl::PointXYZRGB &a, pcl::PointXYZRGB &b)
{
	return (a.z < b.z);
}

static bool compare_x(pcl::PointXYZRGB &a, pcl::PointXYZRGB &b)
{
	return (a.x < b.x);
}

static bool compare_y(pcl::PointXYZRGB &a, pcl::PointXYZRGB &b)
{
	return (a.y < b.y);
}

// color by height
// [0 102 153]
// [102 204 153]

static inline float lerp(float v0, float v1, float t)
{
	return (1 - t) * v0 + t * v1;
}

void DunaPaint(const PointCloudT &cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud)
{

	*colored_cloud = cloud; // copy

	//off white
	int white_r = 245;
	int white_g = 245;
	int white_b = 245;

	//green
	int aqua_r = 102; //AQU
	int aqua_g = 204;
	int aqua_b = 153;

	//blue
	int saph_r = 0; //SAPHIRE
	int saph_g = 102;
	int saph_b = 153;

	int hon_r = 255; // RUIM
	int hon_g = 180;
	int hon_b = 0;

	int or_r = 246; // RUIM DMS
	int or_g = 81;
	int or_b = 29;

	size_t cloud_size = colored_cloud->size();

	// std::sort(colored_cloud->begin(), colored_cloud->end(), compare_z); // no need
	pcl::PointXYZRGB min, max;
	pcl::getMinMax3D(*colored_cloud, min, max);

	float z_part = (max.z - min.z) / 2;

	// std::cout << "mid: " << mid_z << std::endl;
	for (auto &it : colored_cloud->points)
	{

		if (it.z < min.z + z_part)
		{ // white
			it.r = lerp(white_r, aqua_r, (it.z - min.z) / z_part);
			it.g = lerp(white_g, aqua_g, (it.z - min.z) / z_part);
			it.b = lerp(white_b, aqua_b, (it.z - min.z) / z_part);
		}
		else if (it.z < min.z + 2 * z_part)
		{ // aqua
			it.r = lerp(aqua_r, saph_r, (it.z - min.z - z_part) / z_part);
			it.g = lerp(aqua_g, saph_g, (it.z - min.z - z_part) / z_part);
			it.b = lerp(aqua_b, saph_b, (it.z - min.z - z_part) / z_part);
		}
		// else if (it.z < min.z + 3 * z_part)
		// { //sapphire
		// 	it.r = lerp(saph_r, hon_r, (it.z - min.z - 2*z_part) / z_part);
		// 	it.g = lerp(saph_g, hon_g, (it.z - min.z - 2*z_part) / z_part);
		// 	it.b = lerp(saph_b, hon_b, (it.z - min.z - 2*z_part) / z_part);
		// }
		// else if (it.z < min.z + 4 * z_part)
		// { // honey
		// 	it.r = lerp(hon_r, or_r, (it.z - min.z - 3*z_part) / z_part);
		// 	it.g = lerp(hon_g, or_g, (it.z - min.z - 3*z_part) / z_part);
		// 	it.b = lerp(hon_b, or_b, (it.z - min.z - 3*z_part) / z_part);
		// }
	}
}

// TODO add multiple input clouds

std::mutex g_mutex;

// thread here
// TODO por alguma razão nuvens RGB (kinect) dão problema
// TODO Implementar Pause, resume ..
// TODO cores !
// TODO velocidade flexivel
void areaPickCallback(const pcl::visualization::AreaPickingEvent &event, void *cookie)
{
	pcl::IndicesPtr selected_pts(new pcl::Indices);
	event.getPointsIndices(*selected_pts);
	pcl::ExtractIndices<pcl::PointXYZRGB> extractor;
	extractor.setIndices(selected_pts);
	extractor.setInputCloud(cloud_vector[0]);
	extractor.setNegative(true);
	extractor.filter(*cloud_vector[0]);

	// Repaint
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr duna_color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	DunaPaint(*cloud_vector[0], duna_color_cloud);

	pcl::visualization::PCLVisualizer *viewer_ptr = (pcl::visualization::PCLVisualizer *)cookie;

	viewer_ptr->updatePointCloud(duna_color_cloud, "cloud0");
}

void keyCallback(const pcl::visualization::KeyboardEvent &event, void *cookie)
{
	// std::cout << "key pressed " << std::endl;
	if (event.keyDown() == false)
		return;

	pcl::visualization::PCLVisualizer *viewer_ptr = (pcl::visualization::PCLVisualizer *)cookie;
	char key = event.getKeyCode();
	static bool c0_set = false;
	static bool c1_set = false;

	if (key == 'b')
	{
		viewer_ptr->getCameraParameters(c0);
		std::cout << "Camera 0 Set!" << std::endl;
		c0_set = true;
	}
	else if (key == 'n')
	{
		viewer_ptr->getCameraParameters(c1);
		std::cout << "Camera 1 Set!" << std::endl;
		c1_set = true;
	}
	else if (key == 'm')
	{

		if ((c0_set && c1_set) == false)
		{
			std::cout << "No camera path" << std::endl;
			return;
		}
		else
		{
			animate = true; //triggers animation
		}
	}
}

void make_grid(pcl::visualization::PCLVisualizer &viewer, float res = 1)
{
	float x_min = -10;
	float x_max = 10;
	float y_min = -10;
	float y_max = 10;
	int N_lines = 10;

	pcl::PointXYZ p1, p2, p3, p4;
	// p1.x = 0;
	// p2.x = 0;
	// p3.x = 0;
	// p4.x = 0;
	// p1.y = 0;
	// p2.y = 0;
	// p3.y = 0;
	// p4.y = 0;
	// p1.z = 0;
	// p2.z = 0;
	// p3.z = 0;
	// p4.z = 0;

	p1.x = x_min;
	p2.x = x_max;
	p1.y = y_min;
	p2.y = y_min;

	p3.x = x_min;
	p4.x = x_min;
	p3.y = y_min;
	p4.y = y_max;

	float y = y_min;
	float x = x_min;
	int i = 0;
	while (y < y_max && x < x_max)
	{
		p1.y = y;
		p2.y = y;
		p3.x = x;
		p4.x = x;
		viewer.addLine(p1, p2, "line" + std::to_string(i++));
		viewer.addLine(p3, p4, "line" + std::to_string(i++));

		y += res;
		x += res;
	}
	p1.y = y;
	p2.y = y;
	p3.x = x;
	p4.x = x;

	viewer.addLine(p1, p2, "line" + std::to_string(i++));
	viewer.addLine(p3, p4, "line" + std::to_string(i++));
}

void pp_callback(const pcl::visualization::PointPickingEvent &event)
{

	if (event.getPointIndex() != -1)
	{

		float x, y, z;
		event.getPoint(x, y, z);
		int i = event.getPointIndex();
		std::cout << "i:" << i << " x = " << x << ";" << y << ";" << z << std::endl;
	}
}

typedef pcl::PointCloud<pcl::PointXYZ> CloudType;

void PrintUsage()
{
	std::cerr << "Usage : vizualise -r [voxel rex] -f cloud.pcd/.txt ..." << std::endl;
	
}

int main(int argc, char **argv)
{

	if (argc < 2)
	{
		PrintUsage();
		exit(-1);
	}
	int n_clouds = 1;
	int index;
	index = pcl::console::find_argument(argc, argv, "-f");
	if(index == -1){
		PrintUsage();
		exit(-1);
	}

	if (argv[index+1] == nullptr){
		PrintUsage();
		exit(-1);
	}
	

	float voxel_rex = 0.01;
	if (pcl::console::find_switch(argc, argv, "-r") == true)
	{
		pcl::console::parse_argument(argc, argv, "-r", voxel_rex);
	}


	pcl::visualization::PCLVisualizer viewer("My Viewer");
	int v1, v2;
	viewer.createViewPort(0, 0, 1, 1, v1);
	// viewer.setCameraPosition(-6.61, -2.13, 10.33, 1.0, 1, 1);
	// viewer.setBackgroundColor(0, 0, 0, v1);
	// viewer.addCoordinateSystem(1, "ref", v1);
	viewer.registerPointPickingCallback(pp_callback);
	viewer.registerKeyboardCallback(keyCallback, &viewer);
	viewer.registerAreaPickingCallback(areaPickCallback, &viewer);
	// make_grid(viewer, 1);

	

	std::cout << "N clouds = " << n_clouds << std::endl;

	cloud_vector.resize(n_clouds);

	for (int i = 0; i < n_clouds; i++)
	{
		std::cout << "Reading file .. " << i << std::endl;
		cloud_vector[i] = pcl::make_shared<PointCloudT>();

		PCUtils::readFile(argv[index + 1], *cloud_vector[i]); // no check done

		PCL_INFO("Applying Voxel. Res = %f\n",voxel_rex);
		pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
		voxel.setInputCloud(cloud_vector[i]);
		voxel.setLeafSize(voxel_rex, voxel_rex, voxel_rex);
		voxel.filter(*cloud_vector[i]);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr duna_color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		DunaPaint(*cloud_vector[i], duna_color_cloud);

		std::string cloudname = "cloud" + std::to_string(i);

		viewer.addPointCloud(duna_color_cloud, cloudname);
	}

	std::cout << "Done" << std::endl;

	while (!viewer.wasStopped())
	{

		if (animate)
		{

			pcl::visualization::Camera ci = c0;

			const int alpha = 1000;

			float increment_x = (c1.pos[0] - c0.pos[0]) / (float)alpha;
			float increment_y = (c1.pos[1] - c0.pos[1]) / (float)alpha;
			float increment_z = (c1.pos[2] - c0.pos[2]) / (float)alpha;

			for (int i = 0; i < alpha; ++i)
			{

				ci.pos[0] += increment_x;
				ci.pos[1] += increment_y;
				ci.pos[2] += increment_z;

				viewer.setCameraParameters(ci);
				viewer.spinOnce();
				usleep(1000);
			}

			for (int i = 0; i < alpha; ++i)
			{

				ci.pos[0] -= increment_x;
				ci.pos[1] -= increment_y;
				ci.pos[2] -= increment_z;

				viewer.setCameraParameters(ci);
				viewer.spinOnce();
				usleep(1000);
			}

			animate = false;
		}

		viewer.spinOnce();
		usleep(1000);
	}

	return 0;
}
