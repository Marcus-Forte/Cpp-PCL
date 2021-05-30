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

#include <pcl/filters/voxel_grid.h>

#include "PCUtils.h"
#include <vector>

using PointCloudT = pcl::PCLPointCloud2;

using ColorHandler = pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>;
using ColorHandlerPtr = ColorHandler::Ptr;
using ColorHandlerConstPtr = ColorHandler::ConstPtr;

using GeometryHandler = pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2>;
using GeometryHandlerPtr = GeometryHandler::Ptr;
using GeometryHandlerConstPtr = GeometryHandler::ConstPtr;

void keyCallback(const pcl::visualization::KeyboardEvent &event, void *cookie)
{
    pcl::visualization::PCLVisualizer *viewer = (pcl::visualization::PCLVisualizer *)cookie;
    pcl::visualization::Camera c0;
    viewer->getCameraParameters(c0);


    // orientation
    float dx = c0.focal[0] - c0.pos[0];
    float dy = c0.focal[1] - c0.pos[1];
    float dz = c0.focal[2] - c0.pos[2];

    float norm = sqrtf(dx * dx + dy * dy + dz * dz);
    dx /= norm;
    dy /= norm;
    dz /= norm;
    // std::cout << dx << "," << dy << "," << dz << std::endl;
    float vel = 1;

    std::string keyName = event.getKeySym();

    if (keyName == "Up" && event.keyDown())
    {
        c0.pos[0] += dx * vel;
        c0.pos[1] += dy * vel;
        c0.pos[2] += dz * vel;

        c0.focal[0] = c0.pos[0] + dx*2;
        c0.focal[1] = c0.pos[1] + dy*2;
        c0.focal[2] = c0.pos[2] + dz*2;

        viewer->setCameraParameters(c0);
    }
    else if (keyName == "Down" && event.keyDown())
    {
        c0.pos[0] -= dx * vel;
        c0.pos[1] -= dy * vel;
        c0.pos[2] -= dz * vel;

        c0.focal[0] = c0.pos[0] + dx*2;
        c0.focal[1] = c0.pos[1] + dy*2;
        c0.focal[2] = c0.pos[2] + dz*2;
        viewer->setCameraParameters(c0);
    }
    else if (keyName == "Left" && event.keyDown())
    {
    }
    else if (keyName == "Right" && event.keyDown())
    {
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

int main(int argc, char **argv)
{

	if (argc < 2)
	{
		std::cerr << "Usage : vizualise cloud.pcd/.txt ..." << std::endl;
		exit(-1);
	}

	pcl::visualization::PCLVisualizer viewer("My Viewer");
	int v1, v2;
	viewer.createViewPort(0, 0, 1, 1, v1);
	// viewer.setCameraPosition(-6.61, -2.13, 10.33, 1.0, 1, 1);
	// viewer.setBackgroundColor(0, 0, 0, v1);
	// viewer.addCoordinateSystem(1, "ref", v1);
	viewer.registerPointPickingCallback(pp_callback);
	viewer.registerKeyboardCallback(keyCallback,&viewer);
	make_grid(viewer, 1);

	int n_clouds = argc - 1;

	std::cout << "N clouds = " << n_clouds << std::endl;

	std::vector<PointCloudT::Ptr> cloud_vector(n_clouds);

	for (int i = 0; i < n_clouds; i++)
	{
		std::cout << "Reading file .. " << i << std::endl;
		cloud_vector[i] = pcl::make_shared<PointCloudT>();

		PCUtils::readFile(argv[i+1], *cloud_vector[i]);
		// pcl::io::loadPCDFile(argv[i + 1], *cloud_vector[i]);
		
		ColorHandlerPtr color;

		Eigen::Vector4f origin = Eigen::Vector4f::Zero();
		Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();

		for (std::size_t j = 0; j < cloud_vector[i]->fields.size(); ++j)
		{
			std::string field_name = cloud_vector[i]->fields[j].name;
			std::cout << "Color field: " << field_name << std::endl;
			if(field_name == "rgba"){
			color.reset(new pcl::visualization::PointCloudColorHandlerRGBAField<PointCloudT>(cloud_vector[i]));
			} else {
			color.reset(new pcl::visualization::PointCloudColorHandlerGenericField<PointCloudT>(cloud_vector[i], cloud_vector[i]->fields[j].name));
			}
			
			viewer.addPointCloud(cloud_vector[i], color, origin, orientation, "cloud" + std::to_string(i));
		}
	}

	std::cout << "Done" << std::endl;

	while (!viewer.wasStopped())
	{
		viewer.spin();
	}

	return 0;
}
