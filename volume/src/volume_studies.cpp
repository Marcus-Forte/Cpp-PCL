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

// Utilidades (Classe)
#include "PCUtils.h"

// Normals
#include "pcl/features/normal_3d.h"
#include "pcl/features/normal_3d_omp.h"

// Voxel
#include <pcl/filters/voxel_grid.h>

//Mesh
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>

// Transformações
#include <pcl/common/transforms.h>

// VTK
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <vtkPolyData.h>
#include <vtkMassProperties.h>
#include <vtkFillHolesFilter.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataNormals.h>
#include <vtkDelaunay3D.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCleanPolyData.h>
#include <vtkTetra.h>
#include <vtkPoints.h>

/* LINKS

https://lorensen.github.io/VTKExamples/site/Cxx/Modelling/Delaunay3D/

*/

void PrintUsage(const char *progName)
{
	std::cout << "\n\nUsage: " << progName << " cloudfile resolution\n\n";
}

std::atomic<bool> done{false};
// TODO Design lixo, melhorar dps

float getVolumefromMesh(const pcl::PolygonMesh::Ptr &pclMesh)
{
	vtkSmartPointer<vtkPolyData> vtkMesh = vtkSmartPointer<vtkPolyData>::New();

	pcl::VTKUtils::convertToVTK(*pclMesh, vtkMesh); //converte p/ vtk

	// Mais filtros podem ser adicionados aqui

	vtkSmartPointer<vtkFillHolesFilter> fillholes = vtkSmartPointer<vtkFillHolesFilter>::New();
	fillholes->SetInputData(vtkMesh);
	fillholes->SetHoleSize(0.1);
	fillholes->Update();

	// vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
	// triangleFilter->SetInputConnection(fillholes->GetOutputPort());
	// triangleFilter->Update();

	vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New(); //calcula normais
	// normals->SetInputData(vtkMesh);
	normals->SetInputConnection(fillholes->GetOutputPort());
	normals->ConsistencyOn();
	normals->SplittingOff();
	normals->Update();

	vtkSmartPointer<vtkMassProperties> massProperties = vtkSmartPointer<vtkMassProperties>::New(); //calcula volume
	massProperties->SetInputConnection(normals->GetOutputPort());
	massProperties->Update();
	// std::cout << "Vtk volume : " << massProperties->GetVolume() << std::endl;
	return massProperties->GetVolume();
}

float signedVolumeOfTriangle(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3)
{
	float v321 = p3.x * p2.y * p1.z;
	float v231 = p2.x * p3.y * p1.z;
	float v312 = p3.x * p1.y * p2.z;
	float v132 = p1.x * p3.y * p2.z;
	float v213 = p2.x * p1.y * p3.z;
	float v123 = p1.x * p2.y * p3.z;
	return (1.0f / 6.0f) * (-v321 + v231 + v312 - v132 - v213 + v123);
}

float volumeOfMesh(const pcl::PolygonMesh &mesh)
{
	float vols = 0.0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

	for (int triangle = 0; triangle < mesh.polygons.size(); triangle++)
	{
		pcl::PointXYZ pt1 = cloud->points[mesh.polygons[triangle].vertices[0]];
		pcl::PointXYZ pt2 = cloud->points[mesh.polygons[triangle].vertices[1]];
		pcl::PointXYZ pt3 = cloud->points[mesh.polygons[triangle].vertices[2]];
		vols += signedVolumeOfTriangle(pt1, pt2, pt3);
	}
	return abs(vols);
}

void concHULL(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_in, pcl::PolygonMesh::Ptr mesh, float alpha)
{

	std::cout << "starting thread" << std::endl;
	pcl::ConcaveHull<pcl::PointXYZ> hull;
	hull.setInputCloud(cloud_in);
	hull.setAlpha(alpha);
	hull.reconstruct(*mesh);
	std::cout << "finished thread" << std::endl;
	std::cout << "Alpha : " << alpha << "| Mesh size: " << mesh->polygons.size() << std::endl;
	std::cout << "Volume : " << getVolumefromMesh(mesh) << std::endl;
	done = true;
}

int main(int argc, char **argv)
{

	if (argc < 3)
	{
		PrintUsage(argv[0]);
		return 0;
	}

	std::cout.precision(10);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr processing_cloud(new pcl::PointCloud<pcl::PointXYZ>); // Cloud to be processed


	PCUtils::readFile(argv[1],*cloud);
	//Convex Hull Volume


	return 0;

	pcl::PointXYZ centroid;
	pcl::computeCentroid(*cloud, centroid);

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	transform(0, 3) = -centroid.x;
	transform(1, 3) = -centroid.y;
	transform(2, 3) = -centroid.z;

	std::cout << centroid << std::endl;

	pcl::transformPointCloud(*cloud, *cloud, transform);

	/* --------------   VOXEL GRID ------------------ */
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	float res = 0.01; //Tunnable
	sor.setLeafSize(res, res, res);
	sor.filter(*cloud_filtered);

	pcl::PointCloud<pcl::PointXYZ>::Ptr floor(new pcl::PointCloud<pcl::PointXYZ>);

	/* --------------   FLOOR MAKER ------------------ */
	PCUtils::makeFloor(cloud_filtered, floor, 1);

	// MERGE FLOOR AND ORIGINAL CLOUD
	pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
	*merged = *cloud_filtered + *floor;

	/* --------------   NORMAL ESTIMATION ------------------ */
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne_omp(6);
	// ne_omp.setNumberOfThreads(8);
	ne_omp.setInputCloud(merged);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
	ne_omp.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	ne_omp.setRadiusSearch(0.1); // Tunnable

	PCL_INFO("Estimating normals...\n");
	ne_omp.compute(*cloud_normals);
	PCL_INFO("Normals -> %d\n", cloud_normals->size());
	PCL_INFO("DONE !\n");

	/* MESH */
	pcl::PolygonMesh triangles;

	/* --------------   GREEDY ------------------ */
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*merged, *cloud_normals, *cloud_with_normals);

	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

	gp3.setSearchRadius(10);
	gp3.setMu(10);
	gp3.setMaximumNearestNeighbors(50);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18);		  // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3);	  // 120 degrees
	gp3.setNormalConsistency(false);

	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	PCL_INFO("Reconstructing mesh...\n");
	gp3.reconstruct(triangles);
	PCL_INFO("DONE !\n");

	/* --------------   POISSON ------------------ */
	// pcl::Poisson<pcl::PointNormal> poisson;

	// poisson.setDepth(0.01);
	// poisson.setInputCloud(cloud_with_normals);
	// poisson.reconstruct(triangles);

	/* --------------   VISUALIZATION  ------------------ */
	int vp0, vp1;
	pcl::visualization::PCLVisualizer::Ptr viewer_ = pcl::make_shared<pcl::visualization::PCLVisualizer>("quick viewer");
	//viewports

	viewer_->createViewPort(0, 0, 0.5, 1, vp0);
	viewer_->createViewPort(0.5, 0, 1, 1, vp1);
	// Normals
	pcl::visualization::PointCloudColorHandlerCustom<pcl::Normal> normal_color(cloud_normals, 255, 0, 0); // N SEI USAR
	viewer_->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(merged, cloud_normals, 10, 0.1, "cloud_normals", vp0);
	// Original + Floor
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(merged, 0, 255, 0);
	viewer_->addPointCloud(merged, single_color, "merged cloud", vp0);

	//Point Size
	viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "merged cloud");
	viewer_->addCoordinateSystem(1, "ref");
	viewer_->addPolygonMesh(triangles, "polygon", vp1);

	float v = volumeOfMesh(triangles);
	std::cout << "volume = " << v << std::endl;
	while (!viewer_->wasStopped())
	{
		viewer_->spinOnce();
	}

	return 0;

	// PCUtils::quickView(cloud_filtered);

	// PCUtils::quickView(floor);

	// pcl::PointCloud<pcl::PointXYZ>::Ptr filled (new pcl::PointCloud<pcl::PointXYZ>);

	// PCUtils::fillCloud(merged,filled,20);

	// std::cout << "filled cloud size : " << filled->size() << std::endl;

	// PCUtils::quickView(filled);

	// return 0;

	// return 0;

	// VISUALIZER

	pcl::visualization::PCLVisualizer::Ptr viewer = pcl::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
	viewer->setCameraPosition(-6.61, -4.33, 2.7, 1.0, 1, 1);
	viewer->setSize(1920, 1080);
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

	int v1, v2, v3;

	viewer->createViewPort(0, 0, 0.3, 1, v1);
	viewer->createViewPort(0.3, 0, 0.6, 1, v2);
	viewer->createViewPort(0.6, 0, 1, 1, v3);

	// viewer.addPolygonMesh(*convex_mesh,"mesh",0);

	float alpha = 0.1;
	viewer->setBackgroundColor(0, 0, 0, 0);

	viewer->addCoordinateSystem(1, "ref");
	viewer->addPointCloud(cloud, "cloud in", v1); //original
	viewer->addPointCloud(processing_cloud, "cloud filtered", v2);
	// viewer.addPolygonMesh(*convex_mesh,"conv mesh",v2);

	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

	std::thread t(concHULL, processing_cloud, mesh, alpha); //usamos thread p/ poder mover a camera enquanto ele calcula
	t.join();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100, true);

		if (done)
		{
			viewer->removePolygonMesh("mesh");
			viewer->addPolygonMesh(*mesh, "mesh", v3);
			alpha += 0.01;
			std::thread t(concHULL, processing_cloud, mesh, alpha);
			t.detach();
			done = false;
		}
	}
	return 0;
}
