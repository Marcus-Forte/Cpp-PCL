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

std::atomic<bool> done{false};


float getVolumefromMesh(const pcl::PolygonMesh::Ptr& pclMesh){
	vtkSmartPointer<vtkPolyData> vtkMesh = vtkSmartPointer<vtkPolyData>::New();

		pcl::VTKUtils::convertToVTK(*pclMesh,vtkMesh); //converte p/ vtk

		// Mais filtros podem ser adicionados aqui

				vtkSmartPointer<vtkFillHolesFilter> fillholes = vtkSmartPointer<vtkFillHolesFilter>::New();
		fillholes->SetInputData(vtkMesh);
		fillholes->SetHoleSize(10);
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
		return massProperties->GetVolume() ;


}



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
		std::cout << "Volume : " << getVolumefromMesh(mesh) << std::endl;
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

		
			
		


		pcl::PolygonMesh::Ptr convex_mesh (new pcl::PolygonMesh);
		pcl::ConvexHull<pcl::PointXYZ> convex;
		convex.setComputeAreaVolume(true);
		convex.setInputCloud(cloud);
		convex.reconstruct(*convex_mesh);
		std::cout << "Convex Hull Volume " << convex.getTotalVolume() << std::endl;
		std::cout << "Concave - Number of meshes : " <<convex_mesh->polygons.size() << std::endl;
		std::cout << "My volume = " << volumeOfMesh(convex_mesh) << std::endl;
		pcl::io::savePLYFile("convex.ply",*convex_mesh);
		std::cout << "Mesh saved" << std::endl;	

				//		pcl::PolygonMesh concave_mesh;
		// pcl::PolygonMesh::Ptr concave_mesh (new pcl::PolygonMesh);
		// pcl::ConcaveHull<pcl::PointXYZ> chull;
		// chull.setInputCloud(cloud);
		// chull.setAlpha(0.1);
		// chull.reconstruct(*concave_mesh);
		  

		vtkSmartPointer<vtkPolyData> vtkMesh = vtkSmartPointer<vtkPolyData>::New();
		pcl::VTKUtils::convertToVTK(*convex_mesh,vtkMesh);
		//TODO como pegar a saida do delaunay3D do vtk p/ plotar mesh
	
		vtkSmartPointer<vtkCleanPolyData> vtkCleaner = vtkSmartPointer<vtkCleanPolyData>::New();
		vtkCleaner->SetInputData(vtkMesh);
		vtkCleaner->Update();

		vtkSmartPointer<vtkDelaunay3D> delaunay = vtkSmartPointer<vtkDelaunay3D>::New();
		delaunay->SetInputData(vtkMesh);
		delaunay->Update();

		
		vtkSmartPointer<vtkUnstructuredGrid> d_output = vtkSmartPointer<vtkUnstructuredGrid>::New();
		d_output = delaunay->GetOutput();
		int n_cells = d_output->GetNumberOfCells();
			
		vtkSmartPointer<vtkPoints> pontos = vtkSmartPointer<vtkPoints>::New();
		
		vtkIdList * IDs;
		d_output->GetCellPoints(0,IDs);
	//	std::cout << *IDs << std::endl;



		return 0;

		double *p0;
		double *p1;
		double *p2;
		double *p3;

		for (int j = 0; j < n_cells; ++j)
		{
			pontos = d_output->GetCell(j)->GetPoints();
			for (int i=0;i<4;++i){
			
			}
		}

		// vtkTetra::ComputeVolume
		return 0;


		pcl::visualization::PCLVisualizer viewer("3D Viewer");
	
		float alpha = 0.001;
		viewer.setBackgroundColor(0,0,0);
		// viewer.addPointCloud(cloud,"nuvem");
		pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);

		std::thread t (concHULL,cloud,mesh,alpha); //usamos thread p/ poder mover a camera enquanto ele calcula
		t.join();
		while(!viewer.wasStopped()){
				viewer.spinOnce(100);

				if(done){
						viewer.removePolygonMesh("mesh");
						viewer.addPolygonMesh(*mesh,"mesh");
						alpha += 80;
						std::thread t(concHULL,cloud,mesh,alpha);
						t.detach();
						done = false;
				}


		}
		return 0;
}
