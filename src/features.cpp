#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

void PrintUsage(){
    std::cout << "Usage : features [cloud.pcd]" << std::endl;
}


int main(int argc,char** argv){

    
    if(argc < 2){
        PrintUsage();
        exit(-1);
    }

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::io::loadPCDFile(argv[1],*input_cloud);

    PCL_INFO("Number of points -> %d \n",input_cloud->size());


    pcl::visualization::PCLVisualizer viewer;

    viewer.addPointCloud(input_cloud);
    viewer.addCoordinateSystem(1,"ref");

    while(!viewer.wasStopped()){
        viewer.spin();
    }








    return 0;
}