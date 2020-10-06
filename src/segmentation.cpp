#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/segmentation/extract_clusters.h>

using PointT = pcl::PointXYZ;

int main(int argc,char** argv){

if(argc < 2){
    PCL_ERROR("Not enough arguments!\n");
    exit(-1);
}

std::string cloud_file = argv[1];
pcl::PointCloud<PointT>::Ptr cloud = pcl::make_shared<pcl::PointCloud<PointT>>();


pcl::io::loadPLYFile(cloud_file,*cloud);

PCL_INFO("Cloud loaded! Npoints -> %d\n",cloud->size());



pcl::search::KdTree<PointT>::Ptr tree = pcl::make_shared<pcl::search::KdTree<PointT>>();
tree->setInputCloud(cloud);

std::vector<pcl::PointIndices> cluster_indices;
PCL_WARN("Extracting clusters...\n");
pcl::EuclideanClusterExtraction<PointT> ec;
ec.setClusterTolerance(1);
ec.setMinClusterSize(100);
// ec.setMaxClusterSize(30000);
ec.setSearchMethod(tree);
ec.extract(cluster_indices);
PCL_WARN("Clusters exctracted. Clusters -> %d\n",cluster_indices.size());


return 0;

}