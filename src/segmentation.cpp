#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/segmentation/extract_clusters.h>


int main(int argc,char** argv){

if(argc < 2){
    PCL_ERROR("Not enough arguments!");
    exit(-1);
}

std::string cloud_file = argv[1];

pcl::i

}