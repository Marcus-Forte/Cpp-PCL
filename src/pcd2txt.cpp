#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <fstream>








int main(int argc,char** argv){

	if (argc < 2)
	{
		std::cerr << "Please give an .pcd argument" << std::endl;
		exit(-1);
	}

    std::string cloudfile = argv[1];

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // verificar se é .pcd ou .ply no argv1


    	if (pcl::io::loadPCDFile(cloudfile, *cloud) == -1)
	{
		PCL_ERROR("Nao deu pra abrir arquivo \n");
		return -1;
	}

    std::string outfile;
    int final = cloudfile.find_last_of('.');
    int init = cloudfile.find_last_of('/');
    std::cout << "(" << init << "," << final << ")" << std::endl;
    outfile = cloudfile.substr(init+1,final-init-1);
    outfile += ".txt";
    // std::cout << outfile << std::endl;

    std::ofstream file;

    file.open(outfile);

    for(int i=0;i<cloud->size();++i){
        file << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;
    }

    

    file.close();

    std::cout << "sucesso! \n";


    
}