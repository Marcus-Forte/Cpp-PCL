#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <fstream>


// getopt
#include <unistd.h>



void printUsage(){
        std::cout << "Usage: extconverter [.pcd / .txt] [-D dir]" << std::endl;
}

template <typename pointT>
static inline int readTxt(const std::string &filename, pcl::PointCloud<pointT> &cloud)
{
    std::ifstream file;
    file.open(filename);

    std::string line;
    // std::cout << "read file" << std::endl;
    pointT pt;
    while (std::getline(file, line))
    {

        // std::cout << "LINE : " << line << std::endl;
        if (line.find("#") != std::string::npos)
        {
            // std::cout << "DEBUG : COMMENTARIO" << std::endl;
            continue;
        }
        else if (line.size() > 1) //non empty
        {
            sscanf(line.c_str(), "%f %f %f", &pt.x, &pt.y, &pt.z);
            // std::cout << line.size() << std::endl;
            cloud.push_back(pt);
        }
    }
    return 0;
}

int main(int argc, char **argv)
{

    if (argc < 2)
    {
		printUsage();
        exit(-1);
    }

	std::string custom_dir = "./";
	char opt;
	while ((opt = getopt(argc, argv, "D:h")) != -1){

			switch(opt){
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

			//optind start from non option arguments
    

    std::string cloudfile = argv[optind];

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    	int index = cloudfile.find_last_of('.');
		std::string extension = cloudfile.substr(index + 1);
        std::string output_extension;
        bool isOutputTxt = false;

		if (extension.compare("pcd") == 0)
		{
			std::cout << "Loading Input Clouds..." << std::endl;
			if (pcl::io::loadPCDFile(cloudfile, *cloud) == -1)
			{
				PCL_ERROR("Cloud not open .pcd file.\n");
				exit(-1);
			}
            output_extension = ".txt";
            isOutputTxt = true;
		}
		else if (extension.compare("txt") == 0)
		{

			std::cout << "Loading Input Clouds..." << std::endl;
			if (readTxt(cloudfile, *cloud) == -1)
			{
				PCL_ERROR("Cloud not open .txt file.\n");
				exit(-1);
			}
            output_extension = ".pcd";
            
		}
		else {
			
				PCL_ERROR("file format not supported!\n");
				exit(-1);
		}

    

    // Find out the extension

    std::string outfile;
    int final = cloudfile.find_last_of('.');
    int init = cloudfile.find_last_of('/');
    // std::cout << "(" << init << "," << final << ")" << std::endl;
    outfile = cloudfile.substr(init + 1, final - init - 1);
    outfile += output_extension;
    // std::cout << outfile << std::endl;

    
	std::cout << "Converting..." << std::endl;
	// std::cout << "Custom dir = '" << custom_dir << "'" << std::endl;
    if(isOutputTxt){
    std::ofstream file;

    file.open(custom_dir + outfile);

    for (int i = 0; i < cloud->size(); ++i)
    {
        file << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;
    }

    file.close();
    std::cout << ".txt saved!" << std::endl;

    } else {

        pcl::io::savePCDFileASCII(custom_dir + outfile,*cloud);
        std::cout << ".pcd saved!" << std::endl;
        return 0;

    }

    
}
