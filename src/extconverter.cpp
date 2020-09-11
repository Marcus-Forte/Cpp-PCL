#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <fstream>

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
        std::cerr << "Please give an .pcd or .txt file" << std::endl;
        exit(-1);
    }

    

    std::string cloudfile = argv[1];

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    	int index = cloudfile.find_last_of('.');
		std::string extension = cloudfile.substr(index + 1);
        std::string output_extension;
        bool isOutputTxt = false;

		if (extension.compare("pcd") == 0)
		{
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

			if (readTxt(cloudfile, *cloud) == -1)
			{
				PCL_ERROR("Cloud not open .txt file.\n");
				exit(-1);
			}
            output_extension = ".pcd";
            
		}

    

    // Find out the extension

    std::string outfile;
    int final = cloudfile.find_last_of('.');
    int init = cloudfile.find_last_of('/');
    // std::cout << "(" << init << "," << final << ")" << std::endl;
    outfile = cloudfile.substr(init + 1, final - init - 1);
    outfile += output_extension;
    // std::cout << outfile << std::endl;

    
    if(isOutputTxt){
    std::ofstream file;

    file.open(outfile);

    for (int i = 0; i < cloud->size(); ++i)
    {
        file << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;
    }

    file.close();
    std::cout << ".txt saved!" << std::endl;

    } else {

        pcl::io::savePCDFileASCII(outfile,*cloud);
        std::cout << ".pcd saved!" << std::endl;
        return 0;

    }

    
}