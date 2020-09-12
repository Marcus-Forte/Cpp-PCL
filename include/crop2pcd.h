#include <vector>
#include <istream>
#include "pcl/point_cloud.h"
#include "utm.h"

// Parameters
// content -> content of a .crop file
// cloud -> cloud to be converted

// Returns :
// 0 if ok
// -1 if cant read file
// 1 if no origin

static int readCropfile(const std::string &filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, NavSatFix _origin = {0})
{
    int final = filename.find_last_of(".");
    int init = filename.find_first_of("/");

    bool hasOrigin = false;

    if (filename.substr(final + 1).compare("crop") != 0) //checa se é .crop
    {
        std::cerr << "Wrong file extension. Exiting" << std::endl;
        return -1;
    }

    std::ifstream file;
    file.open(filename);

   

    if(file.is_open() == false ){
        std::cerr << "Cannot open file " << filename << std::endl;
        return -1;
    }

    std::string line;

    NavSatFix Origin, Coords;
    Vector3 XY_Origin = {0}, XY;

    // Valid origin ?
    if (_origin.latitude != 0 && _origin.longitude != 0)
    {
        XY_Origin = LatLong2Utm(_origin);
        // std::cout << "crop2pcd : using argument origin" << std::endl;
        hasOrigin = true;
    }

    while (!file.eof())
    {
        std::getline(file, line);
        size_t Orig_pos, V_pos, Comment_pos;

        Orig_pos = line.find("o");
        V_pos = line.find("v");
        Comment_pos = line.find("#");

        // std::cout << "Orig_pos :" << Orig_pos << std::endl;
        // std::cout << "V_pos :" << V_pos << std::endl;
        // std::cout << "Comment_pos :" << Comment_pos << std::endl;

        std::cout.precision(10);

        //Found origin
        if (Orig_pos < V_pos && Orig_pos < Comment_pos && hasOrigin == false)
        {
            // std::cout << "Found origin" << std::endl;

            hasOrigin = true;
            sscanf(line.c_str(), "%*s %lf %lf %*s", &Origin.latitude, &Origin.longitude);

            XY_Origin = LatLong2Utm(Origin); // get from file
            // std::cout << "crop2pcd : using file origin" << std::endl;

            // std::cout << "Origin LLA \n";
            // std::cout << "Lat : " << Origin.latitude << std::endl;
            // std::cout << "Lon : " << Origin.longitude << std::endl;

            // std::cout << "Origin UTM \n";
            // std::cout << "E : " << XY_Origin.x << std::endl;
            // std::cout << "N : " << XY_Origin.y << std::endl;
        }

        // Found Vertex
        else if (V_pos < Orig_pos && V_pos < Comment_pos)
        {
            // std::cout << "Found vertex" << std::endl;
            sscanf(line.c_str(), "%*s %lf %lf %*s", &Coords.latitude, &Coords.longitude);

            XY = LatLong2Utm(Coords);

            double x = XY.x - XY_Origin.x;
            double y = XY.y - XY_Origin.y;

            cloud->push_back(pcl::PointXYZ(x, y, 0));

            // std::cout << "x = " << x << " | y = " << y << std::endl;
        }

        else if (Comment_pos < V_pos && Comment_pos < Orig_pos)
        {
            // std::cout << "Found comment" << std::endl;
        }
        else
        {
            // Nope
        }

    } // End of file parse

    if (hasOrigin)
        return 0;
    else
        return 1;
}
