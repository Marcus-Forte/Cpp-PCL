#ifndef TXT2PC_
#define TXT2PC_

#include <fstream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

pcl::PointCloud<pcl::PointXYZ>* ReadTxt(const char* filename);








#endif 
