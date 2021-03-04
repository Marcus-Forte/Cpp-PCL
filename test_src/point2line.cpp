#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>

using namespace std;
int main(){


    // pcl::PointXYZ pt0(23,15,0);
    // pcl::PointXYZ pt1(2,6,0);
    // pcl::PointXYZ pt2(3,-5,0);


    pcl::PointXYZ pt0(0.7616,-3.4346,0);
    pcl::PointXYZ pt1(0.7416,-3.3909,0); // l0
    pcl::PointXYZ pt2(0.7611,-3.3855,0); // l1
    
     Eigen::Vector4f pt(pt0.x,pt0.y,0,0);
     Eigen::Vector4f line_pt(pt1.x,pt1.y,0,0);
     Eigen::Vector4f line_dir = Eigen::Vector4f::Zero();
     line_dir = pt2.getVector4fMap() - line_pt;
     line_dir[3] = 0;

    cout << "line_dir:" << line_dir << endl;
    cout << "line_pt:" << line_pt << endl;
    cout << "pt:" << pt << endl;



    std::cout << pcl::sqrPointToLineDistance(pt,line_pt,line_dir) << std::endl;

    // const Eigen::Vector4 pt()
    // const Eigen::Vector4 line_pt()
    // const Eigen::Vector4 line_dir()
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
    pc->push_back(pt0);
    pc->push_back(pt1);
    pc->push_back(pt2);
    pcl::Indices indices;
    indices.push_back(1);
    indices.push_back(2);
    pcl::PointXYZ line_ctr;
    pcl::computeCentroid(*pc,indices,line_ctr);


    pcl::visualization::PCLVisualizer viewer;
    viewer.addPointCloud(pc,"cloud");
    viewer.addArrow(pt1,pt2,1.0,0.0,0.0,true,"arrow1");
    viewer.addArrow(pt0,line_ctr,0.0,1.0,0.0,true,"arrow2");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10,"cloud");

    while(viewer.wasStopped() == false){
        viewer.spin();
    }
      
    
    



    return 0;
}