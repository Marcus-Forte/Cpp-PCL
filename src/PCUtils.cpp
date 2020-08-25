#include "PCUtils.h"

/* 


function cloud_floored = makeFloor(cloud,density)
step = round(1/density);
size_cloud = length(cloud);
orig = [0 0 0]';
normal = [0 0 1]';
size_floor = length(1:step:size_cloud);
Projected_points = zeros(size_floor,3);
k = 1;
for i=1:step:size_cloud
    p = cloud(i,:)';
v = p - orig;
dist = dot(v,normal);
Projected_points(k,:) = p - dist*normal;
k = k+1;
end
z_min = min(cloud(:,3));
Projected_points_offset = Projected_points + [0 0 z_min];
cloud_floored = [cloud;Projected_points_offset];
end

*/

void PCUtils::makeFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& floor, float density)
{ 
   
    int N = cloud_in->size();
    int step = (1/density); // TODO normalizar
    
    PCL_INFO("N = %d, step = %d\n",N,step);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr floora = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    
    floor->resize(N/step);

    
    Eigen::Vector4f min,max;
    pcl::getMinMax3D(*cloud_in,min,max);

    float z_min = min[2]; // comentário so pra te lembrar qu tu ficou 1h preso aqui por causa do indice errado...
    
    PCL_INFO("zmin = %f\n \n",z_min);

    // std::cout << min << std::endl;
    // std::cout << max << std::endl;
    

    Eigen::Vector3f origin(0,0,0);
    Eigen::Vector3f normal(0,0,1);

    Eigen::Vector3f v,p;
    float d;
    int i = 0;
    for (int index = 0;index < N; index+= step){
        p = cloud_in->points[index].getVector3fMap();
        // std::cout << p << std::endl;
        v = p - origin;
        d = v.dot(normal);


     floor->points[i].getVector3fMap() = p - d*normal;
     floor->points[i].z = z_min;    

     if(i < floor->size()-1)
        i++;
     
    }

    PCUtils::printPoints(floor,"floored");

    // cloud_out = floor;
    

    

    


}

// Blocking
void PCUtils::quickView(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in){

    pcl::visualization::PCLVisualizer::Ptr viewer = pcl::make_shared<pcl::visualization::PCLVisualizer>("quick viewer");

    viewer->addPointCloud(cloud_in,"cloud_in");
    viewer->addCoordinateSystem(1,"ref");

    while (! viewer->wasStopped()){

        viewer->spinOnce();
    }
    
    viewer->close(); //n fecha pq ?
    
    

}


void  PCUtils::printPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, const std::string& name){

std::cout << "Cloud : " << name  << "| points : " << cloud_in->size() << std::endl << std::endl;
for(int i=0; i < cloud_in->size();++i){
std::cout << "x = " << cloud_in->points[i].x << "|" ;
std::cout << "y = " << cloud_in->points[i].y << "|" ;
std::cout << "z = " << cloud_in->points[i].z << std::endl;

}

}


void PCUtils::fillCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out){



    
}