#include "PCUtils.h"

#include "pcl/filters/extract_indices.h"

#include <omp.h>
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
// Paint Cloud
// 3x (pi/2) de senoidais deslocadas
// T -> Periodo = 4 * max
// TODO --> USAR LUT, COMO ??
// TODO --> Entrar com nuvem, e nao ponto
void setColorMap(int n, int max, pcl::PointXYZRGB &pt)
{
    float min_color = 0;
    float max_color = 240;
    //Normaliza
    float t = n;
    float T = 4 * max;

    // Blue sine (lowest) começa com 90 graus
    float b = ((std::sin(2 * M_PI * t / T + M_PI_2))) * max_color; // ineficiente
    //Green sine (mid) // 45 graus
    float g = ((std::sin(3 * M_PI * t / T + M_PI_4))) * max_color;
    // Red sine (highest) // 0 graus
    float r = ((std::sin(2 * M_PI * t / T))) * max_color;
    pt.r = r;
    pt.g = g;
    pt.b = b;
    // std::cout << "RGB: " << r << "," << g << "," << b << std::endl;
}

static void Interpolate(const Eigen::MatrixXf &input, Eigen::MatrixXf &interpolated)
{
    interpolated = input;
    int cols = input.cols();
    int rows = input.rows();

    // std::cout << input.block<3,3>(cols-4,rows-4) << std::endl;

    for (int i = 0; i < input.size(); ++i)
    {

        if (input(i) == 0)
        { // INTERPOLATE AQ
        }
    }

    std::cout << "FIM INTERPOLATE\n";
}

void PCUtils::makeFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &floor, float density)
{

    int N = cloud_in->size();
    int step = (1 / density); // TODO normalizar

    PCL_INFO("N = %d, step = %d\n", N, step);

    pcl::PointCloud<pcl::PointXYZ>::Ptr floora = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    floor->resize(N / step);

    Eigen::Vector4f min, max;
    pcl::getMinMax3D(*cloud_in, min, max);

    float z_min = min[2]; // comentário so pra te lembrar qu tu ficou 1h preso aqui por causa do indice errado...

    PCL_INFO("zmin = %f\n \n", z_min);

    // std::cout << min << std::endl;
    // std::cout << max << std::endl;

    Eigen::Vector3f origin(0, 0, 0);
    Eigen::Vector3f normal(0, 0, 1);

    Eigen::Vector3f v, p;
    float d;
    int i = 0;
    for (int index = 0; index < N; index += step)
    {
        p = cloud_in->points[index].getVector3fMap();
        // std::cout << p << std::endl;
        v = p - origin;
        d = v.dot(normal);

        floor->points[i].getVector3fMap() = p - d * normal;
        floor->points[i].z = z_min;

        if (i < floor->size() - 1)
            i++;
    }

    // PCUtils::printPoints(floor, "floored");

    // cloud_out = floor;
}

template <typename pointT>
void PCUtils::printPoints(const pcl::PointCloud<pointT> &cloud_in, const std::string &name)
{

    std::cout << "Cloud : " << name << " | points : " << cloud_in.size() << std::endl
              << std::endl;
    for (int i = 0; i < cloud_in.size(); ++i)
    {
        std::cout << "x = " << cloud_in.points[i].x << "|";
        std::cout << "y = " << cloud_in.points[i].y << "|";
        std::cout << "z = " << cloud_in.points[i].z << std::endl;
    }
}

// Fill towards centroid
// Density -> extra points per current point. recommended = 2
void PCUtils::fillCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, int density)
{
    bool block = true;

    pcl::PointXYZ centroid;
    pcl::computeCentroid(*cloud_in, centroid);

    pcl::copyPointCloud(*cloud_in, *cloud_out);

    std::cout << "out cloud size : " << cloud_out->size() << std::endl;

    for (int i = 0; i < cloud_in->size(); ++i)
    {
        // std::cout << "####### ITERATION: " << i << " ##############" << std::endl;

        Eigen::Vector3f current_point = cloud_in->points[i].getVector3fMap();
        // std::cout << "current_point: " << cloud_in->points[i] << std::endl;

        // std::cout << "cenroid: " << centroid << std::endl;
        Eigen::Vector3f direction = centroid.getVector3fMap() - current_point; // vector
        // std::cout << "direction: " << direction << std::endl;
        //point insertion
        Eigen::Vector3f increment = direction / density;
        // std::cout << "increment: " << increment << std::endl;

        Eigen::Vector3f new_point = current_point;

        for (int j = 0; j < density - 1; ++j)
        {

            new_point += increment;

            // std::cout << "new_point: " << new_point << std::endl;

            cloud_out->push_back(pcl::PointXYZ(new_point[0], new_point[1], new_point[2])); //Preallocate !

            // std::cout << "checking .. " << cloud_out->points[cloud_out->size()-1] << std::endl;
        }

        // if(block){
        // char c = std::cin.get();
        // if(c == 'c')
        // block = false;
        // }
    }
}

// Ideia -> gerar uma malha 2D de resolução 'res'; pra cada celular da malha, pegar os ponhos de maior altura e calcular o volume do paralelepipedo
float PCUtils::computeVolume(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_in, float res)
{

    pcl::PointXYZ min, max;
    pcl::getMinMax3D(*cloud_in, min, max);
    std::cout << min << max << std::endl;
    int size_x = int((max.x + res - min.x) / res);
    int size_y = int((max.y + res - min.y) / res);
    Eigen::MatrixXf Cells;

    Cells.resize(size_x, size_y);

    Cells = Eigen::MatrixXf::Zero(size_x, size_y);

    // std::cout << Cells << std::endl;
    std::cout << "size = "
              << "(" << size_x << "," << size_y << ")" << std::endl;

    pcl::IndicesPtr pointIdxVec = pcl::make_shared<pcl::Indices>();

    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud(cloud_in);
    extractor.setIndices(pointIdxVec);

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(res);
    pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    octree.setInputCloud(cloud_in);

    std::cout << "cloud size : " << cloud_in->size() << std::endl;
    octree.addPointsFromInputCloud();

    pcl::PointXYZ box_max(min.x + res, min.y + res, max.z);
    pcl::PointXYZ box_min = min;
    box_max.z = max.z;

    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("janela"));
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud_in, 0, 255, 0);

    // viewer->addPointCloud(cloud_in, green, "in");
    // viewer->addCoordinateSystem(1, "ref");

    int i = 0;
    int j = 0;

    pcl::PointXYZ centroid;

    pcl::PointXYZ _min, _max;
    // Pode ser multithread
    std::cout << omp_get_max_threads() << std::endl;

    double ground = min.z;

    // #pragma omp parallel num_threads(2) shared(pointIdxVec,Cells)
    {
        while (box_max.y < max.y + res)
        {

            j = 0;

            while (box_max.x < max.x + res)
            {

                octree.boxSearch(box_min.getVector3fMap(), box_max.getVector3fMap(), *pointIdxVec);
                extractor.filter(*box_cloud);

                if (box_cloud->size())
                {
                    pcl::getMinMax3D(*box_cloud, _min, _max);
                    pcl::computeCentroid(*box_cloud, centroid);

                    // std::cout << "cloud extracted size = " <<cloud_out->size() << std::endl;
                    // std::cout << "(" << i << "," << j << ")" << std::endl;
                    // std::cout << _min << _max << std::endl;
                    // std::cout << centroid << std::endl;

                    // Maximum Height
                    Cells(j, i) = _max.z - ground; // GROUND

                    // Centroid
                    // Cells(j, i) = centroid.z - ground;

                    // FLOOR
                }
                else
                {
                    Cells(j, i) = 0; // Interpolar aqui. Marcar células p/ interpolar. (média dos adjacentes termos )
                }

                // Debug Animation
                // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(box_cloud,255,0,0);
                // viewer->addPointCloud(box_cloud,red,"extracted");
                // // viewer->updatePointCloud(cloud_out,red,"extracted");
                // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "extracted");
                // viewer->spinOnce();
                // std::this_thread::sleep_for(std::chrono::milliseconds(20));
                // viewer->removePointCloud("extracted");

                box_max.x += res;
                box_min.x += res;

                j++;
            }

            box_min.y += res;
            box_max.y += res;

            box_max.x = min.x + res;
            box_min.x = min.x;

            i++;
        }
    }

    // Processar interpoçao aqui

    Eigen::MatrixXf Interpolated;

    std::cout << "sum = " << Cells.sum() << std::endl;

    // Interpolate(Cells,Interpolated);

    // std::cout << Cells << std::endl << std::endl;
    // std::cout << Interpolated << std::endl;
    // std::cout << Interpolated(0) << "," << Interpolated(1) << std::endl;

    return Cells.sum() * res * res;
}



// typedef std::pair<float,int> mypair; //curvature + index

// bool comparator(const mypair& l, const mypair& r){
//     return l.first < r.first;
// }

// // Must preallocate
// template <class T> 
// void PCUtils::PlaneDetection(const pcl::PointCloud<T> &input_cloud, pcl::PointCloud<T> &features_cloud, int N, int N_planar)
// {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr window_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     // pcl::PointCloud<pcl::PointXYZ>::Ptr input_buffer(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr feature_points(new pcl::PointCloud<pcl::PointXYZ>);
//     // pcl::copyPointCloud(input_cloud, *input_buffer);
//     // Main Loop
//     int count = 0;
    

//     std::vector<mypair> curvatures;


//     for (int i = N; i < input_cloud.size() - N; i++)
//     {

//         window_cloud->clear();
//         window_cloud->push_back(input_cloud.points[i]);
//         for (int j = 1; j < N; ++j)
//         {
//             window_cloud->push_back(input_cloud.points[i + j]);
//             window_cloud->push_back(input_cloud.points[i - j]);
//         }

//         T centroid;
//         pcl::computeCentroid(*window_cloud, centroid);

//         // less means more smooth
//         // float smoothness = (centroid.getVector3fMap() - window_cloud->points[0].getVector3fMap()).norm();


        
//         Eigen::Vector3f sum = Eigen::Vector3f::Zero();
//         for(int k=1;k<window_cloud->size();++k){
//             sum += ( window_cloud->points[0].getVector3fMap() - window_cloud->points[k].getVector3fMap() );

//         }
//         // from LOAM
//         float smoothness = sum.norm() / ( window_cloud->points[0].getVector3fMap().norm() * window_cloud->size() );
//         mypair curv;
//         curv.first = smoothness;
//         curv.second = i;
//         curvatures.push_back(curv);
//         // curvature[count] = smoothness;
//         // count++;
        
//     }

//     std::sort(curvatures.begin(),curvatures.end(),comparator);
//     int index;
//     int index_low;
//     for(int j = 0;j<N_planar;++j){ // pick top ten
//         index = ( curvatures.begin() + j)->second; // index
//         index_low = (curvatures.end() - 1 - j)->second;
//         feature_points->points.push_back(input_cloud.points[index]);
//         // feature_points->points.push_back(input_cloud.points[index_low]);
//     }

//     pcl::copyPointCloud(*feature_points, features_cloud);
// }

// Instantiations

// Explicit Instantiation
template void PCUtils::printPoints<pcl::PointXYZ>(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, const std::string &name);
template void PCUtils::printPoints<pcl::PointXYZRGB>(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, const std::string &name);
// template void PCUtils::EdgeDetection(const pcl::PointCloud<pcl::PointXYZ> &input_cloud, pcl::PointCloud<pcl::PointXYZ> &features_cloud, int N);
// template void PCUtils::PlaneDetection(const pcl::PointCloud<pcl::PointXYZ> &input_cloud, pcl::PointCloud<pcl::PointXYZ> &features_cloud, int N, int N_planar);