#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <thread>

#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

#include <pcl/common/centroid.h>
// #include <pcl/common/cov

using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;

// Scan frame
double pseudoAngle2(const pcl::PointXYZ &pt)
{
    if (pt.x == 0 && pt.y == 0)
        return 0;

    if (pt.y <= 0)
    {
        if (-pt.y > pt.x)
            return pt.x / (-pt.y);
        else
            return 2 - (-pt.y) / pt.x;
    }
    else
    { // y > 0
        if (pt.x > pt.y)
            return 2 + pt.y / pt.x;
        else
            return 4 - pt.x / pt.y;
    }
}

// double pseudoAngle(pcl::PointXYZ pt)
// {

//     if (pt.y >= 0)
//     {

//         if (pt.x >= 0)
//         { //Q1
//             if (pt.x >= pt.y)
//                 return pt.y / pt.x;
//             else
//                 return 2 - pt.x / pt.y;
//         }
//         else
//         { // x < 0 Q2

//             if (-pt.x <= pt.y)
//                 return 2 + (-pt.x) / pt.y;
//             else
//                 return 4 - pt.y / (-pt.x);
//         }
//     }
//     else
//     { // y < 0
//         if (pt.x <= 0)
//         { //Q3
//             if (-pt.x >= -pt.y)
//                 return 4 + (-pt.y) / (-pt.x);
//             else
//                 return 6 - (-pt.x) / (-pt.y);
//         }
//         else // x > 0
//         {    //Q4
//             if (pt.x < -pt.y)
//                 return 6 + pt.x / (-pt.y);
//             else
//                 return 8 - (-pt.y) / (pt.x);
//         }
//     }
// }

// bool compareAngle(pcl::PointXYZ& pt1, pcl::PointXYZ& pt2)
// {
//     float a1 = pseudoAngle(pt1);
//     float a2 = pseudoAngle(pt2);
//     if (a1 < a2)
//         return true;
//     else
//         return false;
// }

void debugKey (const pcl::visualization::KeyboardEvent& key, void* cookie){
    bool* keyPressed = (bool*)cookie;
    if(key.getKeyCode() == 'a' && key.keyDown() == true)
        *keyPressed = true;

}

template <class T>
void ExtractFeature(const pcl::PointCloud<T> &input_cloud, int N, float *covariances)
{
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr window_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_buffer(new pcl::PointCloud<pcl::PointXYZ>); 

    pcl::PointCloud<pcl::PointXYZ>::Ptr feature_points(new pcl::PointCloud<pcl::PointXYZ>); 

    pcl::copyPointCloud(input_cloud,*input_buffer);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Debug"));
    viewer->addPointCloud(input_buffer,"input");
    viewer->addPointCloud(window_cloud,"window");
    viewer->addPointCloud(feature_points,"feature_points");
    viewer->addCoordinateSystem(1,"ref");
    viewer->setBackgroundColor(0,0,0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5,"window");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"window");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5,"feature_points");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,"feature_points");
    
    bool * keyPressed = new bool;
    *keyPressed = false;
    viewer->registerKeyboardCallback(debugKey,keyPressed);
    // window_cloud.resize(N);
    int count = 0;
    for (int i = 0; i < input_cloud.size() - N; i++)
    {
        
        window_cloud->clear();
        window_cloud->push_back(input_cloud.points[i]);
        for(int j=1;j<N;++j){
            window_cloud->push_back(input_cloud.points[i+j]);
        }

        cout << "## ITERATION " << count++ << " ##" << endl;
        for (auto it : window_cloud->points)
        {
            cout << it << endl;
        }



        // cout << "Centroid..." << endl;

        // pcl::PointXYZ xyz_centroid;
        // pcl::computeCentroid(*window_cloud, xyz_centroid);
        // cout << xyz_centroid << endl;

        // cout << "Covariance..." << endl;

        // float x_ = 0;
        // for (auto it : window_cloud->points)
        // {
        //     x_ += (it.x - xyz_centroid.x)*(it.x - xyz_centroid.x);
        // }

        // // x_ /= 3;
        // cout << "x_ = " << x_ << endl;

        // Eigen::Matrix3f covariance;
        // pcl::computeCovarianceMatrix(*window_cloud,xyz_centroid.getVector4fMap(), covariance);
        // cout << covariance << endl;
        // // cout << covariance.trace() << endl;

        // if(covariance.trace() > 1){ // probable edges
        //     *feature_points+= *window_cloud;
        //     i += N;
        // }
        
        int count_ = 0;
        viewer->removeAllShapes();
        Eigen::MatrixXf Y_(N,1);
        Eigen::MatrixXf X_(N,2);
        for( pcl::PointCloud<pcl::PointXYZ>::iterator it = window_cloud->begin(); it != window_cloud->end(); ++it){
            // std::string arrow_name = "arrow" + std::to_string(count_);
            // viewer->addArrow(*it,*(it+1),1,0,0,arrow_name);
            X_(count_,0) = it->x;
            X_(count_,1) = 1;

            Y_(count_,0) = it->y;


            count_++;
        }
        cout << "Regression ##" << endl;
        cout << "X_ = " << X_ << endl;
        cout << "Y_ = " << Y_ << endl;

        cout << "[a,b] = " << X_.colPivHouseholderQr().solve(Y_) << endl;
        

        viewer->updatePointCloud(window_cloud,"window");
        viewer->updatePointCloud(feature_points,"feature_points");
        
        while(*keyPressed == false){
            viewer->spinOnce();
        }
        *keyPressed = false;
        


        

        
    }
    
    
    
}

bool compareAngle2(pcl::PointXYZ &pt1, pcl::PointXYZ &pt2)
{
    float a1 = pseudoAngle2(pt1);
    float a2 = pseudoAngle2(pt2);
    if (a1 < a2)
        return true;
    else
        return false;
}

void PrintUsage()
{
    std::cout << "Usage : features [cloud.pcd]" << std::endl;
}

int main(int argc, char **argv)
{

    if (argc < 2)
    {
        PrintUsage();
        exit(-1);
    }

    PointCloudT::Ptr input_cloud = pcl::make_shared<PointCloudT>();

    pcl::io::loadPCDFile(argv[1], *input_cloud);

    // Preprocessing
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.05, 0.05);
    pass.setNegative(true);
    pass.filter(*input_cloud);

    for (PointCloudT::iterator it = input_cloud->begin(); it != input_cloud->end(); ++it)
    {
        // std::cout << "Index = " << it - input_cloud->begin() << std::endl;
        // std::cout << it->x;
    }
    // return 0;

    int cloud_size = input_cloud->size();
    PCL_INFO("Number of points -> %d \n", input_cloud->size());

    

    // std::sort(input_cloud->begin(), input_cloud->end(), compareAngle2); //PseudoAngle

    float covariances[2000] = {0};
    ExtractFeature<pcl::PointXYZ>(*input_cloud, 10, covariances);

    pcl::visualization::PCLVisualizer viewer;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr buffer_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // viewer.addPointCloud(input_cloud);
    viewer.addPointCloud(buffer_cloud, "buffer");
    viewer.setBackgroundColor(1, 1, 1);
    viewer.addCoordinateSystem(1, "ref");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "buffer");

    int count = 0;

    while (!viewer.wasStopped())
    {

        if (count < cloud_size)
        {
            // pcl::PointXYZRGB pt(input_cloud->points[count].x,input_cloud->points[count].y,input_cloud->points[count].z);
            pcl::PointXYZRGB pt;
            pt.x = input_cloud->points[count].x;
            pt.y = input_cloud->points[count].y;
            pt.z = input_cloud->points[count].z;
            pt.r = 255;
            pt.g = 0;
            pt.b = 0;
            buffer_cloud->push_back(pt);
            count++;
        }

        viewer.updatePointCloud(buffer_cloud, "buffer");
        // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,0,"buffer");
        viewer.spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    return 0;
}
