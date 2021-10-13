#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>

using PointCloudT = pcl::PCLPointCloud2;

using ColorHandler = pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>;
using ColorHandlerPtr = ColorHandler::Ptr;
using ColorHandlerConstPtr = ColorHandler::ConstPtr;

using PointListT = std::vector<pcl::PointXYZ>;
using PixelListT = std::vector<cv::Point2i>;

PointListT pointList;
PixelListT pixelList;


void Optimize(const PointListT& points, const PixelListT& pixels, Eigen::Matrix4d intrinsics){
    
    // I -> Projection matrix from Intrinsics
    // E -> Extrinsics * frame conversion (i.e camera frame is rotated with respect to lidar frame)
    // E -> 6 parameters (rx ry rz tx ty tz) to optimize cost

    // minimize cost: 
    // error[i] = pixel[i] - I * E * point[i] (two dimensional)


    

}

void mouseCallbackImg(const pcl::visualization::MouseEvent &event, void *cookie)
{
    pcl::visualization::ImageViewer &viewer = *(*((pcl::visualization::ImageViewer::Ptr *)cookie));
    const std::string layer = "circles";
    if (event.getButton() == event.LeftButton && event.getType() == event.MouseButtonPress)
    {
        int x = event.getX();
        int y = event.getY();
        PCL_INFO("x: %d y: %d\n", event.getX(), event.getY());

        cv::Point2i pixel(x, y);
        pixelList.push_back(pixel);

        viewer.removeLayer(layer);
        for (int i = 0; i < pixelList.size(); ++i)
        {
            viewer.addText(pixelList[i].x-15, pixelList[i].y-15,std::to_string(i + 1),1,0,0,layer);
            viewer.addCircle(pixelList[i].x, pixelList[i].y, 3, 1, 0, 0, layer);
        }
    }
    else if (event.getButton() == event.RightButton && event.getType() == event.MouseButtonPress)
    {
        viewer.removeLayer(layer);
        pixelList.clear();
    }
}

void ppCallback(const pcl::visualization::PointPickingEvent &event, void *cookie)
{
    pcl::visualization::PCLVisualizer &viewer = *(*((pcl::visualization::PCLVisualizer::Ptr *)cookie));
    float x, y, z;
    event.getPoint(x, y, z);
    printf("Point Pick: (%f,%f,%f)\n", x, y, z);
    pcl::PointXYZ pt(x, y, z);

    pointList.push_back(pt);

    viewer.removeAllShapes();
    for (int i = 0; i < pointList.size(); ++i)
    {
        viewer.addText3D(std::to_string(i + 1), pointList[i], 0.1);
        viewer.addSphere(pointList[i], 0.05, 1, 0, 0, "s" + std::to_string(i + 1));
    }
}

// clear selection
void mouseCallbackPC(const pcl::visualization::MouseEvent &event, void *cookie)
{
    pcl::visualization::PCLVisualizer &viewer = *(*((pcl::visualization::PCLVisualizer::Ptr *)cookie));

    if (event.getButton() == event.RightButton && event.getType() == event.MouseButtonPress)
    {
        pointList.clear();
        viewer.removeAllShapes();
    }
}

int main(int argc, char **argv)
{

    if (argc < 3)
    {
        std::cerr << "usage: laser_camera_calibration image.jpeg cloud.pcd" << std::endl;
        exit(0);
    }
    cv::Mat img = cv::imread(argv[1]);

    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);

    if (pcl::io::loadPCDFile(argv[2], *cloud) == -1)
    {
        PCL_ERROR("Cloud not open .pcd file.\n");
        exit(-1);
    }

    // viewer.addRGBImage()
    pcl::visualization::ImageViewer::Ptr viewer(new pcl::visualization::ImageViewer);
    
    pcl::visualization::PCLVisualizer::Ptr c_viewer(new pcl::visualization::PCLVisualizer);
    // pcl::visualization::CloudViewer::Ptr c_viewer (new  pcl::visualization::CloudViewer("cloud"));

    viewer->addRGBImage(reinterpret_cast<unsigned char const *>(img.data), img.cols, img.rows);
    viewer->registerMouseCallback(mouseCallbackImg, &viewer);

    Eigen::Vector4f origin = Eigen::Vector4f::Zero();
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();

    //add color handlers
    ColorHandlerPtr color;
    for (int i = 0; i < cloud->fields.size(); ++i)
    {
        std::string field_name = cloud->fields[i].name;
        std::cout << "Color field: " << field_name << std::endl;
        if (field_name == "rgba")
        {

            color.reset(new pcl::visualization::PointCloudColorHandlerRGBAField<PointCloudT>(cloud));
        }
        else if (field_name == "rgb")
        {
            color.reset(new pcl::visualization::PointCloudColorHandlerRGBField<PointCloudT>(cloud));
        }
        else
        {
            color.reset(new pcl::visualization::PointCloudColorHandlerGenericField<PointCloudT>(cloud, field_name));
        }
        c_viewer->addPointCloud(cloud, color, origin, orientation, "cloud");
    }

    c_viewer->registerPointPickingCallback(ppCallback, &c_viewer);
    c_viewer->registerMouseCallback(mouseCallbackPC, &c_viewer);

    while (!viewer->wasStopped() && !c_viewer->wasStopped())
    {
        viewer->spinOnce();
        c_viewer->spinOnce();
    }

    // while(!c_viewer.wasStopped());

    return 0;
}