#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <algorithm>

#include <pcl/visualization/pcl_visualizer.h>

using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;

bool compareX(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2)
{
    if (p1.y > p2.y)
        return true;

    return false;
}

void printUsage()
{
    std::cout << "Usage : normals [target.pcd] [neighboors]" << std::endl;
}

int main(int argc, char **argv)
{

    if (argc < 3)
    {
        printUsage();
        exit(-1);
    }

    PointCloudT::Ptr cloud = pcl::make_shared<PointCloudT>();

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file model \n");
        return (-1);
    }
    // Sort X

    std::sort(cloud->points.begin(), cloud->points.end(), compareX);

    int KSearch = atoi(argv[2]);

    pcl::visualization::PCLVisualizer viewer("Viewer");
    viewer.addCoordinateSystem(1, "ref");
    int count = 0;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr Nearest_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    pcl::ModelCoefficients line;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::IndicesPtr indices(new pcl::Indices);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    std::vector<float> pointNKNSquaredDistance(KSearch);

    int rest = cloud->size();

    while (rest)
    {
        std::cout << "Rest = " << rest << std::endl;
        tree->nearestKSearch(cloud->points[0], KSearch, *indices, pointNKNSquaredDistance);
        pcl::ExtractIndices<pcl::PointXYZ> extractor;
        extractor.setInputCloud(cloud);
        extractor.setIndices(indices);
        extractor.filter(*Nearest_cloud);
        extractor.setNegative(true);
        extractor.filter(*cloud);
        rest = cloud->size();

        std::cout << "Computing Line..." << std::endl;
        seg.setOptimizeCoefficients(true);
        seg.setInputCloud(Nearest_cloud);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(200);
        seg.setDistanceThreshold(0.1);
        seg.segment(*inliers, line);

        pcl::PointCloud<pcl::PointXYZ>::Ptr inliner_clouds = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        extractor.setInputCloud(Nearest_cloud);
        extractor.setIndices(inliers);
        extractor.filter(*inliner_clouds);

        
        // viewer.addPointCloud(Nearest_cloud, "cloud" + std::to_string(count++));
        viewer.addPointCloud(inliner_clouds, "cloud" + std::to_string(count++));
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"cloud" + std::to_string(count-1));
        viewer.spinOnce(100, true);
    }

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

    pcl::PointCloud<pcl::Normal>::Ptr normals = pcl::make_shared<pcl::PointCloud<pcl::Normal>>();

    ne.setInputCloud(cloud);
    ne.setKSearch(KSearch);
    ne.compute(*normals);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = pcl::make_shared<pcl::PointCloud<pcl::PointNormal>>();

    pcl::concatenateFields(*cloud, *normals, *cloud_normals);

    viewer.addPointCloudNormals<pcl::PointNormal>(cloud_normals, 10, 0.1, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud");

    while (!viewer.wasStopped())
    {
        viewer.spin();
    }
}
