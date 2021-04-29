#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
#include <algorithm>

#include <pcl/visualization/pcl_visualizer.h>
#include <ctime>
using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;

void printUsage()
{
    std::cout << "Usage : normals [target.pcd] [neighboors] [random sampling factor]" << std::endl;
}

int main(int argc, char **argv)
{

    pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);
    
    if (argc < 4)
    {
        printUsage();
        exit(-1);
    }

    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);

    if (pcl::io::loadPCDFile(argv[1], *cloud_blob) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file model \n");
        return (-1);
    }

    int kN = atoi(argv[2]);

    for (const auto &field : cloud_blob->fields)
    {
        PCL_INFO("f: %s\n", field.name.c_str());
    }

    clock_t start,elapsed;

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr random_samples(new pcl::PointCloud<pcl::PointNormal>);

    pcl::fromPCLPointCloud2(*cloud_blob, *cloud);

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud(cloud);

    pcl::RandomSample<pcl::PointNormal> ransamp;
    ransamp.setInputCloud(cloud);
    int samples = atof(argv[3]) * cloud->size();
    ransamp.setSample(samples);
    ransamp.filter(*random_samples);

    PCL_INFO("Coputing Normals on %d random samples...\n",samples);
    
    //for each point, compute its normal
    pcl::Indices nearestK(kN);
    std::vector<float> unused(kN);
    start = clock();
    for (auto &it : random_samples->points)
    {

        tree->nearestKSearch(it, kN, nearestK, unused);
        Eigen::Vector4f normal;
        float curvature;
        pcl::computePointNormal(*cloud, nearestK, normal, curvature);

        it.normal_x = normal[0];
        it.normal_y = normal[1];
        it.normal_z = normal[2];

        // PCL_INFO("nx : %f ny: %f nz: %f\n", normal[0], normal[1], normal[2]);
    }

    elapsed = clock() - start;
    PCL_INFO("Random Sampling normal computation time: %f\n",(float)elapsed / CLOCKS_PER_SEC);

    PCL_INFO("Coputing Normals on full cloud : %d pts...\n",cloud->size());
    start = clock();
    pcl::NormalEstimation<pcl::PointNormal,pcl::PointNormal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(kN);
    ne.compute(*cloud);
    elapsed = clock() - start;
    PCL_INFO("Computation time: %f\n",(float)elapsed / CLOCKS_PER_SEC);

    pcl::visualization::PCLVisualizer viewer;

    // pcl::PCLPointCloud2Ptr cloud_blob_normals (new pcl::PCLPointCloud2);
    // pcl::toPCLPointCloud2<pcl::PointNormal>(*cloud_normals, *cloud_blob_normals);

    // pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::Ptr color;

    // for (const auto &it : cloud_blob_normals->fields)
    // {

    //     color.reset(new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2>(cloud_blob_normals, it.name));
    //     Eigen::Vector4f origin = Eigen::Vector4f::Zero();
    //     Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();

    //     viewer.addPointCloud(cloud_blob_normals,color,origin,orientation,"cloud");
    // }

    viewer.addPointCloud<pcl::PointNormal>(cloud, "cloud");
    viewer.addPointCloud<pcl::PointNormal>(random_samples, "random");
    viewer.addPointCloudNormals<pcl::PointNormal>(random_samples, 1, 0.06, "normals"); // normals

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "normals");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "random");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "random");

    while (!viewer.wasStopped())
    {
        viewer.spin();
    }

    return 0;
}
