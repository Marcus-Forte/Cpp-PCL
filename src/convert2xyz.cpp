#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>

void Int2RGB(float intensity, float &r, float &g, float &b)
{
    const float b1 = 50;
    const float b0 = 0;
    float b_factor = M_PI_2 * (intensity - b0) / (b1 - b0);

    b = (cos(b_factor));

    const float g0 = 25;
    const float g1 = 90;
    float g_factor = M_PI * (intensity - g0) / (g1 - g0);

    g = (sin(g_factor));

    const float r0 = 80;
    const float r1 = 150;
    float r_factor = M_PI_2 * (intensity - r0) / (r1 - r0);

    r = (sin(r_factor));
}

int main(int argc, char **argv)
{

    if (argc < 2)
    {
        std::cerr << "No argument given\n";
        exit(-1);
    }

    std::cout << "Files: " << argc << std::endl;

    int num_threads_;
#ifdef _OPENMP

    num_threads_ = omp_get_max_threads();

#else
    num_threads_ = 1;
#endif
    printf("Using processors: %d\n", num_threads_);
#pragma omp parallel for num_threads(num_threads_)
    for (int i = 1; i < argc; ++i)
    {
        std::string filename(argv[i]);
        std::cout << "Converting: " << filename << " ...\n";
        pcl::PCDReader reader;
        pcl::PCLPointCloud2 cloud;

        reader.read(filename, cloud);
        int cloud_size = cloud.height * cloud.width;
        std::cout << "read: " << cloud_size << " points with " << cloud.fields.size() << " fields\n";

        pcl::PointCloud<pcl::PointXYZI> output_cloud;

        pcl::fromPCLPointCloud2(cloud, output_cloud);

        size_t init = filename.find_last_of('/');
        size_t end = filename.find_last_of('.');
        std::string out_filename = filename.substr(init + 1, (end - init) - 1) + ".xyz";
        std::ofstream file(out_filename);

        int min = 10000;
        int max = -10000;

        std::cout << "writing: " << out_filename << std::endl;
        for (auto it : output_cloud.points)
        {
            if (it.intensity < min)
                min = it.intensity;
            if (it.intensity > max)
                max = it.intensity;
            float r = 0, g = 0, b = 0;
            // std::cout << it.intensity << " ";
            Int2RGB(it.intensity, r, g, b);
            file << it.x << " " << it.y << " " << it.z << " " << r << " " << g << " " << b << std::endl;
        }
    }
}
