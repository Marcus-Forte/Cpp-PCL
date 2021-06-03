#pragma once
#include "pcl/point_cloud.h"
#include "pcl/search/kdtree.h"
// #include <pcl/kdtree/kdtree_flann.h>

namespace duna
{

    template <typename PointT, typename Scalar = float>
    class Registration
    {

    public:
        using PointCloudT = typename pcl::PointCloud<PointT>;
        using PointCloutTPtr = typename PointCloudT::Ptr;
        using PointCloutTConstPtr = typename PointCloudT::ConstPtr;

        using KdTree = pcl::search::KdTree<PointT>;
        using KdTreePtr = typename KdTree::Ptr;

        using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;

        static void extractFeatures(const PointCloudT &input, PointCloudT &corners, PointCloudT &surfaces);

        Registration()
        {
            surf_tree.reset(new KdTree);
            corner_tree.reset(new KdTree);
            // Parameters
            max_corr_dist = 0.2;
            corner_neighboors = 5;
            surface_neighboors = 8;

            KdTree_corn_ok = false;
            KdTree_surf_ok = false;
        }

        inline void setInputCorners(const PointCloutTConstPtr &corner_cloud)
        {
            input_corners = corner_cloud;
        }

        inline void setInputSurfaces(const PointCloutTConstPtr &surf_cloud)
        {
            input_surfaces = surf_cloud;
        }

        inline void setTargetCorners(const PointCloutTConstPtr &surf_cloud)
        {
            target_corners = surf_cloud;
        }

        inline void setTargetSurfaces(const PointCloutTConstPtr &corner_cloud)
        {
            PCL_DEBUG("Const load\n");
            target_surfaces = corner_cloud;
            // target_sufraces_modifiable = corner_cloud;
        }

        inline void setTargetSurfaces(const PointCloutTPtr &corner_cloud)
        {
            PCL_DEBUG("Non Const Load\n");
            target_surfaces = corner_cloud;
            target_sufraces_modifiable = corner_cloud;
        }

        inline void setTargetCornersSearchMethod(KdTreePtr tree, bool recompute = true)
        {
            corner_tree = tree;
            if (recompute)
            {
                PCL_DEBUG("computing target corner KDTREE\n");
                corner_tree->setInputCloud(target_corners);
            }
            KdTree_corn_ok = true;
        }

        inline void setTargetSurfacesSearchMethod(KdTreePtr tree, bool recompute = true)
        {
            surf_tree = tree;

            if (recompute)
            {
                PCL_DEBUG("computing target surfaces KDTREE\n");
                surf_tree->setInputCloud(target_surfaces);
            }
            KdTree_surf_ok = true;
        }

        // TODO Set Parameters

        void align();

        void align(const Matrix4 &guess);

        inline Matrix4 getFinalTransformation()
        {
            return final_transform;
        }

    private:
        KdTreePtr surf_tree;
        KdTreePtr corner_tree;

        PointCloutTConstPtr input_corners;
        PointCloutTConstPtr input_surfaces;

        PointCloutTConstPtr target_corners;
        PointCloutTConstPtr target_surfaces;

        PointCloutTPtr target_sufraces_modifiable;

        Matrix4 final_transform;

        bool KdTree_surf_ok;
        bool KdTree_corn_ok;

        // TODO Parameters
        Scalar max_corr_dist;
        unsigned int corner_neighboors;
        unsigned int surface_neighboors;

    protected:
    };

}
