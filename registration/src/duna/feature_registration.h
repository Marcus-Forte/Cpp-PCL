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
        using PointCloudTPtr = typename PointCloudT::Ptr;
        using PointCloudTConstPtr = typename PointCloudT::ConstPtr;

        using KdTree = pcl::search::KdTree<PointT>;
        using KdTreePtr = typename KdTree::Ptr;

        using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
        using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;
        using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
        using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
        using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

        static void extractFeatures(const PointCloudT &input, PointCloudT &corners, PointCloudT &surfaces);

        Registration()
        {
            surf_tree.reset(new KdTree);
            corner_tree.reset(new KdTree);
            // Parameters
            max_corr_dist = 0.2;
            kn_corner_neighboors = 5;
            kn_surface_neighboors = 8;

            max_iterations = 100;

            KdTree_corn_ok = false;
            KdTree_surf_ok = false;
            final_transformation_.setIdentity();
        }

        inline void setInputCorners(const PointCloudTConstPtr &corner_cloud)
        {
            input_corners = corner_cloud;
        }

        inline void setInputSurfaces(const PointCloudTConstPtr &surf_cloud)
        {
            input_surfaces = surf_cloud;
        }

        inline void setTargetCorners(const PointCloudTConstPtr &surf_cloud)
        {
            target_corners = surf_cloud;
        }

        inline void setTargetSurfaces(const PointCloudTConstPtr &corner_cloud)
        {
            PCL_DEBUG("Const load\n");
            target_surfaces = corner_cloud;
            // target_sufraces_modifiable = corner_cloud;
        }

        inline void setTargetSurfaces(const PointCloudTPtr &corner_cloud)
        {
            PCL_DEBUG("Non Const Load\n");
            target_surfaces = corner_cloud;
            target_sufraces_modifiable = corner_cloud;
        }

        // TODO improve order of computation of tree
        inline void setTargetCornersSearchMethod(const KdTreePtr &tree, bool recompute = true)
        {
            corner_tree = tree;
            if (recompute)
            {
                PCL_DEBUG("computing target corner KDTREE\n");
                corner_tree->setInputCloud(target_corners); // TODO feedback if sucess?
            }
            KdTree_corn_ok = true;
        }

        inline void setTargetSurfacesSearchMethod(const KdTreePtr &tree, bool recompute = true)
        {
            surf_tree = tree;

            if (recompute)
            {
                PCL_DEBUG("computing target surfaces KDTREE\n");
                surf_tree->setInputCloud(target_surfaces);
            }
            KdTree_surf_ok = true;
        }

        inline void setMaxCorrDist(const float &dist)
        {
            max_corr_dist = dist;
        }

        inline void setMaximumIterations(const int max)
        {
            max_iterations = max;
        }

        // TODO Set Parameters

        void align();

        void align(const Matrix4 &guess);

        inline Matrix4 getFinalTransformation()
        {
            return final_transformation_;
        }

    private:
        KdTreePtr surf_tree;
        KdTreePtr corner_tree;

        PointCloudTConstPtr input_corners;
        PointCloudTConstPtr input_surfaces;

        PointCloudTConstPtr target_corners;
        PointCloudTConstPtr target_surfaces;

        PointCloudTPtr target_sufraces_modifiable;

        /** \brief The final transformation matrix estimated by the registration method after N iterations. */
        Matrix4 final_transformation_;

        /** \brief The transformation matrix estimated by the registration method. */
        Matrix4 transformation_;

        bool KdTree_surf_ok;
        bool KdTree_corn_ok;

        // TODO Parameters
        Scalar max_corr_dist;
        unsigned int kn_corner_neighboors;
        unsigned int kn_surface_neighboors;
        unsigned int max_iterations;

    protected:
    };

}
