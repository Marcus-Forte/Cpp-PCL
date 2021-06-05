#pragma once

#include "pcl/point_cloud.h"
#include "pcl/search/kdtree.h"
#include "pcl/features/normal_3d.h"
#include "pcl/common/transforms.h"
#include "pcl/correspondence.h"
#include "pcl/registration/default_convergence_criteria.h"
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

            max_iterations_ = 10;

            KdTree_corn_ok = false;
            KdTree_surf_ok = false;
            final_transformation_.setIdentity();

            surfaces_correspondences.reset(new pcl::Correspondences);
            corners_correspondences.reset(new pcl::Correspondences);

            // Parameters
            euclidean_fitness_epsilon_ = -std::numeric_limits<double>::max();
            min_number_correspondences_ = 3;
            transformation_epsilon_ = 0;
            transformation_rotation_epsilon_ = 0;

            convergence_criteria_.reset(new pcl::registration::DefaultConvergenceCriteria<Scalar>(nr_iterations_, transformation_, *surfaces_correspondences));
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

        // TODO Set Parameters

        inline void setMaxCorrDist(const float &dist) { max_corr_dist = dist; }

        inline void setMaximumIterations(const int max) { max_iterations_ = max; }

        inline void setTransformationEpsilon(double epsilon) { transformation_epsilon_ = epsilon; }

        inline void setTransformationRotationEpsilon(double epsilon) { transformation_rotation_epsilon_ = epsilon; }

        inline bool hasConverged() const { return (converged_); }
        void align();

        void align(const Matrix4 &guess);

        inline Matrix4 getFinalTransformation()
        {
            return final_transformation_;
        }


        inline void setVisualize(bool visualize_) {visualize = visualize_; }

    private:
        KdTreePtr surf_tree;
        KdTreePtr corner_tree;

        PointCloudTConstPtr input_corners;
        PointCloudTConstPtr input_surfaces;

        PointCloudTConstPtr target_corners;
        PointCloudTConstPtr target_surfaces;

        pcl::CorrespondencesPtr surfaces_correspondences;
        pcl::CorrespondencesPtr corners_correspondences;

        bool KdTree_surf_ok;
        bool KdTree_corn_ok;

        PointCloudTPtr target_sufraces_modifiable;

        // TODO Parameters

        /** \brief The final transformation matrix estimated by the registration method after N iterations. */
        Matrix4 final_transformation_;

        /** \brief The transformation matrix estimated by the registration method. */
        Matrix4 transformation_;

        Scalar max_corr_dist;

        unsigned int kn_corner_neighboors;
        unsigned int kn_surface_neighboors;

        /** \brief The maximum difference between two consecutive transformations in order to consider convergence 
        * (user defined). 
        */
        double transformation_epsilon_;

        /** \brief The maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the 
        * algorithm is considered to have converged. The error is estimated as the sum of the differences between 
        * correspondences in an Euclidean sense, divided by the number of correspondences.
        */
        double euclidean_fitness_epsilon_;

        /** \brief The minimum number of correspondences that the algorithm needs before attempting to estimate the 
        * transformation. The default value is 3.
        */
        int min_number_correspondences_;

        /** \brief The maximum rotation difference between two consecutive transformations in order to consider convergence
        * (user defined).
        */
        double transformation_rotation_epsilon_;

        /** \brief The number of iterations the internal optimization ran for (used internally). */
        int nr_iterations_;

        /** \brief The maximum number of iterations the internal optimization should run for.
        * The default value is 10.
        */
        int max_iterations_;

        /** \brief Holds internal convergence state, given user parameters. */
        bool converged_;

        typename pcl::registration::DefaultConvergenceCriteria<Scalar>::Ptr convergence_criteria_;


        bool visualize = false;

    protected:
    };

}
