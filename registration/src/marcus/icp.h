/* Author: Marcus Forte https://github.com/Marcus-Davi
   Email: davi2812@dee.ufc.br
*/

#pragma once

#include <pcl/registration/registration.h>
#include <pcl/registration/default_convergence_criteria.h>

#include "transform_normals.h" // mudar para .h
#include "correspondence_estimator.h"

template <typename PointSource, typename PointTarget, typename Scalar = float>
class MyRegistration : public pcl::Registration<PointSource, PointTarget, Scalar>
{

public:
    using PointCloudSource = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudSource;
    using PointCloudSourcePtr = typename PointCloudSource::Ptr;
    using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

    using PointCloudTarget = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudTarget;
    using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
    using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

    using PointIndicesPtr = pcl::PointIndices::Ptr;
    using PointIndicesConstPtr = pcl::PointIndices::ConstPtr;

    using Ptr = pcl::shared_ptr<MyRegistration<PointSource, PointTarget, Scalar>>;
    using ConstPtr = pcl::shared_ptr<const MyRegistration<PointSource, PointTarget, Scalar>>;

    using pcl::Registration<PointSource, PointTarget, Scalar>::reg_name_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::getClassName;
    using pcl::Registration<PointSource, PointTarget, Scalar>::input_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::indices_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::target_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::nr_iterations_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::max_iterations_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::previous_transformation_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::final_transformation_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::transformation_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::transformation_epsilon_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::transformation_rotation_epsilon_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::converged_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::corr_dist_threshold_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::inlier_threshold_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::min_number_correspondences_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::update_visualizer_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::euclidean_fitness_epsilon_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::correspondences_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::transformation_estimation_;
    // using pcl::Registration<PointSource, PointTarget, Scalar>::correspondence_estimation_; // Custom
    typename MyCorrespondenceEstimation<PointSource, PointTarget, Scalar>::Ptr correspondence_estimation_; // custom
    using pcl::Registration<PointSource, PointTarget, Scalar>::correspondence_rejectors_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::tree_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::force_no_recompute_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::target_cloud_updated_;
    // using pcl::Registration<PointSource, PointTarget, Scalar>::point_representation_;
    using pcl::PCLBase<PointSource>::deinitCompute;

    typename pcl::registration::DefaultConvergenceCriteria<Scalar>::Ptr convergence_criteria_;
    using Matrix4 = typename pcl::Registration<PointSource, PointTarget, Scalar>::Matrix4;

    MyRegistration()
        : x_idx_offset_(0), y_idx_offset_(0), z_idx_offset_(0), nx_idx_offset_(0), ny_idx_offset_(0), nz_idx_offset_(0), use_reciprocal_correspondence_(false), source_has_normals_(false), target_has_normals_(false)
    {
        reg_name_ = "MyRegistration";
        // transformation_estimation_.reset(new pcl::registration::TransformationEstimationSVD<PointSource, PointTarget, Scalar>());
        transformation_estimation_.reset(new MyTransformNormals<PointSource,PointTarget,Scalar>());
        // correspondence_estimation_.reset(new pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>);
        correspondence_estimation_.reset(new MyCorrespondenceEstimation<PointSource, PointTarget, Scalar>(1));
        convergence_criteria_.reset(new pcl::registration::DefaultConvergenceCriteria<Scalar>(nr_iterations_, transformation_, *correspondences_));
    };

    MyRegistration(int nearestK)
        : x_idx_offset_(0), y_idx_offset_(0), z_idx_offset_(0), nx_idx_offset_(0), ny_idx_offset_(0), nz_idx_offset_(0), use_reciprocal_correspondence_(false), source_has_normals_(false), target_has_normals_(false)
    {
        reg_name_ = "MyRegistration";
        transformation_estimation_.reset(new MyTransformNormals<PointSource,PointTarget,Scalar>());
        // transformation_estimation_.reset(new pcl::registration::TransformationEstimationSVD<PointSource, PointTarget, Scalar>());
        // correspondence_estimation_.reset(new pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>);
        correspondence_estimation_.reset(new MyCorrespondenceEstimation<PointSource, PointTarget, Scalar>(nearestK));
        convergence_criteria_.reset(new pcl::registration::DefaultConvergenceCriteria<Scalar>(nr_iterations_, transformation_, *correspondences_));
    };

    ~MyRegistration() {}

    inline typename pcl::registration::DefaultConvergenceCriteria<Scalar>::Ptr
    getConvergeCriteria()
    {
        return convergence_criteria_;
    }

    void
    setInputSource(const PointCloudSourceConstPtr &cloud) override
    {
        pcl::Registration<PointSource, PointTarget, Scalar>::setInputSource(cloud);
        const auto fields = pcl::getFields<PointSource>();
        source_has_normals_ = false;
        for (const auto &field : fields)
        {
            if (field.name == "x")  
                x_idx_offset_ = field.offset;
            else if (field.name == "y")
                y_idx_offset_ = field.offset;
            else if (field.name == "z")
                z_idx_offset_ = field.offset;
            else if (field.name == "normal_x")
            {
                source_has_normals_ = true;
                nx_idx_offset_ = field.offset;
            }
            else if (field.name == "normal_y")
            {
                source_has_normals_ = true;
                ny_idx_offset_ = field.offset;
            }
            else if (field.name == "normal_z")
            {
                source_has_normals_ = true;
                nz_idx_offset_ = field.offset;
            }
        }
    }
    void
    setInputTarget(const PointCloudTargetConstPtr &cloud) override
    {
        pcl::Registration<PointSource, PointTarget, Scalar>::setInputTarget(cloud);
        const auto fields = pcl::getFields<PointSource>();
        target_has_normals_ = false;
        for (const auto &field : fields)
        {
            if (field.name == "normal_x" || field.name == "normal_y" || field.name == "normal_z")
            {
                target_has_normals_ = true;
                break;
            }
        }
    }

    void
    setInputTarget(const PointCloudTargetPtr &cloud)
    {
        // PCL_DEBUG("MYREG: using modifiable target\n");
        modifiable_target_ = cloud;
        // pcl::Registration<PointSource, PointTarget, Scalar>::setInputTarget(cloud);
        const auto fields = pcl::getFields<PointSource>();
        target_has_normals_ = false;
        for (const auto &field : fields)
        {
            if (field.name == "normal_x" || field.name == "normal_y" || field.name == "normal_z")
            {
                target_has_normals_ = true;
                break;
            }
        }
    }

    /** \brief Set whether to use reciprocal correspondence or not
        *
        * \param[in] use_reciprocal_correspondence whether to use reciprocal correspondence or not
        */
    inline void
    setUseReciprocalCorrespondences(bool use_reciprocal_correspondence)
    {
        use_reciprocal_correspondence_ = use_reciprocal_correspondence;
    }

    /** \brief Obtain whether reciprocal correspondence are used or not */
    inline bool
    getUseReciprocalCorrespondences() const
    {
        return (use_reciprocal_correspondence_);
    }

     void align(PointCloudSource &output);

     void align(PointCloudSource &output, const Matrix4 &guess);

    bool initCompute();

protected:
    /** \brief Apply a rigid transform to a given dataset. Here we check whether whether
        * the dataset has surface normals in addition to XYZ, and rotate normals as well.
        * \param[in] input the input point cloud
        * \param[out] output the resultant output point cloud
        * \param[in] transform a 4x4 rigid transformation
        * \note Can be used with cloud_in equal to cloud_out
        */
    virtual void
    transformCloud(const PointCloudSource &input,
                   PointCloudSource &output,
                   const Matrix4 &transform);

    /** \brief Rigid transformation computation method  with initial guess.
        * \param output the transformed input point cloud dataset using the rigid transformation found
        * \param guess the initial guess of the transformation to compute
        */
    void
    computeTransformation(PointCloudSource &output, const Matrix4 &guess) override;

    /** \brief Looks at the Estimators and Rejectors and determines whether their blob-setter methods need to be called */
    virtual void
    determineRequiredBlobData();

    /** \brief XYZ fields offset. */
    std::size_t x_idx_offset_, y_idx_offset_, z_idx_offset_;

    /** \brief Normal fields offset. */
    std::size_t nx_idx_offset_, ny_idx_offset_, nz_idx_offset_;

    /** \brief The correspondence type used for correspondence estimation. */
    bool use_reciprocal_correspondence_;

    /** \brief Internal check whether source dataset has normals or not. */
    bool source_has_normals_;
    /** \brief Internal check whether target dataset has normals or not. */
    bool target_has_normals_;

    PointCloudTargetPtr modifiable_target_;

    /** \brief Checks for whether estimators and rejectors need various data */
    bool need_source_blob_, need_target_blob_;
};
