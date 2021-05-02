#pragma once

template <typename PointSource, typename PointTarget, typename Scalar = float>
class MyICP : public pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>
{

public:
    using PointCloudSource = typename pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::PointCloudSource;
    using PointCloudSourcePtr = typename PointCloudSource::Ptr;
    using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

    using PointCloudTarget = typename pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::PointCloudTarget;
    using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
    using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;
    using Matrix4 = typename pcl::Registration<PointSource, PointTarget, Scalar>::Matrix4;

    // using pcl::Registration<PointSource, PointTarget, Scalar>::nr_iterations_;
    using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::reg_name_;
    using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::getClassName;
    using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::input_;
    using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::indices_;
    using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::target_;
    using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::nr_iterations_;
    using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::max_iterations_;
    using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::previous_transformation_;
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
    using pcl::Registration<PointSource, PointTarget, Scalar>::correspondence_estimation_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::correspondence_rejectors_;
    using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::need_source_blob_;
    using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::need_target_blob_;
    using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::source_has_normals_;
    using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::target_has_normals_;
    using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::use_reciprocal_correspondence_;
    // using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::setInputSource;
    // using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::setInputTarget;

    using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::determineRequiredBlobData;
    using pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::transformCloud;

    typename pcl::registration::DefaultConvergenceCriteria<Scalar>::Ptr convergence_criteria_;

    MyICP()
    {
        reg_name_ = "MyICP";
        transformation_estimation_.reset(new pcl::registration::TransformationEstimationSVD<PointSource, PointTarget, Scalar>());
        correspondence_estimation_.reset(new pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>);
        convergence_criteria_.reset(new pcl::registration::DefaultConvergenceCriteria<Scalar>(nr_iterations_, transformation_, *correspondences_));
    }

    ~MyICP()
    {
    }

protected:
    void computeTransformation(
        PointCloudSource &output, const Matrix4 &guess)
    {
        // Point cloud containing the correspondences of each point in <input, indices>
        PointCloudSourcePtr input_transformed(new PointCloudSource);

        nr_iterations_ = 0;
        converged_ = false;

        // Initialise final transformation to the guessed one
        final_transformation_ = guess;

        // If the guessed transformation is non identity
        if (guess != Matrix4::Identity())
        {
            input_transformed->resize(input_->size());
            // Apply guessed transformation prior to search for neighbours
            transformCloud(*input_, *input_transformed, guess);
        }
        else
            *input_transformed = *input_;

        transformation_ = Matrix4::Identity();

        // Make blobs if necessary
        // determineRequiredBlobData();
        pcl::PCLPointCloud2::Ptr target_blob(new pcl::PCLPointCloud2);
        if (need_target_blob_)
            pcl::toPCLPointCloud2(*target_, *target_blob);

        // Pass in the default target for the Correspondence Estimation/Rejection code
        correspondence_estimation_->setInputTarget(target_);
        if (correspondence_estimation_->requiresTargetNormals())
            correspondence_estimation_->setTargetNormals(target_blob);
        // Correspondence Rejectors need a binary blob
        for (std::size_t i = 0; i < correspondence_rejectors_.size(); ++i)
        {
            pcl::registration::CorrespondenceRejector::Ptr &rej = correspondence_rejectors_[i];
            if (rej->requiresTargetPoints())
                rej->setTargetPoints(target_blob);
            if (rej->requiresTargetNormals() && target_has_normals_)
                rej->setTargetNormals(target_blob);
        }

        convergence_criteria_->setMaximumIterations(max_iterations_);
        convergence_criteria_->setRelativeMSE(euclidean_fitness_epsilon_);
        convergence_criteria_->setTranslationThreshold(transformation_epsilon_);
        if (transformation_rotation_epsilon_ > 0)
            convergence_criteria_->setRotationThreshold(transformation_rotation_epsilon_);
        else
            convergence_criteria_->setRotationThreshold(1.0 - transformation_epsilon_);

        // Repeat until convergence
        do
        {
            // Get blob data if needed
            pcl::PCLPointCloud2::Ptr input_transformed_blob;
            if (need_source_blob_)
            {
                input_transformed_blob.reset(new pcl::PCLPointCloud2);
                toPCLPointCloud2(*input_transformed, *input_transformed_blob);
            }
            // Save the previously estimated transformation
            previous_transformation_ = transformation_;

            // Set the source each iteration, to ensure the dirty flag is updated
            correspondence_estimation_->setInputSource(input_transformed);
            if (correspondence_estimation_->requiresSourceNormals())
                correspondence_estimation_->setSourceNormals(input_transformed_blob);
            // Estimate correspondences
            if (use_reciprocal_correspondence_)
                correspondence_estimation_->determineReciprocalCorrespondences(*correspondences_, corr_dist_threshold_);
            else
                correspondence_estimation_->determineCorrespondences(*correspondences_, corr_dist_threshold_);

            //if (correspondence_rejectors_.empty ())
            pcl::CorrespondencesPtr temp_correspondences(new pcl::Correspondences(*correspondences_));
            for (std::size_t i = 0; i < correspondence_rejectors_.size(); ++i)
            {
                pcl::registration::CorrespondenceRejector::Ptr &rej = correspondence_rejectors_[i];
                PCL_DEBUG("Applying a correspondence rejector method: %s.\n", rej->getClassName().c_str());
                if (rej->requiresSourcePoints())
                    rej->setSourcePoints(input_transformed_blob);
                if (rej->requiresSourceNormals() && source_has_normals_)
                    rej->setSourceNormals(input_transformed_blob);
                rej->setInputCorrespondences(temp_correspondences);
                rej->getCorrespondences(*correspondences_);
                // Modify input for the next iteration
                if (i < correspondence_rejectors_.size() - 1)
                    *temp_correspondences = *correspondences_;
            }

            std::size_t cnt = correspondences_->size();
            // Check whether we have enough correspondences
            if (static_cast<int>(cnt) < min_number_correspondences_)
            {
                PCL_ERROR("[pcl::%s::computeTransformation] Not enough correspondences found. Relax your threshold parameters.\n", getClassName().c_str());
                convergence_criteria_->setConvergenceState(pcl::registration::DefaultConvergenceCriteria<Scalar>::CONVERGENCE_CRITERIA_NO_CORRESPONDENCES);
                converged_ = false;
                break;
            }

            // Estimate the transform
            transformation_estimation_->estimateRigidTransformation(*input_transformed, *target_, *correspondences_, transformation_);

            // Transform the data
            transformCloud(*input_transformed, *input_transformed, transformation_);

            // Obtain the final transformation
            final_transformation_ = transformation_ * final_transformation_;

            ++nr_iterations_;

            // Update the vizualization of icp convergence

            if (update_visualizer_ != 0)
            {
                PCL_DEBUG("Updating visualizer\n");
                std::vector<int> source_indices_good;
                std::vector<int> target_indices_good;

                for (int i = 0; i < cnt; ++i)
                {
                    source_indices_good.push_back((*correspondences_)[i].index_query);
                    target_indices_good.push_back((*correspondences_)[i].index_match);
                }
                update_visualizer_(*input_transformed, source_indices_good, *target_, target_indices_good);
                usleep(1000000);
            }

            converged_ = static_cast<bool>((*convergence_criteria_));
        } while (convergence_criteria_->getConvergenceState() == pcl::registration::DefaultConvergenceCriteria<Scalar>::CONVERGENCE_CRITERIA_NOT_CONVERGED);

        // Transform the input cloud using the final transformation
        PCL_DEBUG("Transformation is:\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n",
                  final_transformation_(0, 0), final_transformation_(0, 1), final_transformation_(0, 2), final_transformation_(0, 3),
                  final_transformation_(1, 0), final_transformation_(1, 1), final_transformation_(1, 2), final_transformation_(1, 3),
                  final_transformation_(2, 0), final_transformation_(2, 1), final_transformation_(2, 2), final_transformation_(2, 3),
                  final_transformation_(3, 0), final_transformation_(3, 1), final_transformation_(3, 2), final_transformation_(3, 3));

        // Copy all the values
        output = *input_;
        // Transform the XYZ + normals
        transformCloud(*input_, output, final_transformation_);
    }
};