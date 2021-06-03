#include "icp.h"

// IMPLEMENTATIONS

template <typename PointSource, typename PointTarget, typename Scalar>
void MyRegistration<PointSource, PointTarget, Scalar>::computeTransformation(
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
    determineRequiredBlobData();
    pcl::PCLPointCloud2::Ptr target_blob(new pcl::PCLPointCloud2);
    if (need_target_blob_)
        pcl::toPCLPointCloud2(*target_, *target_blob);

    // Pass in the default target for the Correspondence Estimation/Rejection code
    // std::cout << "Correspondence estimator" << std::endl;
    // correspondence_estimation_->setInputTarget(target_); // vaic chamar sempre base class... pq eh ponteiro da base
    // pcl::shared_ptr<MyCorrespondenceEstimation<PointSource,PointTarget,Scalar>> AAA;
    (correspondence_estimation_)->setInputTarget(modifiable_target_); // burlado ponteiro da base
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
            pcl::toPCLPointCloud2(*input_transformed, *input_transformed_blob);
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
        {
            correspondence_estimation_->determineCorrespondences(*correspondences_, corr_dist_threshold_);
        } // added

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

        // Compute correspondenctes normals on the fly (MARCUS)
        // tree_->nearestKSearch(it, kN, nearestK, unused);

        // Estimate the transform
        transformation_estimation_->estimateRigidTransformation(*input_transformed, *modifiable_target_, *correspondences_, transformation_);

        // Transform the data
        transformCloud(*input_transformed, *input_transformed, transformation_);

        // Obtain the final transformation
        final_transformation_ = transformation_ * final_transformation_;

        ++nr_iterations_;

        // Update the vizualization of icp convergence
        //if (update_visualizer_ != 0)
        //  update_visualizer_(output, source_indices_good, *target_, target_indices_good );

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

template <typename PointSource, typename PointTarget, typename Scalar>
void MyRegistration<PointSource, PointTarget, Scalar>::transformCloud(
    const PointCloudSource &input,
    PointCloudSource &output,
    const Matrix4 &transform)
{
    Eigen::Vector4f pt(0.0f, 0.0f, 0.0f, 1.0f), pt_t;
    Eigen::Matrix4f tr = transform.template cast<float>();

    // XYZ is ALWAYS present due to the templatization, so we only have to check for normals
    if (source_has_normals_)
    {
        Eigen::Vector3f nt, nt_t;
        Eigen::Matrix3f rot = tr.block<3, 3>(0, 0);

        for (std::size_t i = 0; i < input.size(); ++i)
        {
            const std::uint8_t *data_in = reinterpret_cast<const std::uint8_t *>(&input[i]);
            std::uint8_t *data_out = reinterpret_cast<std::uint8_t *>(&output[i]);
            memcpy(&pt[0], data_in + x_idx_offset_, sizeof(float));
            memcpy(&pt[1], data_in + y_idx_offset_, sizeof(float));
            memcpy(&pt[2], data_in + z_idx_offset_, sizeof(float));

            if (!std::isfinite(pt[0]) || !std::isfinite(pt[1]) || !std::isfinite(pt[2]))
                continue;

            pt_t = tr * pt;

            memcpy(data_out + x_idx_offset_, &pt_t[0], sizeof(float));
            memcpy(data_out + y_idx_offset_, &pt_t[1], sizeof(float));
            memcpy(data_out + z_idx_offset_, &pt_t[2], sizeof(float));

            memcpy(&nt[0], data_in + nx_idx_offset_, sizeof(float));
            memcpy(&nt[1], data_in + ny_idx_offset_, sizeof(float));
            memcpy(&nt[2], data_in + nz_idx_offset_, sizeof(float));

            if (!std::isfinite(nt[0]) || !std::isfinite(nt[1]) || !std::isfinite(nt[2]))
                continue;

            nt_t = rot * nt;

            memcpy(data_out + nx_idx_offset_, &nt_t[0], sizeof(float));
            memcpy(data_out + ny_idx_offset_, &nt_t[1], sizeof(float));
            memcpy(data_out + nz_idx_offset_, &nt_t[2], sizeof(float));
        }
    }
    else
    {
        for (std::size_t i = 0; i < input.size(); ++i)
        {
            const std::uint8_t *data_in = reinterpret_cast<const std::uint8_t *>(&input[i]);
            std::uint8_t *data_out = reinterpret_cast<std::uint8_t *>(&output[i]);
            memcpy(&pt[0], data_in + x_idx_offset_, sizeof(float));
            memcpy(&pt[1], data_in + y_idx_offset_, sizeof(float));
            memcpy(&pt[2], data_in + z_idx_offset_, sizeof(float));

            if (!std::isfinite(pt[0]) || !std::isfinite(pt[1]) || !std::isfinite(pt[2]))
                continue;

            pt_t = tr * pt;

            memcpy(data_out + x_idx_offset_, &pt_t[0], sizeof(float));
            memcpy(data_out + y_idx_offset_, &pt_t[1], sizeof(float));
            memcpy(data_out + z_idx_offset_, &pt_t[2], sizeof(float));
        }
    }
}

template <typename PointSource, typename PointTarget, typename Scalar>
void MyRegistration<PointSource, PointTarget, Scalar>::determineRequiredBlobData()
{
    need_source_blob_ = false;
    need_target_blob_ = false;
    // Check estimator
    need_source_blob_ |= correspondence_estimation_->requiresSourceNormals();
    need_target_blob_ |= correspondence_estimation_->requiresTargetNormals();
    // Add warnings if necessary
    if (correspondence_estimation_->requiresSourceNormals() && !source_has_normals_)
    {
        PCL_WARN("[pcl::%s::determineRequiredBlobData] Estimator expects source normals, but we can't provide them.\n", getClassName().c_str());
    }
    if (correspondence_estimation_->requiresTargetNormals() && !target_has_normals_)
    {
        PCL_WARN("[pcl::%s::determineRequiredBlobData] Estimator expects target normals, but we can't provide them.\n", getClassName().c_str());
    }
    // Check rejectors
    for (std::size_t i = 0; i < correspondence_rejectors_.size(); i++)
    {
        pcl::registration::CorrespondenceRejector::Ptr &rej = correspondence_rejectors_[i];
        need_source_blob_ |= rej->requiresSourcePoints();
        need_source_blob_ |= rej->requiresSourceNormals();
        need_target_blob_ |= rej->requiresTargetPoints();
        need_target_blob_ |= rej->requiresTargetNormals();
        if (rej->requiresSourceNormals() && !source_has_normals_)
        {
            PCL_WARN("[pcl::%s::determineRequiredBlobData] Rejector %s expects source normals, but we can't provide them.\n", getClassName().c_str(), rej->getClassName().c_str());
        }
        if (rej->requiresTargetNormals() && !target_has_normals_)
        {
            PCL_WARN("[pcl::%s::determineRequiredBlobData] Rejector %s expects target normals, but we can't provide them.\n", getClassName().c_str(), rej->getClassName().c_str());
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar>  void
MyRegistration<PointSource, PointTarget, Scalar>::align(PointCloudSource &output, const Matrix4 &guess)
{
    if (!initCompute())
        return;

    // Resize the output dataset
    if (output.points.size() != indices_->size())
        output.points.resize(indices_->size());
    // Copy the header
    output.header = input_->header;
    // Check if the output will be computed for all points or only a subset
    if (indices_->size() != input_->points.size())
    {
        output.width = static_cast<std::uint32_t>(indices_->size());
        output.height = 1;
    }
    else
    {
        output.width = static_cast<std::uint32_t>(input_->width);
        output.height = input_->height;
    }
    output.is_dense = input_->is_dense;

    // Copy the point data to output
    for (std::size_t i = 0; i < indices_->size(); ++i)
        output.points[i] = input_->points[(*indices_)[i]];

    // Set the internal point representation of choice unless otherwise noted
    // if (point_representation_ && !force_no_recompute_)
    //     tree_->setPointRepresentation(point_representation_);

    // Perform the actual transformation computation
    converged_ = false;
    final_transformation_ = transformation_ = previous_transformation_ = Matrix4::Identity();

    // Right before we estimate the transformation, we set all the point.data[3] values to 1 to aid the rigid
    // transformation
    for (std::size_t i = 0; i < indices_->size(); ++i)
        output.points[i].data[3] = 1.0;

    computeTransformation(output, guess);

    deinitCompute();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar>  void
MyRegistration<PointSource, PointTarget, Scalar>::align(PointCloudSource &output)
{
    align(output, Matrix4::Identity());
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar>
bool MyRegistration<PointSource, PointTarget, Scalar>::initCompute()
{
    if (!modifiable_target_)
    {
        PCL_ERROR("[pcl::registration::%s::compute] No input target dataset was given!\n", getClassName().c_str());
        return (false);
    }

    // Only update target kd-tree if a new target cloud was set
    if (target_cloud_updated_ && !force_no_recompute_)
    {
        PCL_DEBUG("[%s::compute] Computing KDTree\n", reg_name_.c_str());
        tree_->setInputCloud(modifiable_target_);
        target_cloud_updated_ = false;
    }

    // Update the correspondence estimation
    if (correspondence_estimation_)
    {
        correspondence_estimation_->setSearchMethodTarget(tree_, force_no_recompute_);
        // correspondence_estimation_->setSearchMethodSource(tree_reciprocal_, force_no_recompute_reciprocal_);
    }

    // Note: we /cannot/ update the search method on all correspondence rejectors, because we know
    // nothing about them. If they should be cached, they must be cached individually.

    return (pcl::PCLBase<PointSource>::initCompute());
}

// Instantiation
template class MyRegistration<pcl::PointXYZINormal, pcl::PointXYZINormal, float>;