
#include "correspondence_estimator.h"

template <typename PointSource, typename PointTarget, typename Scalar>
void MyCorrespondenceEstimation<PointSource, PointTarget, Scalar>::determineCorrespondences(
    pcl::Correspondences &correspondences, double max_distance)
{
    if (!initCompute())
        return;

    double max_dist_sqr = max_distance * max_distance;

    correspondences.resize(indices_->size());

    std::vector<int> index(nearestK);
    std::vector<float> distance(nearestK);
    pcl::Correspondence corr;
    unsigned int nr_valid_correspondences = 0;

    // Check if the template types are the same. If true, avoid a copy.
    // Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
    if (pcl::isSamePointType<PointSource, PointTarget>())
    {
        // Iterate over the input set of source indices
        for (std::vector<int>::const_iterator idx = indices_->begin(); idx != indices_->end(); ++idx)
        {
            tree_->nearestKSearch(input_->points[*idx], nearestK, index, distance); // CHANGED
            if (distance[0] > max_dist_sqr)
                continue;

            float unused;
            Eigen::Vector4f normal;

            const PointTarget& probe = modifiable_target_->points[index[0]];
            // TODO melhorar heuristica
            if (probe.normal_x == 0 && probe.normal_y == 0 && probe.normal_z == 0 )
            {
                PCL_DEBUG("Updating normals ");
                pcl::computePointNormal(*modifiable_target_, index, normal, unused);
                // PCL_DEBUG("ok!-> %f %f %f\n",normal[0],normal[1],normal[2]);

                modifiable_target_->points[index[0]].normal_x = normal[0];
                modifiable_target_->points[index[0]].normal_y = normal[1];
                modifiable_target_->points[index[0]].normal_z = normal[2];
            }

            corr.index_query = *idx;
            corr.index_match = index[0];
            corr.distance = distance[0];
            correspondences[nr_valid_correspondences++] = corr;
        }
    }
    else
    {
        PointTarget pt;

        // Iterate over the input set of source indices
        for (std::vector<int>::const_iterator idx = indices_->begin(); idx != indices_->end(); ++idx)
        {
            // Copy the source data to a target PointTarget format so we can search in the tree
            copyPoint(input_->points[*idx], pt);

            tree_->nearestKSearch(pt, nearestK, index, distance);
            if (distance[0] > max_dist_sqr)
                continue;

            corr.index_query = *idx;
            corr.index_match = index[0];
            corr.distance = distance[0];
            correspondences[nr_valid_correspondences++] = corr;
        }
    }
    correspondences.resize(nr_valid_correspondences);
    deinitCompute();
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar>
void MyCorrespondenceEstimation<PointSource, PointTarget, Scalar>::determineReciprocalCorrespondences(
    pcl::Correspondences &correspondences, double max_distance)
{
    if (!initCompute())
        return;

    // setup tree for reciprocal search
    // Set the internal point representation of choice
    if (!initComputeReciprocal())
        return;
    double max_dist_sqr = max_distance * max_distance;

    correspondences.resize(indices_->size());
    std::vector<int> index(1);
    std::vector<float> distance(1);
    std::vector<int> index_reciprocal(1);
    std::vector<float> distance_reciprocal(1);
    pcl::Correspondence corr;
    unsigned int nr_valid_correspondences = 0;
    int target_idx = 0;

    // Check if the template types are the same. If true, avoid a copy.
    // Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
    if (pcl::isSamePointType<PointSource, PointTarget>())
    {
        // Iterate over the input set of source indices
        for (std::vector<int>::const_iterator idx = indices_->begin(); idx != indices_->end(); ++idx)
        {
            tree_->nearestKSearch(input_->points[*idx], nearestK, index, distance); // 20 nearest
            if (distance[0] > max_dist_sqr)
                continue;

            target_idx = index[0];

            tree_reciprocal_->nearestKSearch(target_->points[target_idx], 1, index_reciprocal, distance_reciprocal);
            if (distance_reciprocal[0] > max_dist_sqr || *idx != index_reciprocal[0])
                continue;

            corr.index_query = *idx;
            corr.index_match = index[0];
            corr.distance = distance[0];
            correspondences[nr_valid_correspondences++] = corr;
        }
    }
    else
    {
        PointTarget pt_src;
        PointSource pt_tgt;

        // Iterate over the input set of source indices
        for (std::vector<int>::const_iterator idx = indices_->begin(); idx != indices_->end(); ++idx)
        {
            // Copy the source data to a target PointTarget format so we can search in the tree
            copyPoint(input_->points[*idx], pt_src);

            tree_->nearestKSearch(pt_src, nearestK, index, distance); //
            if (distance[0] > max_dist_sqr)
                continue;

            target_idx = index[0];

            // Copy the target data to a target PointSource format so we can search in the tree_reciprocal
            copyPoint(target_->points[target_idx], pt_tgt);

            tree_reciprocal_->nearestKSearch(pt_tgt, nearestK, index_reciprocal, distance_reciprocal);
            if (distance_reciprocal[0] > max_dist_sqr || *idx != index_reciprocal[0])
                continue;

            corr.index_query = *idx;
            corr.index_match = index[0];
            corr.distance = distance[0];
            correspondences[nr_valid_correspondences++] = corr;
        }
    }
    correspondences.resize(nr_valid_correspondences);
    deinitCompute();
}

template <typename PointSource, typename PointTarget, typename Scalar>
bool MyCorrespondenceEstimation<PointSource, PointTarget, Scalar>::initCompute()
{
    if (!modifiable_target_)
    {
        PCL_ERROR("[pcl::registration::%s::compute] No input target dataset was given!\n", getClassName().c_str());
        return (false);
    }

    // Only update target kd-tree if a new target cloud was set
    if (target_cloud_updated_ && !force_no_recompute_)
    {
        PCL_DEBUG("[%s::compute] Computing KDTree\n",corr_name_.c_str());
        // If the target indices have been given via setIndicesTarget
        if (target_indices_)
            tree_->setInputCloud(modifiable_target_, target_indices_);
        else
            tree_->setInputCloud(modifiable_target_);

        target_cloud_updated_ = false;
    }

    return (pcl::PCLBase<PointSource>::initCompute());
}


// instantiation TODO como contornar isso ?
template class PCL_EXPORTS MyCorrespondenceEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal, float>;