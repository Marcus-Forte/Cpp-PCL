/* Author: Marcus Forte https://github.com/Marcus-Davi
   Email: davi2812@dee.ufc.br
*/

#pragma once

#include <pcl/features/normal_3d.h>
#include <pcl/registration/correspondence_estimation.h>

struct MyCorrespondence
{
  /** \brief Index of the query (source) point. */
  int index_query = 0;
  /** \brief Index of the matching (target) point. Set to -1 if no correspondence found. */
  int index_match = -1;
  /** \brief Distance between the corresponding points, or the weight denoting the confidence in correspondence estimation */
  union
  {
    float distance = std::numeric_limits<float>::max();
    float weight;
  };

  float tgt_normal = 0; // ADDED!!

  /** \brief Standard constructor. 
      * Sets \ref index_query to 0, \ref index_match to -1, and \ref distance to FLT_MAX.
      */
  inline MyCorrespondence() = default;

  /** \brief Constructor. */
  inline MyCorrespondence(int _index_query, int _index_match, float _distance) : index_query(_index_query), index_match(_index_match), distance(_distance)
  {
  }

  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename PointSource, typename PointTarget, typename Scalar = float>
class MyCorrespondenceEstimation : public pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>
{
public:
  using Ptr = pcl::shared_ptr<MyCorrespondenceEstimation<PointSource, PointTarget, Scalar>>;
  using ConstPtr = pcl::shared_ptr<const MyCorrespondenceEstimation<PointSource, PointTarget, Scalar>>;

  using KdTree = pcl::search::KdTree<PointTarget>;
  using KdTreePtr = typename KdTree::Ptr;

  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = pcl::PointCloud<PointTarget>;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

  using PointRepresentationConstPtr = typename KdTree::PointRepresentationConstPtr;

  using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::point_representation_;
  using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_transformed_;
  using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_;
  using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_reciprocal_;
  using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_;
  using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_cloud_updated_;
  using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::corr_name_;
  using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_indices_;
  using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::getClassName;
  using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initCompute;
  using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initComputeReciprocal;
  using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_;
  using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::indices_;
  using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_fields_;
  using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::force_no_recompute_;
  
  using pcl::PCLBase<PointSource>::deinitCompute;

  /** \brief Empty constructor. */
  MyCorrespondenceEstimation()
  {
    corr_name_ = "MyCorrespondenceEstimation";
  }

  MyCorrespondenceEstimation(int K)
  {
    nearestK = K;
    corr_name_ = "MyCorrespondenceEstimation";
    // MyCorrespondenceEstimation();
  }

  /** \brief Empty destructor */
  ~MyCorrespondenceEstimation() {}

  /** \brief Determine the correspondences between input and target cloud.
          * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
  void
  determineCorrespondences(pcl::Correspondences &correspondences,
                           double max_distance = std::numeric_limits<double>::max()) override;

  /** \brief Determine the reciprocal correspondences between input and target cloud.
          * A correspondence is considered reciprocal if both Src_i has Tgt_i as a 
          * correspondence, and Tgt_i has Src_i as one.
          *
          * \param[out] correspondences the found correspondences (index of query and target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
  void
  determineReciprocalCorrespondences(pcl::Correspondences &correspondences,
                                     double max_distance = std::numeric_limits<double>::max()) override;

  /** \brief Clone and cast to CorrespondenceEstimationBase */
  typename pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::Ptr
  clone() const override
  {
    Ptr copy(new MyCorrespondenceEstimation<PointSource, PointTarget, Scalar>(*this));
    return (copy);
  }

  inline void setInputTarget(PointCloudTargetPtr &cloud)
  {
    
    if (cloud->points.empty())
    {
      PCL_ERROR("[pcl::registration::%s::setInputTarget] Invalid or empty point cloud dataset given!\n", getClassName().c_str());
      return;
    }
    modifiable_target_ = cloud;

    // Set the internal point representation of choice
    if (point_representation_)
      tree_->setPointRepresentation(point_representation_);

    target_cloud_updated_ = true;
  }

  /** \brief Internal computation initialization. */
  bool
  initCompute();

private:
  int nearestK = 1;
  PointCloudTargetPtr modifiable_target_;
};
