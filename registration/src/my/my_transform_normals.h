#pragma once

#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/warp_point_rigid_6d.h>
#include <pcl/console/print.h>

//idea for Feature-ICP usage;
// icp.setSourceEdges
// icp.setTargetEdges
// icp.setSourcePlanes
// icp.setTargetPlanes
// icp.setTransformationEstimation(edge p2l + plane p2p)

// mt trabalho!

using namespace pcl::registration;

template <typename PointSource, typename PointTarget, typename MatScalar = float>
class MyTransformNormals : public TransformationEstimation<PointSource, PointTarget, MatScalar>
{
public:
    using Ptr = pcl::shared_ptr<MyTransformNormals<PointSource, PointTarget, MatScalar>>;
    using ConstPtr = pcl::shared_ptr<const MyTransformNormals<PointSource, PointTarget, MatScalar>>;
    using VectorX = Eigen::Matrix<MatScalar, Eigen::Dynamic, 1>;
    using MatrixX = Eigen::Matrix<MatScalar, Eigen::Dynamic, Eigen::Dynamic>;
    using Vector3 = Eigen::Matrix<MatScalar, 3, 1>;
    using Vector4 = Eigen::Matrix<MatScalar, 4, 1>;
    using Matrix3 = Eigen::Matrix<MatScalar, 3, 3>;
    using Matrix4 = typename TransformationEstimation<PointSource, PointTarget, MatScalar>::Matrix4;

    MyTransformNormals()
    {
        warp_point_.reset(new WarpPointRigid6D<PointSource, PointTarget, MatScalar>);
    }
    ~MyTransformNormals() {}

    inline void
    estimateRigidTransformation(
        const pcl::PointCloud<PointSource> &cloud_src,
        const pcl::PointCloud<PointTarget> &cloud_tgt,
        Matrix4 &transformation_matrix) const override
    {
        PCL_DEBUG("call 1");
    }

    void
    estimateRigidTransformation(
        const pcl::PointCloud<PointSource> &cloud_src,
        const std::vector<int> &indices_src,
        const pcl::PointCloud<PointTarget> &cloud_tgt,
        Matrix4 &transformation_matrix) const override
    {
        PCL_DEBUG("call 2");
    }

    void
    estimateRigidTransformation(
        const pcl::PointCloud<PointSource> &cloud_src,
        const std::vector<int> &indices_src,
        const pcl::PointCloud<PointTarget> &cloud_tgt,
        const std::vector<int> &indices_tgt,
        Matrix4 &transformation_matrix) const override
    {
        PCL_DEBUG("call 3");
    }

    // Gauss newton / LM
    void estimateRigidTransformation(
        const pcl::PointCloud<PointSource> &cloud_src,
        const pcl::PointCloud<PointTarget> &cloud_tgt,
        const pcl::Correspondences &correspondences,
        Matrix4 &transformation_matrix) const override;

protected : 
typename WarpPointRigid<PointSource, PointTarget, MatScalar>::Ptr warp_point_;
const MyTransformNormals *estimator_;

public:
PCL_MAKE_ALIGNED_OPERATOR_NEW
};
