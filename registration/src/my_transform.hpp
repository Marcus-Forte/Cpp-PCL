#pragma once

#include <pcl/registration/transformation_estimation.h>
#include <pcl/console/print.h>

//idea for Feature-ICP usage;
// icp.setSourceEdges
// icp.setTargetEdges
// icp.setSourcePlanes
// icp.setTargetPlanes
// icp.setTransformationEstimation(edge p2l + plane p2p)

// mt trabalho!

using namespace pcl::registration;

template <typename PointSource, typename PointTarget, typename Scalar = float>
class MyTransform : public TransformationEstimation<PointSource, PointTarget, Scalar>
{
public:
    using Ptr = pcl::shared_ptr<MyTransform<PointSource, PointTarget, Scalar>>;
    using ConstPtr = pcl::shared_ptr<const MyTransform<PointSource, PointTarget, Scalar>>;
    using Matrix4 = typename TransformationEstimation<PointSource, PointTarget, Scalar>::Matrix4;

    MyTransform() {}
    ~MyTransform() {}

    inline void
    estimateRigidTransformation(
        const pcl::PointCloud<PointSource> &cloud_src,
        const pcl::PointCloud<PointTarget> &cloud_tgt,
        Matrix4 &transformation_matrix) const override
    {
        PCL_INFO("call 1");
    }

    void
    estimateRigidTransformation(
        const pcl::PointCloud<PointSource> &cloud_src,
        const std::vector<int> &indices_src,
        const pcl::PointCloud<PointTarget> &cloud_tgt,
        Matrix4 &transformation_matrix) const override
    {
        PCL_INFO("call 2");
    }

    void
    estimateRigidTransformation(
        const pcl::PointCloud<PointSource> &cloud_src,
        const std::vector<int> &indices_src,
        const pcl::PointCloud<PointTarget> &cloud_tgt,
        const std::vector<int> &indices_tgt,
        Matrix4 &transformation_matrix) const override
    {
        PCL_INFO("call 3");
    }

    // Gauss newton
    void estimateRigidTransformation(
        const pcl::PointCloud<PointSource> &cloud_src,
        const pcl::PointCloud<PointTarget> &cloud_tgt,
        const pcl::Correspondences &correspondences,
        Matrix4 &transformation_matrix) const
    {
        size_t n_pts = correspondences.size();
        PCL_DEBUG("Correspondences: %d", n_pts);
        // Parameters : tx, ty, tz, ax, ay, az
        Eigen::VectorXf parameters(6);

        Eigen::MatrixXf Jacobian(3, 6);
        Eigen::VectorXf Error(3);

        Eigen::MatrixXf Hessian(6, 6);
        Eigen::VectorXf Residuals(6);

        Hessian = Eigen::MatrixXf::Zero(6, 6);
        Residuals = Eigen::VectorXf::Zero(6);

        parameters.setConstant(6, 0);
        float accelerator = 2;

        // Iterations
        // for (int i = 0; i < 5; ++i)
        // {
        for (int i = 0; i < n_pts; ++i)
        {
            int src_index = correspondences[i].index_query;
            int tgt_index = correspondences[i].index_match;
            //  compute jacobian
            // PCL_DEBUG("Computing Jacobian...");
            errorAndJacobian(parameters, cloud_src.points[src_index], cloud_tgt[tgt_index], Error, Jacobian);

            // Stack to hessian
            // PCL_DEBUG("Computing Hessian...");
            Hessian += Jacobian.transpose() * Jacobian;
            Residuals += Jacobian.transpose() * Error;
        }

        parameters -= Hessian.inverse() * Residuals * accelerator;
        std::cout << parameters << std::endl
                  << std::endl;

        // } // for end

        Eigen::AngleAxisf rollAngle(parameters(3), Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(parameters(4), Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(parameters(5), Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf q = rollAngle * pitchAngle * yawAngle;

        Eigen::Matrix3f rotMatrix = q.matrix();

        transformation_matrix << rotMatrix(0, 0), rotMatrix(0, 1), rotMatrix(0, 2), parameters(0),
            rotMatrix(1, 0), rotMatrix(1, 1), rotMatrix(1, 2), parameters(1),
            rotMatrix(2, 0), rotMatrix(2, 1), rotMatrix(2, 2), parameters(2),
            0, 0, 0, 1;
    }

private:
    inline void errorAndJacobian(const Eigen::VectorXf &parameters, const PointSource &src_pt, const PointTarget &tgt_pt, Eigen::VectorXf &error, Eigen::MatrixXf &Jacobian) const

    {
        // Translation
        Eigen::Vector3f translation(3);

        translation(0) = parameters(0);
        translation(1) = parameters(1);
        translation(2) = parameters(2);

        float c_ax = cos(parameters(3));
        float s_ax = sin(parameters(3));

        float c_ay = cos(parameters(4));
        float s_ay = sin(parameters(4));

        float c_az = cos(parameters(5));
        float s_az = sin(parameters(5));

        // Rotations
        Eigen::Matrix3f Rx = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f Ry = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f Rz = Eigen::Matrix3f::Identity();

        // Derivatives
        Eigen::Matrix3f Rx_ = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f Ry_ = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f Rz_ = Eigen::Matrix3f::Identity();

        Rx << 1, 0, 0,
            0, c_ax, -s_ax,
            0, s_ax, c_ax;

        Ry << c_ay, 0, s_ay,
            0, 1, 0,
            -s_ay, 0, c_ay;

        Rz << c_az, -s_az, 0,
            s_az, c_az, 0,
            0, 0, 1;

        Rx_ << 1, 0, 0,
            0, -s_ax, -c_ax,
            0, c_ax, -s_ax;

        Ry_ << -s_ay, 0, c_ay,
            0, 1, 0,
            -c_ay, 0, -s_ay;

        Rz_ << -s_az, -c_az, 0,
            c_az, -s_az, 0,
            0, 0, 1;

        // 3x1
        error = Rx * Ry * Rz * src_pt.getVector3fMap() + translation - tgt_pt.getVector3fMap();

        Jacobian.block<3, 3>(0, 0) = Eigen::MatrixXf::Identity(3, 3);
        Jacobian.block<3, 1>(0, 3) = Rx_ * Ry * Rz * src_pt.getVector3fMap();
        Jacobian.block<3, 1>(0, 4) = Rx * Ry_ * Rz * src_pt.getVector3fMap();
        Jacobian.block<3, 1>(0, 5) = Rx * Ry * Rz_ * src_pt.getVector3fMap();
    }
};
