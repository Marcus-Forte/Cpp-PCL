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
class MyTransform : public TransformationEstimation<PointSource, PointTarget, MatScalar>
{
public:
    using Ptr = pcl::shared_ptr<MyTransform<PointSource, PointTarget, MatScalar>>;
    using ConstPtr = pcl::shared_ptr<const MyTransform<PointSource, PointTarget, MatScalar>>;
    using VectorX = Eigen::Matrix<MatScalar, Eigen::Dynamic, 1>;
    using MatrixX = Eigen::Matrix<MatScalar, Eigen::Dynamic, Eigen::Dynamic>;
    using Vector3 = Eigen::Matrix<MatScalar, 3, 1>;
    using Vector4 = Eigen::Matrix<MatScalar, 4, 1>;
    using Matrix3 = Eigen::Matrix<MatScalar, 3, 3>;
    using Matrix4 = typename TransformationEstimation<PointSource, PointTarget, MatScalar>::Matrix4;

    MyTransform()
    {
        warp_point_.reset(new WarpPointRigid6D<PointSource, PointTarget, MatScalar>);
    }
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
        PCL_DEBUG("Correspondences: %d\n", n_pts);
        // Parameters : tx, ty, tz, ax, ay, az
        VectorX parameters(6);

        Eigen::MatrixXf Jacobian(1, 6); // 3 x 6

        MatrixX Hessian(6, 6); // 6 x 3 x 3 x 6 -> 6 x 6

        MatScalar Error;      // 3 x 1
        VectorX Residuals(6); // 6 x 1

        Hessian.setZero();
        Residuals.setZero();

        Matrix3 Rxyz;

        parameters.setConstant(6, 0); // Init
        for (int i = 0; i < 10; ++i)
        {
            Vector3 translation;
            translation[0] = parameters[0];
            translation[1] = parameters[1];
            translation[2] = parameters[2];

            MatScalar alpha = parameters[3];
            MatScalar beta = parameters[4];
            MatScalar gamma = parameters[5];

            Rxyz << 1, alpha*beta-gamma, alpha*gamma+beta,
                gamma, alpha*beta*gamma+1, beta*gamma-alpha,
                -beta, alpha, 1;



            for (int i = 0; i < n_pts; i++)
            {
                const int src_index = correspondences[i].index_query;
                const int tgt_index = correspondences[i].index_match;

                const PointSource &src_pt = cloud_src.points[src_index];
                const PointSource &tgt_pt = cloud_tgt.points[tgt_index];
                //  compute jacobian
                VectorX error = Rxyz * src_pt.getVector3fMap() + translation - tgt_pt.getVector3fMap(); //p2p
                Error = error[0]*error[0] + error[1] * error[1] * error[2] * error[2];
                Jacobian(0, 0) = 2 * error[0];
                Jacobian(0, 1) = 2 * error[1];
                Jacobian(0, 2) = 2 * error[2];

                Jacobian(0, 3) = 2 * error[0] * (beta * src_pt.y + gamma * src_pt.z) + 
                                 2 * error[1] * (beta*gamma*src_pt.y - src_pt.z) +
                                 2 * error[2] * (src_pt.y);

                Jacobian(0, 4) = 2 * error[0] * (alpha * src_pt.y + src_pt.z) + 
                                 2 * error[1] * (alpha*gamma*src_pt.y + gamma*src_pt.z) +
                                 2 * error[2] * (-src_pt.x);

                Jacobian(0, 5) = 2 * error[0] * (-src_pt.y + alpha * src_pt.z) + 
                                 2 * error[1] * (alpha*beta*src_pt.y + beta*src_pt.z);

                Jacobian = Jacobian * 0.5 / sqrt(Error);
                                 

                Hessian += Jacobian.transpose() * Jacobian;
                Residuals += Jacobian.transpose() * sqrt(Error);
            }

            parameters -= Hessian.colPivHouseholderQr().solve(Residuals);

        } // Gauss Newton Iteration for

        PCL_DEBUG("GN Solver : %f %f %f %f %f %f\n", parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], parameters[5]);
        warp_point_->setParam(parameters);
        transformation_matrix = warp_point_->getTransform();
    }

protected:
    typename WarpPointRigid<PointSource, PointTarget, MatScalar>::Ptr warp_point_;
    const MyTransform *estimator_;
};

// error[0] = (Rx * Ry * Rz * src_pt.getVector3fMap() + translation - tgt_pt.getVector3fMap()).dot(tgt_pt.getNormalVector3fMap()); //p2p
// Jacobian_.block<3, 3>(0, 0) = Eigen::MatrixXf::Identity(3, 3);
// Jacobian_.block<3, 1>(0, 3) = Rx_ * Ry * Rz * src_pt.getVector3fMap();
// Jacobian_.block<3, 1>(0, 4) = Rx * Ry_ * Rz * src_pt.getVector3fMap();
// Jacobian_.block<3, 1>(0, 5) = Rx * Ry * Rz_ * src_pt.getVector3fMap();

// Jacobian = Jacobian_.block<1, 6>(0, 0) * tgt_pt.normal_x +
//            Jacobian_.block<1, 6>(1, 0) * tgt_pt.normal_y +
//            Jacobian_.block<1, 6>(2, 0) * tgt_pt.normal_z;