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

        Eigen::MatrixXf Jacobian(3, 6); // 3 x 6

        MatrixX Hessian(6, 6); // 6 x 3 x 3 x 6 -> 6 x 6

        VectorX Error(3);     // 3 x 1
        VectorX Residuals(6); // 6 x 1

        Hessian.setZero();
        Residuals.setZero();

        // Rotations
        Matrix3 Rx;
        Matrix3 Ry;
        Matrix3 Rz;

        // Derivatives
        Matrix3 Rx_;
        Matrix3 Ry_;
        Matrix3 Rz_;

        Matrix3 Rxyz;
        Matrix3 Rx_yz;
        Matrix3 Rxy_z;
        Matrix3 Rxyz_;

        parameters.setConstant(6, 0); // Init
        for (int i = 0; i < 3; ++i)
        {

            MatScalar c_ax = cos(parameters(3));
            MatScalar s_ax = sin(parameters(3));

            MatScalar c_ay = cos(parameters(4));
            MatScalar s_ay = sin(parameters(4));

            MatScalar c_az = cos(parameters(5));
            MatScalar s_az = sin(parameters(5));

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

            Vector3 translation(3);
            translation(0) = parameters(0);
            translation(1) = parameters(1);
            translation(2) = parameters(2);

            Rxyz = Rx*Ry*Rz;
            Rx_yz = Rx_ * Ry * Rz;
            Rxy_z = Rx * Ry_ * Rz;
            Rxyz_ = Rx * Ry * Rz_;

            for (int i = 0; i < n_pts; i++)
            {
                const int src_index = correspondences[i].index_query;
                const int tgt_index = correspondences[i].index_match;

                const PointSource &src_pt = cloud_src.points[src_index];
                const PointSource &tgt_pt = cloud_tgt.points[tgt_index];
                //  compute jacobian

                Error = Rxyz * src_pt.getVector3fMap() + translation - tgt_pt.getVector3fMap(); //p2p
                Jacobian.block<3, 3>(0, 0) = MatrixX::Identity(3, 3);
                Jacobian.block<3, 1>(0, 3) = Rx_yz * src_pt.getVector3fMap();
                Jacobian.block<3, 1>(0, 4) = Rxy_z * src_pt.getVector3fMap();
                Jacobian.block<3, 1>(0, 5) = Rxyz_ * src_pt.getVector3fMap();

                // Linearized
                // Error = src_pt.getVector3fMap() - tgt_pt.getVector3fMap(); //p2p
                // Jacobian.block<3, 3>(0, 0) = MatrixX::Identity(3, 3);
                // Jacobian.block<3, 1>(0, 3) = Vector3(src_pt.x, -src_pt.z, src_pt.y);
                // Jacobian.block<3, 1>(0, 4) = Vector3(src_pt.z, src_pt.y, -src_pt.x);
                // Jacobian.block<3, 1>(0, 5) = Vector3(-src_pt.y, src_pt.x, src_pt.z);

                Hessian += Jacobian.transpose() * Jacobian;
                Residuals += Jacobian.transpose() * Error;
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