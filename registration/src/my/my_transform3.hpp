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

    using PointCloudSource = pcl::PointCloud<PointSource>;
    using PointCloudSourcePtr = typename PointCloudSource::Ptr;
    using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

    using PointCloudTarget = pcl::PointCloud<PointTarget>;

    using PointIndicesPtr = pcl::PointIndices::Ptr;
    using PointIndicesConstPtr = pcl::PointIndices::ConstPtr;

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

        Eigen::MatrixXf Jacobian(n_pts, 6); // 3 x 6

        MatrixX Hessian(6, 6); // 6 x 3 x 3 x 6 -> 6 x 6

        VectorX NError(n_pts);
        VectorX Residuals(6); // 6 x 1

        Hessian.setZero();
        Residuals.setZero();

        Matrix3 Rxyz;
        std::vector<int> indices_src(n_pts);
        std::vector<int> indices_tgt(n_pts);
        for (int i = 0; i < n_pts; ++i)
        {
            indices_src[i] = correspondences[i].index_query;
            indices_tgt[i] = correspondences[i].index_match;
            NError[i] = correspondences[i].distance;
        }

        // Set temporary pointers
        tmp_src_ = &cloud_src;
        tmp_tgt_ = &cloud_tgt;
        tmp_idx_src_ = &indices_src;
        tmp_idx_tgt_ = &indices_tgt;

        parameters.setConstant(6, 0); // Init
        Functor<MatScalar> functor(static_cast<int>(n_pts), this);
        Eigen::NumericalDiff<Functor<MatScalar>> num_diff(functor);

        for (int i = 0; i < 8; ++i)
        {

            num_diff.df(parameters, Jacobian);

            MatScalar damping = 2;
            Hessian = Jacobian.transpose() * Jacobian; // Fitness: 0.006057
            MatrixX diagonal = damping * Hessian.diagonal().asDiagonal();
            Hessian = Hessian + diagonal;
            Residuals = Jacobian.transpose() * NError;

            parameters -= Hessian.inverse() * Residuals;

        } // Gauss Newton Iteration for

        PCL_DEBUG("GN Solver : %f %f %f %f %f %f\n", parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], parameters[5]);
        warp_point_->setParam(parameters);
        transformation_matrix = warp_point_->getTransform();
    }

protected:
    typename WarpPointRigid<PointSource, PointTarget, MatScalar>::Ptr warp_point_;
    /** \brief Temporary pointer to the source dataset. */
    mutable const PointCloudSource *tmp_src_;

    /** \brief Temporary pointer to the target dataset. */
    mutable const PointCloudTarget *tmp_tgt_;

    /** \brief Temporary pointer to the source dataset indices. */
    mutable const std::vector<int> *tmp_idx_src_;

    /** \brief Temporary pointer to the target dataset indices. */
    mutable const std::vector<int> *tmp_idx_tgt_;
    const MyTransform *estimator_;

    MatScalar computeDistance(const Vector4 &p_src, const PointTarget &p_tgt) const
    {
        Vector4 t(p_tgt.x, p_tgt.y, p_tgt.z, 0);
        return ((p_src - t).norm());
    }

    template <typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
    struct Functor
    {
        using Scalar = _Scalar;
        enum
        {
            InputsAtCompileTime = NX,
            ValuesAtCompileTime = NY
        };
        using InputType = Eigen::Matrix<_Scalar, InputsAtCompileTime, 1>;
        using ValueType = Eigen::Matrix<_Scalar, ValuesAtCompileTime, 1>;
        using JacobianType = Eigen::Matrix<_Scalar, ValuesAtCompileTime, InputsAtCompileTime>;

        /** \brief Empty Constructor. */
        Functor() : m_data_points_(ValuesAtCompileTime) {}

        /** \brief Constructor
            * \param[in] m_data_points number of data points to evaluate.
            */
        Functor(int m_data_points, const MyTransform *estimator) : m_data_points_(m_data_points), estimator_(estimator) {}

        /** \brief Destructor. */
        virtual ~Functor() {}

        /** \brief Get the number of values. */
        int
        values() const { return (m_data_points_); }

        int
        operator()(const VectorX &x, VectorX &fvec) const
        {
            const pcl::PointCloud<PointSource> &src_points = *estimator_->tmp_src_;
            const pcl::PointCloud<PointTarget> &tgt_points = *estimator_->tmp_tgt_;
            const std::vector<int> &src_indices = *estimator_->tmp_idx_src_;
            const std::vector<int> &tgt_indices = *estimator_->tmp_idx_tgt_;

            // Initialize the warp function with the given parameters
            estimator_->warp_point_->setParam(x);

            // Transform each source point and compute its distance to the corresponding target point
            for (int i = 0; i < values(); ++i)
            {
                const PointSource &p_src = src_points.points[src_indices[i]];
                const PointTarget &p_tgt = tgt_points.points[tgt_indices[i]];

                // Transform the source point based on the current warp parameters
                Vector4 p_src_warped;
                estimator_->warp_point_->warpPoint(p_src, p_src_warped);

                // Estimate the distance (cost function)
                fvec[i] = estimator_->computeDistance(p_src_warped, p_tgt);
            }
            return (0);
        }

    protected:
        int m_data_points_;
        const MyTransform<PointSource, PointTarget, MatScalar> *estimator_;
    };
};

// error[0] = (Rx * Ry * Rz * src_pt.getVector3fMap() + translation - tgt_pt.getVector3fMap()).dot(tgt_pt.getNormalVector3fMap()); //p2p
// Jacobian_.block<3, 3>(0, 0) = Eigen::MatrixXf::Identity(3, 3);
// Jacobian_.block<3, 1>(0, 3) = Rx_ * Ry * Rz * src_pt.getVector3fMap();
// Jacobian_.block<3, 1>(0, 4) = Rx * Ry_ * Rz * src_pt.getVector3fMap();
// Jacobian_.block<3, 1>(0, 5) = Rx * Ry * Rz_ * src_pt.getVector3fMap();

// Jacobian = Jacobian_.block<1, 6>(0, 0) * tgt_pt.normal_x +
//            Jacobian_.block<1, 6>(1, 0) * tgt_pt.normal_y +
//            Jacobian_.block<1, 6>(2, 0) * tgt_pt.normal_z;