#include "feature_registration.h"
#include "opencv2/opencv.hpp" // TODO mudar

#include <pcl/visualization/pcl_visualizer.h>

namespace duna
{

    template <typename PointT, typename Scalar>
    void Registration<PointT, Scalar>::align()
    {
        align(Matrix4::Identity());
    }

    template <typename PointT, typename Scalar>
    void Registration<PointT, Scalar>::align(const Matrix4 &guess)
    {
        // optimization
        PCL_DEBUG("input corners: %d\n", input_corners->size());
        PCL_DEBUG("input surfaces: %d\n", input_surfaces->size());

        PCL_DEBUG("target corners: %d\n", target_corners->size());
        PCL_DEBUG("target surfaces: %d\n", target_surfaces->size());

        if (!KdTree_surf_ok || !KdTree_corn_ok)
        {
            PCL_DEBUG("No KDTree found. Recomputing...\n");
            surf_tree->setInputCloud(target_surfaces);
            corner_tree->setInputCloud(target_corners);
        }

        pcl::Indices corners_correspondences_indices;
        pcl::Indices surfaces_correspondences_indices;

        std::vector<float> corners_correspondences_dists;
        std::vector<float> surfaces_correspondences_dists;

        PointCloudTPtr transformed_corners(new PointCloudT);
        PointCloudTPtr transformed_surfaces(new PointCloudT);

        // pcl::visualization::PCLVisualizer viewer("registration");

        final_transformation_ = guess;

        // If the guessed transformation is non identity
        if (guess != Matrix4::Identity())
        {
            transformed_corners->resize(input_corners->size());
            transformed_surfaces->resize(input_surfaces->size());

            // Apply guessed transformation prior to search for neighbours
            pcl::transformPointCloud(*input_corners, *transformed_corners, guess);
            pcl::transformPointCloud(*input_surfaces, *transformed_surfaces, guess);
            // transformCloud(*input_, *input_transformed, guess);
        }
        else
        {
            *transformed_corners = *input_corners;
            *transformed_surfaces = *input_surfaces;
        }

        transformation_ = Matrix4::Identity();

        // viewer.addPointCloud<PointT>(target_surfaces, "target");
        // viewer.addPointCloud<PointT>(transformed_surfaces, "tf_surface");

        nr_iterations_ = 0;
        converged_ = false;

        convergence_criteria_->setMaximumIterations(max_iterations_);
        convergence_criteria_->setRelativeMSE(euclidean_fitness_epsilon_);
        convergence_criteria_->setTranslationThreshold(transformation_epsilon_);
        if (transformation_rotation_epsilon_ > 0)
            convergence_criteria_->setRotationThreshold(transformation_rotation_epsilon_);
        else
            convergence_criteria_->setRotationThreshold(1.0 - transformation_epsilon_);

        // TODO registration loop
        do
        {

            // viewer.updatePointCloud<PointT>(transformed_surfaces, "tf_surface");
            // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "tf_surface");
            // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tf_surface");

            // TODO Find Correspondences

            // for (int j = 0; j < transformed_corners.size(); ++j)
            // {
            //     const PointT &pointSel = transformed_corners.points[j];
            //     corner_tree->nearestKSearch(pointSel, corner_neighboors, *corners_correspondences, corners_correspondences_dists);
            // }

            surfaces_correspondences->clear();
            // viewer.removeAllShapes();
            for (int j = 0; j < transformed_surfaces->size(); ++j)
            {
                const PointT &pointSel = transformed_surfaces->points[j];
                surf_tree->nearestKSearch(pointSel, kn_surface_neighboors, surfaces_correspondences_indices, surfaces_correspondences_dists);
                if (surfaces_correspondences_dists[0] > max_corr_dist)
                {
                    continue;
                }
                pcl::Correspondence corr;
                corr.index_query = j;
                corr.index_match = surfaces_correspondences_indices[0];
                corr.distance = surfaces_correspondences_dists[0];
                surfaces_correspondences->push_back(corr);

                // source_corrs->push_back(pointSel);
                // target_corrs->push_back(target_surfaces->points[(*surfaces_correspondences)[0]]);

                Eigen::Vector4f normal;
                float unused;
                const PointT &probe = target_surfaces->points[surfaces_correspondences_indices[0]];
                if (probe.normal_x == 0 && probe.normal_y == 0 && probe.normal_z == 0)
                {
                    // PCL_DEBUG("Updating normals ");
                    pcl::computePointNormal(*target_surfaces, surfaces_correspondences_indices, normal, unused);
                    // PCL_DEBUG("ok!-> %f %f %f\n",normal[0],normal[1],normal[2]);

                    target_sufraces_modifiable->points[surfaces_correspondences_indices[0]].normal_x = normal[0];
                    target_sufraces_modifiable->points[surfaces_correspondences_indices[0]].normal_y = normal[1];
                    target_sufraces_modifiable->points[surfaces_correspondences_indices[0]].normal_z = normal[2];
                }

                // viewer.addLine<PointT, PointT>(pointSel, target_surfaces->points[surfaces_correspondences_indices[0]], 1, 0, 0, "arrow" + std::to_string(j));
            }
            // viewer.removePointCloud("surface_normals");
            // viewer.addPointCloudNormals<PointT>(target_surfaces, 1, 0.1, "surface_normals");
            // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "surface_normals");
            // viewer.spin();

            int n_pts = surfaces_correspondences->size();

            if (n_pts < min_number_correspondences_)
            {
                PCL_ERROR("[pcl::%s::computeTransformation] Not enough correspondences found. Relax your threshold parameters.\n", "duna_matching");
                convergence_criteria_->setConvergenceState(pcl::registration::DefaultConvergenceCriteria<Scalar>::CONVERGENCE_CRITERIA_NO_CORRESPONDENCES);
                converged_ = false;
                break;
            }

            // TODO optimization loop
            // x y z ax ay az
            VectorX parameters(6);

            Eigen::MatrixXf Jacobian(n_pts, 6); // n x 6
            MatrixX Hessian(6, 6);              // 6 x 3 x 3 x 6 -> 6 x 6

            VectorX NError(n_pts);
            VectorX Residuals(6); // 6 x 1

            // Rotations
            Matrix3 Rx;
            Matrix3 Ry;
            Matrix3 Rz;

            // Derivatives
            Matrix3 Rx_;
            Matrix3 Ry_;
            Matrix3 Rz_;

            // Compositions
            Matrix3 Rxyz;
            Matrix3 Rx_yz;
            Matrix3 Rxy_z;
            Matrix3 Rxyz_;

            Scalar damping = 0.5;

            parameters.setConstant(6, 0); // Init
            Hessian.setZero();
            Residuals.setZero();

            for (int j = 0; j < 3; ++j)
            {

                Vector3 translation(parameters(0), parameters(1), parameters(2));

                Scalar c_alpha = std::cos(parameters(3));
                Scalar s_alpha = std::sin(parameters(3));

                Scalar c_beta = std::cos(parameters(4));
                Scalar s_beta = std::sin(parameters(4));

                Scalar c_gamma = std::cos(parameters(5));
                Scalar s_gamma = std::sin(parameters(5));

                Rx << 1, 0, 0,
                    0, c_alpha, -s_alpha,
                    0, s_alpha, c_alpha;

                Ry << c_beta, 0, s_beta,
                    0, 1, 0,
                    -s_beta, 0, c_beta;

                Rz << c_gamma, -s_gamma, 0,
                    s_gamma, c_gamma, 0,
                    0, 0, 1;

                Rx_ << 0, 0, 0,
                    0, -s_alpha, -c_alpha,
                    0, c_alpha, -s_alpha;

                Ry_ << -s_beta, 0, c_beta,
                    0, 0, 0,
                    -c_beta, 0, -s_beta;

                Rz_ << -s_gamma, -c_gamma, 0,
                    c_gamma, -s_gamma, 0,
                    0, 0, 0;

                Rxyz = Rz * Ry * Rx;
                Rx_yz = Rz * Ry * Rx_;
                Rxy_z = Rz * Ry_ * Rx;
                Rxyz_ = Rz_ * Ry * Rx;

                PCL_DEBUG("Surf corrs: %d\n", n_pts);
                for (int i = 0; i < n_pts; i++)
                {

                    const int src_index = (*surfaces_correspondences)[i].index_query;
                    const int tgt_index = (*surfaces_correspondences)[i].index_match;

                    // PCL_DEBUG("src :%d -> %d: tgt d: %f\n", src_index, tgt_index, (*correspondences)[i].distance);

                    const PointT &src_pt = transformed_surfaces->points[src_index];
                    const PointT &tgt_pt = target_surfaces->points[tgt_index];
                    //  compute jacobian
                    // std::cout << "SRC <--> TGT: " << src_pt << " <--> " << tgt_pt << std::endl;

                    VectorX Error = Rxyz * src_pt.getVector3fMap() + translation - tgt_pt.getVector3fMap(); //p2p
                    Scalar error = Error.dot(tgt_pt.getNormalVector3fMap());

                    Vector3 jac_alpha = Rx_yz * src_pt.getVector3fMap();
                    Vector3 jac_beta = Rxy_z * src_pt.getVector3fMap();
                    Vector3 jac_gamma = Rxyz_ * src_pt.getVector3fMap();

                    // Jacobian(0, 0) = Error[0]; // 2 * e1
                    // Jacobian(0, 1) = Error[1]; // 2 * e2
                    // Jacobian(0, 2) = Error[2]; // 2 * e3

                    // Jacobian(0, 3) = Error[0] * jac_alpha[0] + Error[1] * jac_alpha[1] + Error[2] * jac_alpha[2]; // alpha
                    // Jacobian(0, 4) = Error[0] * jac_beta[0] + Error[1] * jac_beta[1] + Error[2] * jac_beta[2];    // beta
                    // Jacobian(0, 5) = Error[0] * jac_gamma[0] + Error[1] * jac_gamma[1] + Error[2] * jac_gamma[2]; // gamma

                    // Stack jacobians
                    Jacobian(i, 0) = tgt_pt.normal_x; // 2 * e1
                    Jacobian(i, 1) = tgt_pt.normal_y; // 2 * e2
                    Jacobian(i, 2) = tgt_pt.normal_z; // 2 * e3

                    Jacobian(i, 3) = (tgt_pt.normal_x * jac_alpha[0] + tgt_pt.normal_y * jac_alpha[1] + tgt_pt.normal_z * jac_alpha[2]); // alpha
                    Jacobian(i, 4) = (tgt_pt.normal_x * jac_beta[0] + tgt_pt.normal_y * jac_beta[1] + tgt_pt.normal_z * jac_beta[2]);    // beta
                    Jacobian(i, 5) = (tgt_pt.normal_x * jac_gamma[0] + tgt_pt.normal_y * jac_gamma[1] + tgt_pt.normal_z * jac_gamma[2]); // gamma
                    NError[i] = error;

                    // Jacobian = Jacobian / error;

                    // Hessian += Jacobian.transpose() * Jacobian; // 6 x 6 0.006197
                    // Residuals += Jacobian.transpose() * error;
                }

                damping *= 1.2;
                Hessian = Jacobian.transpose() * Jacobian; //
                MatrixX diagonal = damping * Hessian.diagonal().asDiagonal();
                Hessian = Hessian + diagonal;
                Residuals = Jacobian.transpose() * NError;

                parameters -= Hessian.inverse() * Residuals;

                PCL_DEBUG("GN Solver (%f damping) : %f %f %f %f %f %f\n", damping, parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], parameters[5]);

            } // End OPT LOOP

            transformation_.setZero();
            transformation_(0, 3) = parameters[0];
            transformation_(1, 3) = parameters[1];
            transformation_(2, 3) = parameters[2];
            transformation_(3, 3) = 1;

            // Compute w from the unit quaternion
            Eigen::Quaternion<Scalar> q(0, parameters[3], parameters[4], parameters[5]);
            q.w() = static_cast<Scalar>(std::sqrt(1 - q.dot(q)));
            q.normalize();
            transformation_.topLeftCorner(3, 3) = q.toRotationMatrix();

            // Transform the data
            pcl::transformPointCloud(*transformed_surfaces, *transformed_surfaces, transformation_);
            // transformCloud(*input_transformed, *input_transformed, transformation_);

            // Obtain the final transformation
            final_transformation_ = transformation_ * final_transformation_;

            nr_iterations_++;

            converged_ = static_cast<bool>((*convergence_criteria_));
        } while (convergence_criteria_->getConvergenceState() == pcl::registration::DefaultConvergenceCriteria<Scalar>::CONVERGENCE_CRITERIA_NOT_CONVERGED);
    }

    // TODO usar PCL, e nao OPENCV
    template <typename PointT>
    static bool plane_judge(const std::vector<PointT> &point_list, const int plane_threshold)
    {
        static cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
        static cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
        static cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

        int num = point_list.size();
        float cx = 0;
        float cy = 0;
        float cz = 0;
        for (int j = 0; j < num; j++)
        {
            cx += point_list[j].x;
            cy += point_list[j].y;
            cz += point_list[j].z;
        }
        cx /= num;
        cy /= num;
        cz /= num;
        //mean square error
        float a11 = 0;
        float a12 = 0;
        float a13 = 0;
        float a22 = 0;
        float a23 = 0;
        float a33 = 0;
        for (int j = 0; j < num; j++)
        {
            float ax = point_list[j].x - cx;
            float ay = point_list[j].y - cy;
            float az = point_list[j].z - cz;

            a11 += ax * ax;
            a12 += ax * ay;
            a13 += ax * az;
            a22 += ay * ay;
            a23 += ay * az;
            a33 += az * az;
        }
        a11 /= num;
        a12 /= num;
        a13 /= num;
        a22 /= num;
        a23 /= num;
        a33 /= num;

        matA1.at<float>(0, 0) = a11;
        matA1.at<float>(0, 1) = a12;
        matA1.at<float>(0, 2) = a13;
        matA1.at<float>(1, 0) = a12;
        matA1.at<float>(1, 1) = a22;
        matA1.at<float>(1, 2) = a23;
        matA1.at<float>(2, 0) = a13;
        matA1.at<float>(2, 1) = a23;
        matA1.at<float>(2, 2) = a33;

        cv::eigen(matA1, matD1, matV1);
        if (matD1.at<float>(0, 0) > plane_threshold * matD1.at<float>(0, 1))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    template <typename PointT, typename Scalar>
    void Registration<PointT, Scalar>::extractFeatures(const PointCloudT &laserCloudIn, PointCloudT &cornerPointsSharp, PointCloudT &surfPointsFlat)
    {
        int cloudSize = laserCloudIn.points.size();
        static int CloudFeatureFlag[32000];
        int scanID;

        if (cloudSize > 32000)
            cloudSize = 32000;

        int count = cloudSize;
        PointT point;
        pcl::PointCloud<PointT> Allpoints;

        for (int i = 0; i < cloudSize; i++)
        {

            point.x = laserCloudIn.points[i].x;
            point.y = laserCloudIn.points[i].y;
            point.z = laserCloudIn.points[i].z;
            double theta = std::atan2(laserCloudIn.points[i].y, laserCloudIn.points[i].z) / M_PI * 180 + 180;

            scanID = std::floor(theta / 9);
            float dis = point.x * point.x + point.y * point.y + point.z * point.z;

            double dis2 = laserCloudIn.points[i].z * laserCloudIn.points[i].z + laserCloudIn.points[i].y * laserCloudIn.points[i].y;
            double theta2 = std::asin(sqrt(dis2 / dis)) / M_PI * 180;

            point.intensity = scanID + (laserCloudIn.points[i].intensity / 10000);
            //point.intensity = scanID+(double(i)/cloudSize);

            if (!std::isfinite(point.x) ||
                !std::isfinite(point.y) ||
                !std::isfinite(point.z))
            {
                continue;
            }

            Allpoints.push_back(point);
        }

        typename pcl::PointCloud<PointT>::Ptr laserCloud(new pcl::PointCloud<PointT>());
        *laserCloud += Allpoints;
        cloudSize = laserCloud->size();

        for (int i = 0; i < cloudSize; i++)
        {
            CloudFeatureFlag[i] = 0;
        }

        //
        cornerPointsSharp.clear();
        surfPointsFlat.clear();
        // pcl::PointCloud<PointT> cornerPointsSharp;

        // pcl::PointCloud<PointT> surfPointsFlat;

        pcl::PointCloud<PointT> laserCloudFull;

        int debugnum1 = 0;
        int debugnum2 = 0;
        int debugnum3 = 0;
        int debugnum4 = 0;
        int debugnum5 = 0;

        int count_num = 1;
        bool left_surf_flag = false;
        bool right_surf_flag = false;
        Eigen::Vector3d surf_vector_current(0, 0, 0);
        Eigen::Vector3d surf_vector_last(0, 0, 0);
        int last_surf_position = 0;
        double depth_threshold = 0.1;
        //********************************************************************************************************************************************
        for (int i = 5; i < cloudSize - 5; i += count_num)
        {
            float depth = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                               laserCloud->points[i].y * laserCloud->points[i].y +
                               laserCloud->points[i].z * laserCloud->points[i].z);

            // if(depth < 2) depth_threshold = 0.05;
            // if(depth > 30) depth_threshold = 0.1;
            //left curvature
            float ldiffX =
                laserCloud->points[i - 4].x + laserCloud->points[i - 3].x - 4 * laserCloud->points[i - 2].x + laserCloud->points[i - 1].x + laserCloud->points[i].x;

            float ldiffY =
                laserCloud->points[i - 4].y + laserCloud->points[i - 3].y - 4 * laserCloud->points[i - 2].y + laserCloud->points[i - 1].y + laserCloud->points[i].y;

            float ldiffZ =
                laserCloud->points[i - 4].z + laserCloud->points[i - 3].z - 4 * laserCloud->points[i - 2].z + laserCloud->points[i - 1].z + laserCloud->points[i].z;

            float left_curvature = ldiffX * ldiffX + ldiffY * ldiffY + ldiffZ * ldiffZ;

            if (left_curvature < 0.01)
            {

                std::vector<PointT> left_list;

                for (int j = -4; j < 0; j++)
                {
                    left_list.push_back(laserCloud->points[i + j]);
                }

                if (left_curvature < 0.001)
                    CloudFeatureFlag[i - 2] = 1; //surf point flag  && plane_judge(left_list,1000)

                left_surf_flag = true;
            }
            else
            {
                left_surf_flag = false;
            }

            //right curvature
            float rdiffX =
                laserCloud->points[i + 4].x + laserCloud->points[i + 3].x - 4 * laserCloud->points[i + 2].x + laserCloud->points[i + 1].x + laserCloud->points[i].x;

            float rdiffY =
                laserCloud->points[i + 4].y + laserCloud->points[i + 3].y - 4 * laserCloud->points[i + 2].y + laserCloud->points[i + 1].y + laserCloud->points[i].y;

            float rdiffZ =
                laserCloud->points[i + 4].z + laserCloud->points[i + 3].z - 4 * laserCloud->points[i + 2].z + laserCloud->points[i + 1].z + laserCloud->points[i].z;

            float right_curvature = rdiffX * rdiffX + rdiffY * rdiffY + rdiffZ * rdiffZ;

            if (right_curvature < 0.01)
            {
                std::vector<PointT> right_list;

                for (int j = 1; j < 5; j++)
                {
                    right_list.push_back(laserCloud->points[i + j]);
                }
                if (right_curvature < 0.001)
                    CloudFeatureFlag[i + 2] = 1; //surf point flag  && plane_judge(right_list,1000)

                count_num = 4;
                right_surf_flag = true;
            }
            else
            {
                count_num = 1;
                right_surf_flag = false;
            }

            //surf-surf corner feature
            if (left_surf_flag && right_surf_flag)
            {
                debugnum4++;

                Eigen::Vector3d norm_left(0, 0, 0);
                Eigen::Vector3d norm_right(0, 0, 0);
                for (int k = 1; k < 5; k++)
                {
                    Eigen::Vector3d tmp = Eigen::Vector3d(laserCloud->points[i - k].x - laserCloud->points[i].x,
                                                          laserCloud->points[i - k].y - laserCloud->points[i].y,
                                                          laserCloud->points[i - k].z - laserCloud->points[i].z);
                    tmp.normalize();
                    norm_left += (k / 10.0) * tmp;
                }
                for (int k = 1; k < 5; k++)
                {
                    Eigen::Vector3d tmp = Eigen::Vector3d(laserCloud->points[i + k].x - laserCloud->points[i].x,
                                                          laserCloud->points[i + k].y - laserCloud->points[i].y,
                                                          laserCloud->points[i + k].z - laserCloud->points[i].z);
                    tmp.normalize();
                    norm_right += (k / 10.0) * tmp;
                }

                //calculate the angle between this group and the previous group
                double cc = fabs(norm_left.dot(norm_right) / (norm_left.norm() * norm_right.norm()));
                //calculate the maximum distance, the distance cannot be too small
                Eigen::Vector3d last_tmp = Eigen::Vector3d(laserCloud->points[i - 4].x - laserCloud->points[i].x,
                                                           laserCloud->points[i - 4].y - laserCloud->points[i].y,
                                                           laserCloud->points[i - 4].z - laserCloud->points[i].z);
                Eigen::Vector3d current_tmp = Eigen::Vector3d(laserCloud->points[i + 4].x - laserCloud->points[i].x,
                                                              laserCloud->points[i + 4].y - laserCloud->points[i].y,
                                                              laserCloud->points[i + 4].z - laserCloud->points[i].z);
                double last_dis = last_tmp.norm();
                double current_dis = current_tmp.norm();

                if (cc < 0.5 && last_dis > 0.05 && current_dis > 0.05)
                { //
                    debugnum5++;
                    CloudFeatureFlag[i] = 150;
                }
            }
        }
        for (int i = 5; i < cloudSize - 5; i++)
        {
            float diff_left[2];
            float diff_right[2];
            float depth = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                               laserCloud->points[i].y * laserCloud->points[i].y +
                               laserCloud->points[i].z * laserCloud->points[i].z);

            for (int count = 1; count < 3; count++)
            {
                float diffX1 = laserCloud->points[i + count].x - laserCloud->points[i].x;
                float diffY1 = laserCloud->points[i + count].y - laserCloud->points[i].y;
                float diffZ1 = laserCloud->points[i + count].z - laserCloud->points[i].z;
                diff_right[count - 1] = sqrt(diffX1 * diffX1 + diffY1 * diffY1 + diffZ1 * diffZ1);

                float diffX2 = laserCloud->points[i - count].x - laserCloud->points[i].x;
                float diffY2 = laserCloud->points[i - count].y - laserCloud->points[i].y;
                float diffZ2 = laserCloud->points[i - count].z - laserCloud->points[i].z;
                diff_left[count - 1] = sqrt(diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2);
            }

            float depth_right = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
                                     laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                                     laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);
            float depth_left = sqrt(laserCloud->points[i - 1].x * laserCloud->points[i - 1].x +
                                    laserCloud->points[i - 1].y * laserCloud->points[i - 1].y +
                                    laserCloud->points[i - 1].z * laserCloud->points[i - 1].z);

            //outliers
            if ((diff_right[0] > 0.1 * depth && diff_left[0] > 0.1 * depth))
            {
                debugnum1++;
                CloudFeatureFlag[i] = 250;
                continue;
            }

            //break points
            if (fabs(diff_right[0] - diff_left[0]) > 0.1)
            {
                if (diff_right[0] > diff_left[0])
                {

                    Eigen::Vector3d surf_vector = Eigen::Vector3d(laserCloud->points[i - 4].x - laserCloud->points[i].x,
                                                                  laserCloud->points[i - 4].y - laserCloud->points[i].y,
                                                                  laserCloud->points[i - 4].z - laserCloud->points[i].z);
                    Eigen::Vector3d lidar_vector = Eigen::Vector3d(laserCloud->points[i].x,
                                                                   laserCloud->points[i].y,
                                                                   laserCloud->points[i].z);
                    double left_surf_dis = surf_vector.norm();
                    //calculate the angle between the laser direction and the surface
                    double cc = fabs(surf_vector.dot(lidar_vector) / (surf_vector.norm() * lidar_vector.norm()));

                    std::vector<PointT> left_list;
                    double min_dis = 10000;
                    double max_dis = 0;
                    for (int j = 0; j < 4; j++)
                    { //TODO: change the plane window size and add thin rod support
                        left_list.push_back(laserCloud->points[i - j]);
                        Eigen::Vector3d temp_vector = Eigen::Vector3d(laserCloud->points[i - j].x - laserCloud->points[i - j - 1].x,
                                                                      laserCloud->points[i - j].y - laserCloud->points[i - j - 1].y,
                                                                      laserCloud->points[i - j].z - laserCloud->points[i - j - 1].z);

                        if (j == 3)
                            break;
                        double temp_dis = temp_vector.norm();
                        if (temp_dis < min_dis)
                            min_dis = temp_dis;
                        if (temp_dis > max_dis)
                            max_dis = temp_dis;
                    }
                    bool left_is_plane = plane_judge(left_list, 100);

                    if (left_is_plane && (max_dis < 2 * min_dis) && left_surf_dis < 0.05 * depth && cc < 0.8)
                    { //
                        if (depth_right > depth_left)
                        {
                            CloudFeatureFlag[i] = 100;
                        }
                        else
                        {
                            if (depth_right == 0)
                                CloudFeatureFlag[i] = 100;
                        }
                    }
                }
                else
                {

                    Eigen::Vector3d surf_vector = Eigen::Vector3d(laserCloud->points[i + 4].x - laserCloud->points[i].x,
                                                                  laserCloud->points[i + 4].y - laserCloud->points[i].y,
                                                                  laserCloud->points[i + 4].z - laserCloud->points[i].z);
                    Eigen::Vector3d lidar_vector = Eigen::Vector3d(laserCloud->points[i].x,
                                                                   laserCloud->points[i].y,
                                                                   laserCloud->points[i].z);
                    double right_surf_dis = surf_vector.norm();
                    //calculate the angle between the laser direction and the surface
                    double cc = fabs(surf_vector.dot(lidar_vector) / (surf_vector.norm() * lidar_vector.norm()));

                    std::vector<PointT> right_list;
                    double min_dis = 10000;
                    double max_dis = 0;
                    for (int j = 0; j < 4; j++)
                    { //TODO: change the plane window size and add thin rod support
                        right_list.push_back(laserCloud->points[i - j]);
                        Eigen::Vector3d temp_vector = Eigen::Vector3d(laserCloud->points[i + j].x - laserCloud->points[i + j + 1].x,
                                                                      laserCloud->points[i + j].y - laserCloud->points[i + j + 1].y,
                                                                      laserCloud->points[i + j].z - laserCloud->points[i + j + 1].z);

                        if (j == 3)
                            break;
                        double temp_dis = temp_vector.norm();
                        if (temp_dis < min_dis)
                            min_dis = temp_dis;
                        if (temp_dis > max_dis)
                            max_dis = temp_dis;
                    }
                    bool right_is_plane = plane_judge(right_list, 100);

                    if (right_is_plane && (max_dis < 2 * min_dis) && right_surf_dis < 0.05 * depth && cc < 0.8)
                    { //

                        if (depth_right < depth_left)
                        {
                            CloudFeatureFlag[i] = 100;
                        }
                        else
                        {
                            if (depth_left == 0)
                                CloudFeatureFlag[i] = 100;
                        }
                    }
                }
            }

            // break point select
            if (CloudFeatureFlag[i] == 100)
            {
                debugnum2++;
                std::vector<Eigen::Vector3d> front_norms;
                Eigen::Vector3d norm_front(0, 0, 0);
                Eigen::Vector3d norm_back(0, 0, 0);
                for (int k = 1; k < 4; k++)
                {
                    Eigen::Vector3d tmp = Eigen::Vector3d(laserCloud->points[i - k].x - laserCloud->points[i].x,
                                                          laserCloud->points[i - k].y - laserCloud->points[i].y,
                                                          laserCloud->points[i - k].z - laserCloud->points[i].z);
                    tmp.normalize();
                    front_norms.push_back(tmp);
                    norm_front += (k / 6.0) * tmp;
                }
                std::vector<Eigen::Vector3d> back_norms;
                for (int k = 1; k < 4; k++)
                {
                    Eigen::Vector3d tmp = Eigen::Vector3d(laserCloud->points[i + k].x - laserCloud->points[i].x,
                                                          laserCloud->points[i + k].y - laserCloud->points[i].y,
                                                          laserCloud->points[i + k].z - laserCloud->points[i].z);
                    tmp.normalize();
                    back_norms.push_back(tmp);
                    norm_back += (k / 6.0) * tmp;
                }
                double cc = fabs(norm_front.dot(norm_back) / (norm_front.norm() * norm_back.norm()));
                if (cc < 0.8)
                {
                    debugnum3++;
                }
                else
                {
                    CloudFeatureFlag[i] = 0;
                }

                continue;
            }
        }

        //push_back feature
        for (int i = 0; i < cloudSize; i++)
        {
            //laserCloud->points[i].intensity = double(CloudFeatureFlag[i]) / 10000;
            float dis = laserCloud->points[i].x * laserCloud->points[i].x + laserCloud->points[i].y * laserCloud->points[i].y + laserCloud->points[i].z * laserCloud->points[i].z;
            float dis2 = laserCloud->points[i].y * laserCloud->points[i].y + laserCloud->points[i].z * laserCloud->points[i].z;
            float theta2 = std::asin(sqrt(dis2 / dis)) / M_PI * 180;
            //std::cout<<"DEBUG theta "<<theta2<<std::endl;
            // if(theta2 > 34.2 || theta2 < 1){
            //    continue;
            // }
            //if(dis > 30*30) continue;

            if (CloudFeatureFlag[i] == 1)
            {
                surfPointsFlat.push_back(laserCloud->points[i]);
                continue;
            }

            if (CloudFeatureFlag[i] == 100 || CloudFeatureFlag[i] == 150)
            {
                cornerPointsSharp.push_back(laserCloud->points[i]);
            }
        }

        std::cout << "ALL point: " << cloudSize << " outliers: " << debugnum1 << std::endl
                  << " break points: " << debugnum2 << " break feature: " << debugnum3 << std::endl
                  << " normal points: " << debugnum4 << " surf-surf feature: " << debugnum5 << std::endl;

        std::cout << "surf cloud pts:" << surfPointsFlat.size() << std::endl;
        std::cout << "corner cloud pts:" << cornerPointsSharp.size() << std::endl;
    }

    template class PCL_EXPORTS Registration<pcl::PointXYZINormal, float>;
}