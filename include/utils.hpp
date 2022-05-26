#pragma once

#include <eigen_conversions/eigen_msg.h>

namespace visual_slam{

    typedef Eigen::Matrix<double, 3, 4> Eigen3_4d;
    typedef Eigen::Matrix<double, 4, 3> Eigen4_3d;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Vector3dVector;

    // template <class Derived>
    // void floatMultiArrayToEigen(std_msgs::Float64MultiArray &m, Eigen::MatrixBase<Derived> &e)
    // {
    //     if (m.layout.dim.size() != 2)
    //     {
    //         ROS_ERROR_STREAM("Float64MultiArray must have 2 dimensions to be converted to an Eigen Vector or Matrix. This message has "<< m.layout.dim.size() <<" dimensions.");
    //         return;
    //     }

    //     int rows = m.layout.dim[0].size;
    //     int cols = m.layout.dim[1].size;

    //     // Data is stored in row major order
    //     e = Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>>(m.data.data(), rows, cols);

    //     // e = Eigen::Map<Derived>(m.data.data(), rows, cols);
    // }

    



    inline Eigen::Isometry3d t2t3d(const Eigen::Isometry2d &iso){
        Eigen::Isometry3d transform;
        transform.setIdentity();
        transform.linear().block<2, 2>(0, 0) = iso.linear();
        transform.translation().block<2, 1>(0, 0) = iso.translation();
        return transform;
    }
    
    // inline Vector6d t2v(const Eigen::Isometry3d& t){
    //     Vector6d v;
    //     v.head<3>()=t.translation();
    //     Eigen::Quaterniond q(t.linear());
    //     v.block<3,1>(3,0)=q.matrix().block<3,1>(1,0);
    //     if (q.w()<0)
    //     v.block<3,1>(3,0) *= -1.0f;
    //     return v;
    // }

    inline Vector6d t2v(const Eigen::Isometry3d& t){
        Vector6d v;
        v.head<3>()=t.translation();
        Eigen::Quaterniond q(t.linear());
        v.block<3,1>(3,0)=q.toRotationMatrix().eulerAngles(0, 1, 2);
        return v;
    }

    inline Eigen::Vector3d t2v(const Eigen::Isometry2d& t){
        Eigen::Vector3d v;
        v.head<2>()=t.translation();
        v(2) = atan2(t.linear()(1,0), t.linear()(0,0));
        return v;
    }

    inline Eigen::Isometry2d v2t(const Eigen::Vector3d& t){
        Eigen::Isometry2d T;
        T.setIdentity();
        T.translation()=t.head<2>();
        double c = cos(t(2));
        double s = sin(t(2));
        T.linear() << c, -s, s, c;
        return T;
    }    

    inline Eigen::Isometry2d t3t2d(const Eigen::Isometry3d &iso){
        Vector6d v_3d = t2v(iso);
        Eigen::Vector3d v_2d;
        v_2d.segment<2>(0) = v_3d.segment<2>(0);
        v_2d(2) = v_3d(5);
        // std::cout << v_2d << std::endl;
        return v2t(v_2d);
    }
    
    inline void covariance_3d_to_control_points( const Eigen::Matrix3d& cov, const Eigen::Vector3d& mean, Vector3dVector& sigma_pts){

            double lambda, alpha, n;
            int point_idx = 1;
            Eigen::Matrix3d L;
            
            n = 3;
            alpha = 0.001;
            lambda = alpha*alpha*n;

            L = ((n+lambda)*cov).llt().matrixL();
            

            sigma_pts.clear();
            sigma_pts.reserve(2*n+1);
            sigma_pts[0] = mean;

            for (int i = 0; i < n; i ++){            
                
                sigma_pts[point_idx] = mean + L.col(i);
                point_idx++;
                sigma_pts[point_idx] = mean - L.col(i);
                point_idx++;
            }
            
        }

        inline void control_pts_to_pose_omega(  const Vector3dVector& sigma_pts, const Eigen::Vector3d& mean, 
                                                    const Eigen::Isometry2d& origin_pose, const Eigen::Isometry2d& separator_pose,
                                                    Eigen::Matrix3d& omega_condensed){
            Vector3dVector transformed_sigma_pts;
            int n = 3;
            double alpha = 0.001;
            double lambda = alpha*alpha*n;
            double w_c = 1/(2*(n+lambda));
            
            omega_condensed.setZero();

            transformed_sigma_pts.reserve(2*n+1);

            for (int i = 0; i < 2*n+1; i ++){

                transformed_sigma_pts[i] = t2v( v2t(mean).inverse() * origin_pose.inverse() * v2t( sigma_pts[i] ) * separator_pose );

            }

            for (int i = 1; i < 2*n+1; i++){

                omega_condensed += w_c * (transformed_sigma_pts[i] - mean)*(transformed_sigma_pts[i] - mean).transpose();

            }
            
            omega_condensed = omega_condensed.inverse().eval();
        }
} // namespace utils