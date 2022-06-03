#pragma once

#include <eigen_conversions/eigen_msg.h>

namespace planar_monocular_slam{

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
    
} // namespace utils