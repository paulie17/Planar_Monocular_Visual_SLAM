#pragma once
#define G2O_USE_VENDORED_CERES

#include <g2o/core/base_vertex.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/core/base_unary_edge.h>

#include <g2o/stuff/misc.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
// #include <g2o/solvers/csparse/linear_solver_csparse.h>
// #include <g2o/solvers/pcg/linear_solver_pcg.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <utils.hpp>

namespace visual_slam{

    class VertexRobotSE2 : public g2o::BaseVertex<3, Eigen::Isometry2d>
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            
            VertexRobotSE2(){}

            virtual void setToOriginImpl() override{
                _estimate = Eigen::Isometry2d::Identity();
            }

            virtual void oplusImpl(const double* update) override{
                Eigen::Isometry2d up;
                up.translation() = Eigen::Vector2d(update[0],update[1]);
                up.linear() = Eigen::Rotation2D<double>(update[2]).matrix();   
                _estimate = up * _estimate;
            }

            virtual bool read(std::istream& is){}
            virtual bool write(std::ostream& os) const{}

            Eigen3_4d p_matrix; // projection matrix

            inline Eigen::Vector2d project(const Eigen::Vector3d& point_world) {
                Eigen::Vector3d point_pixels_extended = p_matrix*Eigen::Vector4d(point_world.x(),point_world.y(),point_world.z(),1.);
                point_pixels_extended /= point_pixels_extended(2);
                Eigen::Vector2d point_pixels = point_pixels_extended.segment<2>(0);
                return point_pixels;
            };
    };

    class EdgeSE2projectXYZ : public g2o::BaseBinaryEdge <2, Eigen::Vector2d, g2o::VertexPointXYZ, VertexRobotSE2>
    {
        public: 
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            EdgeSE2projectXYZ(){}
            bool read(std::istream& is){}
            bool write(std::ostream& os) const{}

            inline void computeError(){      
                VertexRobotSE2* pose = static_cast< VertexRobotSE2*> ( _vertices[1] );        
                g2o::VertexPointXYZ* landmark = static_cast< g2o::VertexPointXYZ*> ( _vertices[0] );                   
                _error = _measurement - pose->project(landmark->estimate());
            }

            // virtual void linearizeOplus(); For the moment keep the Jacobian computation as numerical

    }; 

    class EdgeSE2Custom : public g2o::BaseBinaryEdge <3, Eigen::Vector3d, VertexRobotSE2, VertexRobotSE2>
    {
        public: 
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            EdgeSE2Custom(){}
            bool read(std::istream& is){}
            bool write(std::ostream& os) const{}

            inline void computeError(){
                VertexRobotSE2* pose_1 = static_cast< VertexRobotSE2*> ( _vertices[0] ); 
                VertexRobotSE2* pose_2 = static_cast< VertexRobotSE2*> ( _vertices[1] ); 
                Eigen::Isometry2d transform = pose_1->estimate().inverse()*pose_2->estimate();
                Eigen::Vector3d v;
                v.head<2>()=transform.translation();
                v(2) = atan2(transform.linear()(1,0), transform.linear()(0,0));
                _error = _measurement - v;
                _error[2] = g2o::normalize_theta(_error[2]);
            }
    };

    class EdgeSE2landmarkXYZ : public g2o::BaseBinaryEdge <3, Eigen::Vector3d, g2o::VertexPointXYZ, g2o::VertexSE2>{

        public: 
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            EdgeSE2landmarkXYZ(){}
            bool read(std::istream& is){}
            bool write(std::ostream& os) const{}

            inline void computeError(){      
                g2o::VertexSE2* pose = static_cast< g2o::VertexSE2*> ( _vertices[1] );        
                g2o::VertexPointXYZ* landmark = static_cast< g2o::VertexPointXYZ*> ( _vertices[0] );          
                _error = _measurement - t2t3d(pose->estimate().toIsometry())*landmark->estimate() ;
                // _error = _measurement - pose->project(landmark->estimate());
            }

    };


} // namespace visual_slam
