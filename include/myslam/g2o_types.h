//
// Created by changandao on 15.12.18.
//


#ifndef MYSLAM_G2O_TYPES_H
#define MYSLAM_G2O_TYPES_H

#include "myslam/common_include.h"
#include "frame.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

namespace myslam {

// only to optimize the pose, no point
    class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge< 1, double, g2o::VertexSE3Expmap> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        // Error: measure = R*point+t
        virtual void computeError();

        virtual void linearizeOplus();

        virtual bool read(std::istream &in) {}

        virtual bool write(std::ostream &out) const {}

        Vector3d point_;
        //Camera* camera_;
        Frame* frame_;
        //cv::Mat* image_=nullptr;


    protected:
        // get a gray scale value from reference image (bilinear interpolated)
        inline float getPixelValue (float x, float y )
        {
            uchar* data = & frame_->gray_.data[ int ( y ) * frame_->gray_.step + int ( x ) ];
            float xx = x - floor ( x );
            float yy = y - floor ( y );
            return float (
                    ( 1-xx ) * ( 1-yy ) * data[0] +
                    xx* ( 1-yy ) * data[1] +
                    ( 1-xx ) *yy*data[ frame_->gray_.step ] +
                    xx*yy*data[frame_->gray_.step+1]
            );
        }
    };
}



#endif // MYSLAM_G2O_TYPES_H
