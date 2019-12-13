//
// Created by wang_sn on 4/24/19.
//

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "../classes/trackingFrontEnd.h"
//// g2o include
#include "../3rdparty/g2o/g2o/core/base_vertex.h"
#include "../3rdparty/g2o/g2o/core/base_unary_edge.h"
#include "../3rdparty/g2o/g2o/core/block_solver.h"
#include "../3rdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "../3rdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "../3rdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "../3rdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "../3rdparty/g2o/g2o/core/robust_kernel.h"
#include "../3rdparty/g2o/g2o/core/robust_kernel_impl.h"


namespace fv {
    class VertexCamerawithSSD : public g2o::BaseVertex<8, EIGEN_VECTOR_X>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexCamerawithSSD(){};
        virtual bool read ( std::istream& /*is*/ )
        {
            return false;
        }

        virtual bool write ( std::ostream& /*os*/ ) const
        {
            return false;
        }

        void setToOriginImpl() override {}


        void oplusImpl( const double* update ) override
        {
            Eigen::VectorXd::ConstMapType v ( update, VertexCamerawithSSD::Dimension );
            updateOptimizationVariables(v);
        }

    protected:

        void updateOptimizationVariables(Eigen::VectorXd::ConstMapType v){


            g2o::SE3Quat updateSE3;
            updateSE3.fromMinimalVector(v.head(6));
            g2o::SE3Quat estimateSE3;
            estimateSE3.fromMinimalVector(_estimate.head(6));
            estimateSE3 = updateSE3*estimateSE3;
            EIGEN_VECTOR_X estimate_se3;
            estimate_se3 = estimateSE3.toMinimalVector();

            DATA_TYPE phScalar_track = v(6);
            DATA_TYPE phBias_track   = v(7);

            _estimate.head(3) = estimate_se3.head(3);
            _estimate.head(6).tail(3) = estimate_se3.head(6).tail(3) ;
            _estimate(6) += phScalar_track;
            _estimate(7) += phBias_track;

        }

    };

class EdgeDirectwithSSD : public g2o::BaseUnaryEdge< 1, DATA_TYPE, VertexCamerawithSSD> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeDirectwithSSD(Frame* frame, fv::Pt*, size_t i_mask,const EIGEN_VECTOR_X& cw_ref);
        // Error: measure = R*point+t
        virtual void computeError();

        virtual void linearizeOplus();

        virtual bool read(std::istream &in) {}

        virtual bool write(std::ostream &out) const {}

        //EIGEN_VECTOR_3 point_;
        //Camera* camera_;
        Frame* frame_;
        fv::Pt* pt_;
        //cv::Mat* image_=nullptr;
        //size_t numHgpForTracking{};
        //std::vector <fv::Pt*> hgpForTracking{};

    protected:

        size_t i_mask_;
        EIGEN_VECTOR_X cw_ref_;

    };



}


#endif //OPTIMIZER_H
