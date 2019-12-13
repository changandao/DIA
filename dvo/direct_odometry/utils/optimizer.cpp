//
// Created by wang_sn on 4/24/19.
//

#include "optimizer.h"
namespace fv{
    EdgeDirectwithSSD::EdgeDirectwithSSD(Frame* frame, fv::Pt* pt, size_t i_mask, const EIGEN_VECTOR_X& cw_ref):
    frame_(frame), pt_(pt), i_mask_(i_mask), cw_ref_(cw_ref)
    {
        cw_ref_ = EIGEN_VECTOR_X::Zero(6);

    }

    void EdgeDirectwithSSD::computeError() {


        const VertexCamerawithSSD* camwithSSD = static_cast<VertexCamerawithSSD*>(vertex(0));
        DATA_TYPE phScalar_track = camwithSSD->estimate()[6];
        DATA_TYPE phBias_track = camwithSSD->estimate()[7];

//        EIGEN_VECTOR_3 v_cw_track_ref = cw_ref_.head(3);
//        EIGEN_VECTOR_3 w_cw_track_ref = cw_ref_.head(6).tail(3);

        //int maskSize = int(frame_->cam->mask_size);
        DATA_TYPE e{0.};

        g2o::SE3Quat cw_trackSE3;
        cw_trackSE3.fromMinimalVector(camwithSSD->estimate().head(6));
        auto* refKeyframe = static_cast<fv::Frame*>(pt_->refKeyframe);
        cw_ref_.head(3) = refKeyframe->v_cw;
        cw_ref_.tail(3) = refKeyframe->w_cw;
        g2o::SE3Quat cw_track_refSE3;
        cw_track_refSE3.fromMinimalVector(cw_ref_.head(6));
        //
        cw_trackSE3 = cw_trackSE3.inverse()*cw_track_refSE3;
        EIGEN_VECTOR_X  cw_track{cw_trackSE3.toMinimalVector()};
        EIGEN_VECTOR_3 v_cw_track = cw_track.head(3);
        EIGEN_VECTOR_3 w_cw_track = cw_track.head(6).tail(3);
        fv::VECTOR3 t_cw_track{};
        fv::MATRIX3 R_cw_track{};
        fv::exp_lie(t_cw_track,R_cw_track,v_cw_track,w_cw_track);


        frame_->XYZ_2_xynlambda(*pt_,t_cw_track,R_cw_track);

        frame_->cam->xyn_2_uv(*pt_);
        //std::cout<< i_mask_<<"th mask in optimization"<<std::endl;

        e = frame_->computePhotometricError(*pt_, measurement(), i_mask_, phScalar_track, phBias_track);


        _error(0,0) = e;


    }

    void EdgeDirectwithSSD::linearizeOplus() {
        VertexCamerawithSSD *pose = static_cast<VertexCamerawithSSD *> ( _vertices[0] );

        EIGEN_VECTOR_X J = EIGEN_VECTOR_X::Zero(8);

        frame_->computePhotometricJacobian(*pt_, J, i_mask_);
        if ( level() == 1 )
        {
            _jacobianOplusXi = EIGEN_VECTOR_X::Zero(1,8);
            return;
        }
        _jacobianOplusXi = J;

    }

}