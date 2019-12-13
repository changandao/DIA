//
// Created by font_al on 10/5/18.
//

#include "frame.h"

//This function projects into the World Reference frame ALL the points belonging to the frame.
void fv::Frame::xynlambda_2_XYZ(std::vector<Pt>* points){
    for (fv::Pt& pt: *points){
        xynlambda_2_XYZ(pt);
    }
}

void fv::Frame::xynlambda_2_XYZ(fv::Pt& pt){

    DATA_TYPE xc_ref{}, yc_ref{}, zc_ref{1/(pt.lambda_ref)};

    for(size_t i_mask = 0; i_mask < cam->mask_size; ++i_mask) {

        //Depth projection
        xc_ref = pt.xn_ref[i_mask] * zc_ref;
        yc_ref = pt.yn_ref[i_mask] * zc_ref;

        // 3D projection
        pt.X[i_mask] = xc_ref * R_wc[0] + yc_ref * R_wc[1] + zc_ref * R_wc[2] + t_wc[0];
        pt.Y[i_mask] = xc_ref * R_wc[3] + yc_ref * R_wc[4] + zc_ref * R_wc[5] + t_wc[1];
        pt.Z[i_mask] = xc_ref * R_wc[6] + yc_ref * R_wc[7] + zc_ref * R_wc[8] + t_wc[2];
    }
}

//
void fv::Frame::XYZ_2_xynlambda(fv::Pt& pt){

    for(size_t i_mask{}; i_mask < cam->mask_size; ++i_mask){

        pt.xc[i_mask] = pt.X[i_mask]*R_cw[0]+pt.Y[i_mask]*R_cw[1]+pt.Z[i_mask]*R_cw[2]+t_cw[0];
        pt.yc[i_mask] = pt.X[i_mask]*R_cw[3]+pt.Y[i_mask]*R_cw[4]+pt.Z[i_mask]*R_cw[5]+t_cw[1];
        pt.zc[i_mask] = pt.X[i_mask]*R_cw[6]+pt.Y[i_mask]*R_cw[7]+pt.Z[i_mask]*R_cw[8]+t_cw[2];

        //Coordinates normalization
        pt.xn[i_mask] = pt.xc[i_mask]/pt.zc[i_mask];
        pt.yn[i_mask] = pt.yc[i_mask]/pt.zc[i_mask];
    }
}

void fv::Frame::XYZ_2_xynlambda(fv::Pt& pt, const VECTOR3 t_cw, const MATRIX3 R_cw){

    for(size_t i_mask{}; i_mask < cam->mask_size; ++i_mask){

        pt.xc[i_mask] = pt.X[i_mask]*R_cw[0]+pt.Y[i_mask]*R_cw[1]+pt.Z[i_mask]*R_cw[2]+t_cw[0];
        pt.yc[i_mask] = pt.X[i_mask]*R_cw[3]+pt.Y[i_mask]*R_cw[4]+pt.Z[i_mask]*R_cw[5]+t_cw[1];
        pt.zc[i_mask] = pt.X[i_mask]*R_cw[6]+pt.Y[i_mask]*R_cw[7]+pt.Z[i_mask]*R_cw[8]+t_cw[2];
        //Coordinates normalization
        pt.xn[i_mask] = pt.xc[i_mask]/pt.zc[i_mask];
        pt.yn[i_mask] = pt.yc[i_mask]/pt.zc[i_mask];
    }
}

void fv::Frame::computePhotometricError(EIGEN_VECTOR_X& e, fv::Pt& pt, const DATA_TYPE phScalar_, const DATA_TYPE phBias_) {

    DATA_TYPE phScale{EXP(-phScalar_)};

    auto* refKeyframe = static_cast<fv::Frame*>(pt.refKeyframe);

    DATA_TYPE phScaleRef{EXP(-refKeyframe->phScalar)};
    DATA_TYPE phBiasRef{refKeyframe->phBias};

    extract_I_and_G(pt);

    for(size_t iMask{}; iMask < cam->mask_size; ++iMask){
        e(iMask) = (phScaleRef*(pt.I_ref[iMask][cam->iPyr]-phBiasRef) - phScale*(pt.I[iMask]-phBias_));
    }
}

DATA_TYPE fv::Frame::computePhotometricError(fv::Pt& pt,DATA_TYPE measurement, size_t i_mask, const DATA_TYPE phScalar_, const DATA_TYPE phBias_) {

    DATA_TYPE e{0};
    DATA_TYPE phScale{EXP(-phScalar_)};

    auto* refKeyframe = static_cast<fv::Frame*>(pt.refKeyframe);

    DATA_TYPE phScaleRef{EXP(-refKeyframe->phScalar)};
    DATA_TYPE phBiasRef{refKeyframe->phBias};

    extract_I_and_G(pt);

    e = ( phScale*(pt.I[i_mask]-phBias_)- phScaleRef*(measurement-phBiasRef));
    return e;
}

void fv::Frame::computePhotometricJacobian(fv::Pt& pt,EIGEN_MATRIX_X& J,EIGEN_VECTOR_X& e,EIGEN_VECTOR_X& eAbs, DATA_TYPE ZeroPh, DATA_TYPE HuberPh) {

    DATA_TYPE J1,J2,J3;
    DATA_TYPE phScale = EXP(-phScalar);

    for(size_t i_mask{}; i_mask < cam->mask_size; ++i_mask) {
        if (eAbs(i_mask) > ZeroPh) {
            J.row(i_mask) = EIGEN_VECTOR_X::Zero(8);
        }
        else{
            cam->computeJPhotometric(J1, J2, J3, pt, i_mask);

            J.row(i_mask) << J1,
                             J2,
                             J3,
                            -J2 * (pt.zc[i_mask]) + J3 * (pt.yc[i_mask]),
                            J1 * (pt.zc[i_mask]) - J3 * (pt.xc[i_mask]),
                            -J1 * (pt.yc[i_mask]) + J2 * (pt.xc[i_mask]),
                            pt.I[i_mask]-phBias,
                            1;

            J.row(i_mask) *= phScale;

        }

        if(eAbs(i_mask)  > HuberPh){
            J.row(i_mask) *= ABS(HuberPh/e(i_mask));
        }

    }

}

void fv::Frame::computePhotometricJacobian(fv::Pt& pt, EIGEN_VECTOR_X& J, size_t i_mask)
{

    DATA_TYPE J1,J2,J3;
    DATA_TYPE phScale = EXP(-phScalar);

    cam->computeJPhotometric(J1, J2, J3, pt, i_mask);

    J << J1,
            J2,
            J3,
            -J2 * (pt.zc[i_mask]) + J3 * (pt.yc[i_mask]),
            J1 * (pt.zc[i_mask]) - J3 * (pt.xc[i_mask]),
            -J1 * (pt.yc[i_mask]) + J2 * (pt.xc[i_mask]),
            pt.I[i_mask]-phBias,
            1;

    J *= phScale;





}

// This function extracts 'interesting' points for the odometry.
void fv::Frame::extractPts(const size_t& mode, const std::vector<size_t>& meshExtractPoints, bool* templateExtractPoints){

    switch (mode) {

        case 0: { // Extract HGP points in the style of DSO.

            HGP_ref.clear();
            HGP_refNotInitialized.clear();

            cv::Mat Mat_grad;
            fv::grad_function(Mat_grad,Mat_gray.clone());

            cam->extractHGP(Mat_grad.clone(), HGP_refNotInitialized, sysSet.dForExtractHgp, meshExtractPoints,templateExtractPoints);
            numHGP_ref = HGP_ref.size();
            numHGP_refNotInitialized = HGP_refNotInitialized.size();

            break;
        }

        case 1: { // Extract HGP points on Canny Edges.

            cv::Mat detected_edges;
            detected_edges = cv::Scalar::all(0);

            Canny(Mat_gray.clone(), detected_edges, sysSet.canny_lowThreshold, sysSet.canny_lowThreshold * sysSet.canny_ratio,sysSet.canny_kernel_size);
            detected_edges.convertTo(detected_edges, DATA_TYPE_MAT);
            HGP_ref.clear();
            HGP_refNotInitialized.clear();
            cam->extractHGP(detected_edges.clone(), HGP_refNotInitialized, sysSet.dForExtractHgp, meshExtractPoints,templateExtractPoints);
            numHGP_ref = HGP_ref.size();
            numHGP_refNotInitialized = HGP_refNotInitialized.size();

            break;
        }

        case 2: { // Extract HGP points in the style of DSO + Canny Edges

            cv::Mat detected_edges;
            detected_edges = cv::Scalar::all(0);
            Canny(Mat_gray.clone(), detected_edges, sysSet.canny_lowThreshold, sysSet.canny_lowThreshold * sysSet.canny_ratio,sysSet.canny_kernel_size);
            detected_edges.convertTo(detected_edges, DATA_TYPE_MAT);

            HGP_ref.clear();
            HGP_refNotInitialized.clear();

            cv::Mat Mat_grad;
            fv::grad_function(Mat_grad, Mat_gray.clone());

            Mat_grad += detected_edges;

            cam->extractHGP(Mat_grad.clone(), HGP_refNotInitialized, sysSet.dForExtractHgp, meshExtractPoints,templateExtractPoints);
            numHGP_ref = HGP_ref.size();
            numHGP_refNotInitialized = HGP_refNotInitialized.size();

            break;
        }
    }

}

//This function extract the reference intensity and gradients for ALL the points related to the frame,
// for the whole mask and, at ALL pyramid levels.
void fv::Frame::extract_I_and_G_reference(std::vector<Pt>* points){
    for(fv::Pt& pt: *points){
        for(int i_pyr{}; i_pyr < PYR_LEVELS_USED; ++i_pyr) {
            fv::Frame::extract_I_and_G_reference(pt, i_pyr);
        }
    }
}

//This function extract the reference intensity and gradients for ONE point related to the frame,
// for the whole mask and, at ONE pyramid level.
void fv::Frame::extract_I_and_G_reference(fv::Pt& point, const int iPyr){
    DATA_TYPE inc_u, inc_v;
    DATA_TYPE I11, I12, I21, I22;

    DATA_TYPE u{},v{};
    int up{},vp{};

    for(int i_mask{MASK_SIZE-1}; i_mask > -1 ; --i_mask){

        u = point.u_ref[i_mask]*cam->scaleFactorG[iPyr];
        v = point.v_ref[i_mask]*cam->scaleFactorG[iPyr];

        up = int(u);
        vp = int(v);

        inc_u = (u-up);
        inc_v = (v-vp);

        I11 = mat_grayG[iPyr].at<DATA_TYPE>(cv::Point(up,vp));
        I12 = mat_grayG[iPyr].at<DATA_TYPE>(cv::Point(up+1,vp));
        I21 = mat_grayG[iPyr].at<DATA_TYPE>(cv::Point(up,vp+1));
        I22 = mat_grayG[iPyr].at<DATA_TYPE>(cv::Point(up+1,vp+1));

        point.I_ref[i_mask][iPyr]  = (I11*(1-inc_v) + I21*inc_v)*(1-inc_u) + (I12*(1-inc_v) + I22*inc_v)*inc_u ;
        if(iPyr == 0){
            point.Gu_ref[i_mask] =  (I22-I21)*inc_v +(I12-I11)*(1-inc_v);
            point.Gv_ref[i_mask] =  (I22-I12)*inc_u +(I21-I11)*(1-inc_u);
        }
    }
}


//This function extract the intensity and gradients for ONE point related to the frame,
// for the whole mask and, at ONE pyramid level.
void fv::Frame::extract_I_and_G(fv::Pt& point){

    DATA_TYPE inc_u, inc_v;
    DATA_TYPE I11, I12, I21, I22;

    DATA_TYPE u{};
    DATA_TYPE v{};

    int up{};
    int vp{};
    int iPyr = cam->iPyr;

    int wG_max = cam->wG[iPyr]-3;
    int hG_max = cam->hG[iPyr]-3;

    for(size_t i_mask{}; i_mask < cam->mask_size; ++i_mask){

        u = point.u[i_mask];
        v = point.v[i_mask];
        up = int(u);
        vp = int(v);

        inc_u = (u-DATA_TYPE(up));
        inc_v = (v-DATA_TYPE(vp));

        if((up < 0)or(vp < 0)or(up > wG_max)or(vp > hG_max)){
            point.I[i_mask]  = 0;
            point.Gu[i_mask] = 0;
            point.Gv[i_mask] = 0;
        }
        else{
            I11 = mat_grayG[iPyr].at<DATA_TYPE>(cv::Point(up,vp));
            I12 = mat_grayG[iPyr].at<DATA_TYPE>(cv::Point(up+1,vp));
            I21 = mat_grayG[iPyr].at<DATA_TYPE>(cv::Point(up,vp+1));
            I22 = mat_grayG[iPyr].at<DATA_TYPE>(cv::Point(up+1,vp+1));

            point.I[i_mask] = (I11*(1-inc_v) + I21*inc_v)*(1-inc_u) + (I12*(1-inc_v) + I22*inc_v)*inc_u ;
            point.Gu[i_mask] =  (I22-I21)*inc_v +(I12-I11)*(1-inc_v);
            point.Gv[i_mask] =  (I22-I12)*inc_u +(I21-I11)*(1-inc_u);
        }
    }
}

#ifdef KINECT
//This function gets the depth of the points from a depth map of a kinect sensor.
void fv::Frame::getLambdaRef_static(){

    DATA_TYPE depth{0}, depthi{0}, lambdaRef;
    DATA_TYPE stdDepth{};

    std::vector<fv::Pt> pointsRefTemporal = HGP_refNotInitialized;

    lambdaRefMean = 0;
    HGP_ref.clear();
    HGP_refNotInitialized.clear();
    numHGP_ref = 0;
    numHGP_refNotInitialized = 0;

    bool thereIsDepth;
    for(fv::Pt& pt: pointsRefTemporal) {

        thereIsDepth = false;
        depth = frame_depth.at<DATA_TYPE>(cv::Point(int(pt.u_ref[0]),int(pt.v_ref[0])))/ seqSet.depthConstant;
        if ((not std::isnan(depth))&&(depth > sysSet.depthMin) && (depth < sysSet.depthMax)) {
            thereIsDepth = true;
        } else {
            for (size_t u_patch{0}; u_patch < sysSet.u_SizeKinectPatch; ++u_patch) {
                for (size_t v_patch{0}; v_patch < sysSet.v_SizeKinectPatch; ++v_patch) {

                    depthi = frame_depth.at<DATA_TYPE>(cv::Point(int(int(pt.u_ref[0]) - 1 + u_patch),int(int(pt.v_ref[0]) - 1 + v_patch)))/ seqSet.depthConstant;

                    if ((depthi > sysSet.depthMin) && (depthi < sysSet.depthMax) && (depthi < depth)) {
                        thereIsDepth = true;
                        depth = depthi;
                    }
                }
            }
        }

        pt.refKeyframe = this;

        if (thereIsDepth) {
            depth = depth * METRIC_UNIT_CONV;
            lambdaRef = 1 / (depth);
            pt.setLambdaRef(lambdaRef);

            stdDepth = depth*depth;
            pt.setStdLambdaRef(stdDepth);

            HGP_ref.push_back(pt);
            lambdaRefMean += lambdaRef;

            ++numHGP_ref;

        } else {
            pt.deletePointers();
        }
    }
}
#endif

#ifdef DISPARITY
//This function gets the depth of the points from a disparity map.
void fv::Frame::getLambdaRef_static(){

    DATA_TYPE depth{0}, depthi{0}, lambdaRef,disparity;
    DATA_TYPE stdDepth{};

    std::vector<fv::Pt> pointsRefTemporal = HGP_refNotInitialized;

    lambdaRefMean = 0;
    HGP_ref.clear();
    HGP_refNotInitialized.clear();
    numHGP_ref = 0;
    numHGP_refNotInitialized = 0;

    bool thereIsDepth;
    for(fv::Pt& pt: pointsRefTemporal) {

        thereIsDepth = false;
        disparity = frame_depth.at<DATA_TYPE>(cv::Point(int(int(pt.u_ref[0])),int(int(pt.v_ref[0]) )));

        if (disparity > 0.0000001) {         
            depth = seqSet.depthConstant / (disparity);
            if ((depth > sysSet.depthMin) && (depth < sysSet.depthMax)) {
                thereIsDepth = true;
            }
        } else {
            depth = sysSet.depthMax;
            for (size_t u_patch{0}; u_patch < sysSet.u_SizeKinectPatch; ++u_patch) {
                for (size_t v_patch{0}; v_patch < sysSet.v_SizeKinectPatch; ++v_patch) {
                    disparity = frame_depth.at<DATA_TYPE>(cv::Point(int(int(pt.u_ref[0]) - 1 + u_patch),int(int(pt.v_ref[0]) - 1 + v_patch)));
                    if (disparity > 0.0000001) {
                        depthi = seqSet.depthConstant / (disparity);
                        if ((depthi > sysSet.depthMin) && (depthi < sysSet.depthMax) && (depthi < depth)) {
                            thereIsDepth = true;
                            depth = depthi;
                        }
                    }
                }
            }
        }

        pt.refKeyframe = this;

        if (thereIsDepth) {
            depth = depth * METRIC_UNIT_CONV;
            lambdaRef = 1 / (depth);
            pt.setLambdaRef(lambdaRef);

            stdDepth = depth * depth;
            pt.setStdLambdaRef(stdDepth);

            HGP_ref.push_back(pt);
            lambdaRefMean += lambdaRef;

            ++numHGP_ref;

        } else {
            pt.deletePointers();
        }

    }
    drawDepthMap();
}
#endif