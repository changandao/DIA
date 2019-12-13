//
// Created by font_al on 10/26/18.
//

#include "camera.h"

fv::CamPinholeDist::CamPinholeDist(const DATA_TYPE& fxValue, const DATA_TYPE& fyValue, const DATA_TYPE& cxValue, const DATA_TYPE& cyValue, const cv::Mat distCoeffsValue,
        const size_t& wValue, const size_t& hValue):distCoeffs{distCoeffsValue}{


    fxG  = new DATA_TYPE[pyrLevelsMax]; fyG  = new DATA_TYPE[pyrLevelsMax];
    cxG  = new DATA_TYPE[pyrLevelsMax]; cyG  = new DATA_TYPE[pyrLevelsMax];
    fxiG = new DATA_TYPE[pyrLevelsMax]; fyiG = new DATA_TYPE[pyrLevelsMax];
    cxiG = new DATA_TYPE[pyrLevelsMax]; cyiG = new DATA_TYPE[pyrLevelsMax];
    wG   = new size_t[pyrLevelsMax]   ; hG   = new size_t[pyrLevelsMax];
    numPixelsG = new size_t[pyrLevelsMax];

    fv::CamPinhole::setImageDimensions(wValue,hValue);
    fv::CamPinholeDist::setCalibration(fxValue, fyValue, cxValue, cyValue);
    fv::Camera::setMask(MASK_SIZE);

    templateExtractPoints = new bool[numPixels]{false};
    for(size_t i{}; i < numPixelsPerCell; ++i){
        meshExtractPoints.push_back(i);
    }

    fv::CamPinhole::setResolution(0);
 }

void fv::CamPinholeDist::setCalibration(const DATA_TYPE& fxValue,const DATA_TYPE& fyValue,const DATA_TYPE& cxValue,const DATA_TYPE& cyValue){

    cameraMatrix = (cv::Mat1d(3, 3) << fxValue, 0, cxValue, 0, fyValue, cyValue, 0, 0, 1);
    imageSize = cv::Size(h,w);
    cameraMatrixDist = cv::getOptimalNewCameraMatrix(cameraMatrix,distCoeffs,imageSize,0,imageSize,rect,false);

    fx = DATA_TYPE(cameraMatrixDist.at<double>(0,0));
    fy = DATA_TYPE(cameraMatrixDist.at<double>(1,1));
    cx = DATA_TYPE(cameraMatrixDist.at<double>(0,2));
    cy = DATA_TYPE(cameraMatrixDist.at<double>(1,2));

    fxi = 1/fx; fyi = 1/fy; cxi = -cx/fx; cyi = -cy/fy;
    fv::CamPinhole::setGlobalCalibration();

}

#ifndef ROS
void fv::CamPinholeDist::readAndUndistortImage(cv::Mat& image,const std::string& imagePath ){
    cv::Mat imageTemporal = cv::imread(imagePath, CV_LOAD_IMAGE_GRAYSCALE);
    cv::undistort(imageTemporal, image, cameraMatrix, distCoeffs,cameraMatrixDist);
}

void fv::CamPinholeDist::readAndUndistortDepthMap(cv::Mat& depthMap,const std::string& depthMapPath ){
    cv::Mat temporal = cv::imread(depthMapPath, cv::IMREAD_ANYDEPTH);
    cv::Mat R{};
    cv::Mat map1{};
    cv::Mat map2{};
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs,R,cameraMatrixDist, imageSize, CV_32F, map1, map2);
    cv::remap(temporal, depthMap, map1, map2, cv::INTER_NEAREST);
}
#else
void fv::CamPinholeDist::readAndUndistortImage(cv::Mat& image,long long& imageTimestamp,const sensor_msgs::ImageConstPtr& imageMsg ){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::MONO8);
    cv::Mat imageTemporal = cv_ptr->image;
    cv::undistort(imageTemporal, image, cameraMatrix, distCoeffs,cameraMatrixDist);
    imageTimestamp = long(cv_ptr->header.stamp.sec)*1000000000+long(cv_ptr->header.stamp.nsec);
}
void fv::CamPinholeDist::readAndUndistortDepthMap(cv::Mat& depthMap,const sensor_msgs::ImageConstPtr& depthMapMsg){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depthMapMsg, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat temporal = cv_ptr->image;
    cv::Mat R{};
    cv::Mat map1{};
    cv::Mat map2{};
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs,R,cameraMatrixDist, imageSize, CV_32F, map1, map2);
    cv::remap(temporal, depthMap, map1, map2, cv::INTER_NEAREST);
}
#endif

/*
void fv::CamPinholeDist::computeJCam(DATA_TYPE Ju[3],DATA_TYPE Jv[3], fv::Pt pt, size_t i_mask, int i_pyr){
    DATA_TYPE coef,r2;
    DATA_TYPE dcoef_dxn, dcoef_dyn;
    DATA_TYPE ddx_dxn, ddx_dyn, ddy_dxn, ddy_dyn;
    DATA_TYPE dxd_dxn, dxd_dyn, dyd_dxn, dyd_dyn;

    r2 = (pt.xn[i_mask])*(pt.xn[i_mask])+(pt.yn[i_mask])*(pt.yn[i_mask]);
    coef = 1+kc[0]*r2+kc[1]*(r2*r2)+kc[4]*(r2*r2*r2);

    dcoef_dxn = 2*pt.xn[i_mask]*(kc[0]+2*kc[1]*r2+3*kc[4]*r2*r2);
    dcoef_dyn = 2*pt.yn[i_mask]*(kc[0]+2*kc[1]*r2+3*kc[4]*r2*r2);

    ddx_dxn = 2*kc[2]*pt.yn[i_mask]+kc[3]*(6*pt.xn[i_mask]);
    ddx_dyn = 2*kc[2]*pt.xn[i_mask]+kc[3]*(2*pt.yn[i_mask]);
    ddy_dxn = 2*kc[3]*pt.yn[i_mask]+kc[3]*(2*pt.xn[i_mask]);
    ddy_dyn = 2*kc[3]*pt.xn[i_mask]+kc[3]*(6*pt.yn[i_mask]);

    dxd_dxn = (dcoef_dxn*pt.xn[i_mask]+coef+ddx_dxn);
    dxd_dyn = (dcoef_dyn*pt.xn[i_mask]+ddx_dyn);
    dyd_dxn = (dcoef_dxn*pt.yn[i_mask]+ddy_dxn);
    dyd_dyn = (dcoef_dyn*pt.yn[i_mask]+coef+ddy_dyn);

    Ju[0] = (fxG[i_pyr]*dxd_dxn)/pt.zc[i_mask];
    Ju[1] = (fxG[i_pyr]*dxd_dyn)/pt.zc[i_mask];
    Ju[2] = -Ju[0]*pt.xn[i_mask]-Ju[1]*pt.yn[i_mask];

    Jv[0] = (fyG[i_pyr]*dyd_dxn)/pt.zc[i_mask];
    Jv[1] = (fyG[i_pyr]*dyd_dyn)/pt.zc[i_mask];
    Jv[2] = -Jv[0]*pt.xn[i_mask]-Jv[1]*pt.yn[i_mask];

}

void fv::CamPinholeDist::computeJPhotometric(DATA_TYPE& J1,DATA_TYPE&J2,DATA_TYPE&J3, fv::Pt pt, size_t i_mask){
    DATA_TYPE coef,r2;
    DATA_TYPE dcoef_dxn, dcoef_dyn;
    DATA_TYPE ddx_dxn, ddx_dyn, ddy_dxn, ddy_dyn;
    DATA_TYPE dxd_dxn, dxd_dyn, dyd_dxn, dyd_dyn;

    r2 = (pt.xn[i_mask])*(pt.xn[i_mask])+(pt.yn[i_mask])*(pt.yn[i_mask]);
    coef = 1+kc[0]*r2+kc[1]*(r2*r2)+kc[4]*(r2*r2*r2);

    dcoef_dxn = 2*pt.xn[i_mask]*(kc[0]+2*kc[1]*r2+3*kc[4]*r2*r2);
    dcoef_dyn = 2*pt.yn[i_mask]*(kc[0]+2*kc[1]*r2+3*kc[4]*r2*r2);

    ddx_dxn = 2*kc[2]*pt.yn[i_mask]+kc[3]*(6*pt.xn[i_mask]);
    ddx_dyn = 2*kc[2]*pt.xn[i_mask]+kc[3]*(2*pt.yn[i_mask]);
    ddy_dxn = 2*kc[3]*pt.yn[i_mask]+kc[3]*(2*pt.xn[i_mask]);
    ddy_dyn = 2*kc[3]*pt.xn[i_mask]+kc[3]*(6*pt.yn[i_mask]);

    dxd_dxn = (dcoef_dxn*pt.xn[i_mask]+coef+ddx_dxn);
    dxd_dyn = (dcoef_dyn*pt.xn[i_mask]+ddx_dyn);
    dyd_dxn = (dcoef_dxn*pt.yn[i_mask]+ddy_dxn);
    dyd_dyn = (dcoef_dyn*pt.yn[i_mask]+coef+ddy_dyn);

    J1 = ((pt.Gu[i_mask]*fx)*dxd_dxn+(pt.Gv[i_mask]*fy)*dyd_dxn)/(-pt.zc[i_mask]);
    J2 = ((pt.Gu[i_mask]*fx)*dxd_dyn+(pt.Gv[i_mask]*fy)*dyd_dyn)/(-pt.zc[i_mask]);
    J3 = -(J1*pt.xn[i_mask])-(J2*pt.yn[i_mask]);
}

void fv::CamPinholeDist::xyn_2_uv(fv::Pt& pt){
    DATA_TYPE coef,r2,dx,dy;
    DATA_TYPE xd{}, yd{};

    for (size_t i_mask = 0; i_mask < MASK_SIZE; ++i_mask) {

        r2 = (pt.xn[i_mask])*(pt.xn[i_mask])+(pt.yn[i_mask])*(pt.yn[i_mask]);
        coef = 1+kc[0]*r2+kc[1]*(r2*r2)+kc[4]*(r2*r2*r2);
        dx = 2*kc[2]*pt.xn[i_mask]*pt.yn[i_mask]+kc[3]*(r2+2*pt.xn[i_mask]*pt.xn[i_mask]);
        dy = kc[2]*(r2+2*pt.yn[i_mask]*pt.yn[i_mask])+2*kc[3]*pt.xn[i_mask]*pt.yn[i_mask];

        xd = coef*pt.xn[i_mask] + dx;
        yd = coef*pt.yn[i_mask] + dy;

        pt.u[i_mask] = fx * xd + cx;
        pt.v[i_mask] = fy * yd + cy;

    }

}

bool fv::CamPinholeDist::xyn_2_uv_isPointIn(fv::Pt& pt, int pyrLevel){
    DATA_TYPE coef,r2,dx,dy;
    DATA_TYPE xd{}, yd{};

    for (size_t i_mask = 0; i_mask < MASK_SIZE; ++i_mask) {

        r2 = (pt.xn[i_mask])*(pt.xn[i_mask])+(pt.yn[i_mask])*(pt.yn[i_mask]);
        coef = 1+kc[0]*r2+kc[1]*(r2*r2)+kc[4]*(r2*r2*r2);
        dx = 2*kc[2]*pt.xn[i_mask]*pt.yn[i_mask]+kc[3]*(r2+2*pt.xn[i_mask]*pt.xn[i_mask]);
        dy = kc[2]*(r2+2*pt.yn[i_mask]*pt.yn[i_mask])+2*kc[3]*pt.xn[i_mask]*pt.yn[i_mask];

        xd = coef*pt.xn[i_mask] + dx;
        yd = coef*pt.yn[i_mask] + dy;

        pt.u[i_mask] = fxG[pyrLevel] * xd + cxG[pyrLevel];
        pt.v[i_mask] = fyG[pyrLevel] * yd + cyG[pyrLevel];

//        pt.up[i_mask] = int(std::round(pt.u[i_mask]));
     //   pt.vp[i_mask] = int(std::round(pt.v[i_mask]));
    }

    return isPointIn(pt);
}

void fv::CamPinholeDist::xyn_2_uv_ref(fv::Pt &pt){
    DATA_TYPE coef,r2,dx,dy;
    DATA_TYPE xd{}, yd{};

    for (size_t i_mask = 0; i_mask < MASK_SIZE; ++i_mask) {

        r2 = (pt.xn_ref[i_mask])*(pt.xn_ref[i_mask])+(pt.yn_ref[i_mask])*(pt.yn_ref[i_mask]);
        coef = 1+kc[0]*r2+kc[1]*(r2*r2)+kc[4]*(r2*r2*r2);
        dx = 2*kc[2]*pt.xn_ref[i_mask]*pt.yn_ref[i_mask]+kc[3]*(r2+2*pt.xn_ref[i_mask]*pt.xn_ref[i_mask]);
        dy = kc[2]*(r2+2*pt.yn_ref[i_mask]*pt.yn_ref[i_mask])+2*kc[3]*pt.xn_ref[i_mask]*pt.yn_ref[i_mask];

        xd = coef*pt.xn_ref[i_mask] + dx;
        yd = coef*pt.yn_ref[i_mask] + dy;

        pt.u_ref[i_mask] = fxG[0] * xd + cxG[0];
        pt.v_ref[i_mask] = fyG[0] * yd + cyG[0];

//        pt.up_ref[i_mask] = int(std::round(pt.u_ref[i_mask]));
  //      pt.vp_ref[i_mask] = int(std::round(pt.v_ref[i_mask]));
    }
}

void fv::CamPinholeDist::uv_2_xyn(std::vector<Pt>* points){
    for (fv::Pt& pt: *points){
        CamPinholeDist::uv_2_xyn(pt);
    }
}

void fv::CamPinholeDist::uv_2_xyn(fv::Pt& pt){

    DATA_TYPE xn, yn, xdRef, ydRef, xd, yd;;
    DATA_TYPE coef,r2,dx,dy;
    DATA_TYPE dcoef_dxn, dcoef_dyn;
    DATA_TYPE ddx_dxn, ddx_dyn, ddy_dxn, ddy_dyn;
    Eigen::VectorXf e(2);
    Eigen::Matrix2f J;
    Eigen::Matrix2f A;
    Eigen::VectorXf b(2);
    Eigen::VectorXf delta(2);

    for (size_t i_mask = 0; i_mask < MASK_SIZE; ++i_mask) {
        xdRef = fxi*pt.u_ref[i_mask] + cxi;
        ydRef = fyi*pt.v_ref[i_mask] + cyi;
        xn = xdRef;
        yn = ydRef;

        for(size_t iteration{}; iteration < 10; ++iteration){

            r2 = (xn*xn)+(yn*yn);
            coef = 1+kc[0]*r2+kc[1]*(r2*r2)+kc[4]*(r2*r2*r2);
            dx = 2*kc[2]*xn*yn+kc[3]*(r2+2*xn*xn);
            dy = kc[2]*(r2+2*yn*yn)+2*kc[3]*xn*yn;

            xd = coef*xn + dx;
            yd = coef*yn + dy;

            e(0) = xdRef-xd;
            e(1) = ydRef-yd;

            dcoef_dxn = 2*xn*(kc[0]+2*kc[1]*r2+3*kc[4]*r2*r2);
            dcoef_dyn = 2*yn*(kc[0]+2*kc[1]*r2+3*kc[4]*r2*r2);

            ddx_dxn = 2*kc[2]*yn+kc[3]*(6*xn);
            ddx_dyn = 2*kc[2]*xn+kc[3]*(2*yn);
            ddy_dxn = 2*kc[3]*yn+kc[3]*(2*xn);
            ddy_dyn = 2*kc[3]*xn+kc[3]*(6*yn);

            J(0,0) = -(dcoef_dxn*xn+coef+ddx_dxn);
            J(0,1) = -(dcoef_dyn*xn+ddx_dyn);
            J(1,0) = -(dcoef_dxn*yn+ddy_dxn);
            J(1,1) = -(dcoef_dyn*yn+coef+ddy_dyn);

            A = J.transpose()*J;
            b = -J.transpose()*e;

            delta = A.colPivHouseholderQr().solve(b); // QR decomposition
            xn += delta(0);
            yn += delta(1);

        }
        pt.xn_ref[i_mask] = xn;
        pt.yn_ref[i_mask] = yn;
    }
}
*/

void fv::CamPinholeDist::show_calibration(){

    std::cout << "\nShow CamPinholeDist calibration" << std::endl;
    std::cout <<std::setw(10)<< "pyrLevel"<<std::setw(10)<< "fx" << std::setw(10)<< "fy" << std::setw(10)<< "cx"<< std::setw(10)<< "cy" <<
              std::setw(10)<< "wG" << std::setw(10)<< "hG" << std::endl;

    for(size_t i = 0; i < pyrLevelsUsed; ++i)
    {
        std::cout << std::setw(10) << i << std::setw(10) << fxG[i] <<  std::setw(10) << fyG[i] << std::setw(10) << cxG[i] << std::setw(10) << cyG[i]
                  << std::setw(10) << wG[i] << std::setw(10) << hG[i]<< std::endl;
    }

    std::cout << "\n" << std::endl;

    std::cout <<std::setw(10)<< "k1"<<std::setw(10)<< "k2" << std::setw(10)<< "k3" << std::setw(10)<< "k4"<< std::setw(10)<< "k5" << std::endl;
    std::cout <<std::setw(10)<< distCoeffs.at<DATA_TYPE>(0)<<std::setw(10)<< distCoeffs.at<DATA_TYPE>(1) << std::setw(10)<<  distCoeffs.at<DATA_TYPE>(2) <<
            std::setw(10)<<  distCoeffs.at<DATA_TYPE>(3)<< std::setw(10)<<  distCoeffs.at<DATA_TYPE>(4) << std::endl;
}
