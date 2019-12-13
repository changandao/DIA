//
// Created by font_al on 10/26/18.
//

#include "camera.h"

fv::CamPinhole::CamPinhole(DATA_TYPE fxValue, DATA_TYPE fyValue, DATA_TYPE cxValue, DATA_TYPE cyValue, size_t wValue, size_t hValue){

    fxG  = new DATA_TYPE[pyrLevelsMax]; fyG  = new DATA_TYPE[pyrLevelsMax];
    cxG  = new DATA_TYPE[pyrLevelsMax]; cyG  = new DATA_TYPE[pyrLevelsMax];
    fxiG = new DATA_TYPE[pyrLevelsMax]; fyiG = new DATA_TYPE[pyrLevelsMax];
    cxiG = new DATA_TYPE[pyrLevelsMax]; cyiG = new DATA_TYPE[pyrLevelsMax];
    wG   = new size_t[pyrLevelsMax]   ; hG   = new size_t[pyrLevelsMax];
    numPixelsG = new size_t[pyrLevelsMax];

    fv::CamPinhole::setImageDimensions(wValue,hValue);
    fv::CamPinhole::setCalibration(fxValue, fyValue, cxValue, cyValue);
    fv::Camera::setMask(MASK_SIZE);

    templateExtractPoints = new bool[numPixels]{false};
    for(size_t i{}; i < numPixelsPerCell; ++i){
        meshExtractPoints.push_back(i);
    }

    fv::CamPinhole::setResolution(0);

}

void fv::CamPinhole::setImageDimensions(const size_t& wValue, const size_t& hValue){

    w = wValue;
    h = hValue;

    numPixels = w*h;

    numPixelsPerCell_u = size_t(w/numCellsMesh_u);
    numPixelsPerCell_v = size_t(h/numCellsMesh_v);
    numPixelsPerCell = numPixelsPerCell_u*numPixelsPerCell_v;

    for(int pyrLevel{}; pyrLevel < PYR_LEVELS_USED; ++pyrLevel){
        scaleFactorG[pyrLevel] = POW(pyrLevelScaleSlope,-pyrLevel);
    }

    maskScaleX = DATA_TYPE(w)/640;
    maskScaleY = DATA_TYPE(h)/480;

    u_min = (-0.5 + marginsOfTheImage);
    v_min = (-0.5 + marginsOfTheImage);
}

void fv::CamPinhole::setResolution(const int& pyrLevel_){
    iPyr = pyrLevel_;
    scaleFactor = scaleFactorG[iPyr];

    u_max = wG[pyrLevel_] - (0.5 + marginsOfTheImage);
    v_max = hG[pyrLevel_] - (0.5 + marginsOfTheImage);

    fx = fxG[pyrLevel_];
    fy = fyG[pyrLevel_];
    cx = cxG[pyrLevel_];
    cy = cyG[pyrLevel_];
}

void fv::CamPinhole::computeJCam(DATA_TYPE Ju[3],DATA_TYPE Jv[3], fv::Pt pt, size_t i_mask, int i_pyr){

    Ju[0] = (fxG[i_pyr])/pt.zc[i_mask];
    Ju[1] = 0 ;
    Ju[2] = -Ju[0]*pt.xn[i_mask];
    Jv[0] = 0 ;
    Jv[1] = (fyG[i_pyr])/pt.zc[i_mask];
    Jv[2] = -Jv[1]*pt.yn[i_mask];

}

void fv::CamPinhole::computeJPhotometric(DATA_TYPE& J1,DATA_TYPE&J2,DATA_TYPE&J3, fv::Pt pt, size_t i_mask){

    J1 = (-pt.Gu[i_mask])*(fx)/pt.zc[i_mask];
    J2 = (-pt.Gv[i_mask])*(fy)/pt.zc[i_mask];
    J3 = -(J1*pt.xn[i_mask])-(J2*pt.yn[i_mask]);

}

void fv::CamPinhole::xyn_2_uv(fv::Pt& pt){

    for (size_t i_mask = 0; i_mask < mask_size; ++i_mask) {
        pt.u[i_mask] = fx *  pt.xn[i_mask] + cx;
        pt.v[i_mask] = fy *  pt.yn[i_mask] + cy;
    }

}

bool fv::CamPinhole::xyn_2_uv_isPointIn(fv::Pt& pt, const int pyrLevel){

    for (size_t i_mask = 0; i_mask < mask_size; ++i_mask) {

        pt.u[i_mask] = fxG[pyrLevel] *  pt.xn[i_mask] + cxG[pyrLevel];
        pt.v[i_mask] = fyG[pyrLevel] *  pt.yn[i_mask] + cyG[pyrLevel];
    }

    return isPointIn(pt);
}

void fv::CamPinhole::uv_2_xyn(std::vector<Pt>* points){
    for (fv::Pt& pt: *points){
        uv_2_xyn(pt);
    }
}

void fv::CamPinhole::uv_2_xyn(fv::Pt& pt){

    for (size_t i_mask = 0; i_mask < MASK_SIZE; ++i_mask) {
        // Pinhole model
        pt.xn_ref[i_mask] = fxi*pt.u_ref[i_mask] + cxi;
        pt.yn_ref[i_mask] = fyi*pt.v_ref[i_mask] + cyi;
    }
}

void fv::CamPinhole::compute_uv_coord(int& up, int& vp, const int i_pyr){

    if (up < 0){up = 0;}
    else{
        if (up > wG[i_pyr]-1){up = int(wG[i_pyr]-1);};
    }

    if ( vp < 0){vp = 0;}
    else{
        if( vp > hG[i_pyr]-1){vp = int(hG[i_pyr]-1);};
    }
}


size_t fv::CamPinhole::compute_uv(int up, int vp, const int i_pyr){

    if (up < 0){up = 0;}
    else{
        if (up > wG[i_pyr]-1){up = int(wG[i_pyr]-1);};
    }

    if ( vp < 0){vp = 0;}
    else{
        if( vp > hG[i_pyr]-1){vp = int(hG[i_pyr]-1);};
    }

    size_t uv = vp*wG[i_pyr]+up;

    return uv;
}

int fv::CamPinhole::compute_block(int up, int vp) {
    int i_block{(int(DATA_TYPE(up) / (numCellsMesh_u*scaleFactor)) + int(DATA_TYPE(vp) / (numCellsMesh_v*scaleFactor)) * int(numPixelsPerCell_u))};
    if (i_block < 0) i_block = 0;
    if (i_block > (numPixelsPerCell - 1)) i_block = int(numPixelsPerCell) - 1;

    return i_block;
}

// This function returns if a projected point is inside the limits of the image.
bool fv::CamPinhole::isPointIn(const fv::Pt pt){

    if (pt.zc[0] <  0){
        return false;
    };

    if (pt.u[0] < u_min){
        return false;
    };

    if (pt.v[0] < v_min){
        return false;
    };

    if (pt.u[0] > u_max){
        return false;
    };

    if (pt.v[0] > v_max){
        return false;
    };

    return true;
}

void fv::CamPinhole::show_calibration(){

    std::cout << "\nShow CamPinhole calibration" << std::endl;
    std::cout <<std::setw(10)<< "pyrLevel"<<std::setw(10)<< "fx" << std::setw(10)<< "fy" << std::setw(10)<< "cx"<< std::setw(10)<< "cy" <<
              std::setw(10)<< "wG" << std::setw(10)<< "hG" << std::endl;

    for(size_t i = 0; i < pyrLevelsUsed; ++i)
    {
        std::cout << std::setw(10) << i << std::setw(10) << fxG[i] <<  std::setw(10) << fyG[i] << std::setw(10) << cxG[i] << std::setw(10) << cyG[i]
                  << std::setw(10) << wG[i] << std::setw(10) << hG[i]<< std::endl;
    }

    std::cout << "\n" << std::endl;
}

//
void fv::CamPinhole::setCalibration(const DATA_TYPE& fxValue,const DATA_TYPE& fyValue,const DATA_TYPE& cxValue,const DATA_TYPE& cyValue){
    fx = fxValue; fy = fyValue; cx = cxValue; cy = cyValue;
    fxi = 1/fx; fyi = 1/fy; cxi = -cx/fx; cyi = -cy/fy;
    fv::CamPinhole::setGlobalCalibration();
}

// Function to compute the calibration for all the Pyramid Levels
void fv::CamPinhole::setGlobalCalibration(){

    DATA_TYPE factor{};
    cv::Mat Mat_gray_template(int(w),int(h),cv::DataType<bool>::type);

    for(size_t i = 0; i < pyrLevelsMax; ++i){

        factor = scaleFactorG[i];

        fxG[i] = fx*factor;
        fyG[i] = fy*factor;
        cxG[i] = cx*factor;
        cyG[i] = cy*factor;

        fxiG[i] = fxi*factor;
        fyiG[i] = fyi*factor;
        cxiG[i] = cxi*factor;
        cyiG[i] = cyi*factor;

        wG[i]  = size_t(Mat_gray_template.size[0]);
        hG[i]  = size_t(Mat_gray_template.size[1]);

        numPixelsG[i] = wG[i]*hG[i];

        cv::resize(Mat_gray_template, Mat_gray_template, cv::Size(), 1/pyrLevelScaleSlope, 1/pyrLevelScaleSlope);
    }
}

#ifndef ROS
void fv::CamPinhole::readAndUndistortImage(cv::Mat& image,const std::string& imagePath ){
    image = cv::imread(imagePath, CV_LOAD_IMAGE_GRAYSCALE);
}

void fv::CamPinhole::readAndUndistortDepthMap(cv::Mat& depthMap,const std::string& depthMapPath ){
    depthMap = cv::imread(depthMapPath, cv::IMREAD_ANYDEPTH);
}
#else
void fv::CamPinhole::readAndUndistortImage(cv::Mat& image,long long& imageTimestamp,const sensor_msgs::ImageConstPtr& imageMsg ){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::MONO8);
    image = cv_ptr->image;
    imageTimestamp = long(cv_ptr->header.stamp.sec)*1000000000+long(cv_ptr->header.stamp.nsec);
}
void fv::CamPinhole::readAndUndistortDepthMap(cv::Mat& depthMap,const sensor_msgs::ImageConstPtr& depthMapMsg){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depthMapMsg, sensor_msgs::image_encodings::TYPE_16UC1);
    depthMap = cv_ptr->image;
}
#endif