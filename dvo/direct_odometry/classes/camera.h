//
// Created by font_al on 10/26/18.
//

#ifndef CAMERA_H

#define CAMERA_H

#include "../utils/common_functions.h"

namespace fv {

    class Camera {

    private:

    protected:

    public:

        // Image dimensions
        size_t w{}, h{}, numPixels{};
        size_t *wG{}, *hG{}, *numPixelsG{};
        DATA_TYPE marginsOfTheImage{sysSet.marginsOfTheImage};

        // Pyramid Levels
        const int pyrLevelsMax{PYR_LEVELS_USED};
        int pyrLevelsUsed{PYR_LEVELS_USED};
        int iPyr{};

        DATA_TYPE pyrLevelScaleSlope = SQRT(sysSet.pyrLevelScaleSlope);
        DATA_TYPE scaleFactorG[PYR_LEVELS_USED]{};
        DATA_TYPE scaleFactor{};

        //Mask parameters
        DATA_TYPE *mask_u{};
        DATA_TYPE *mask_v{};
        size_t mask_surface{};
        size_t mask_size{MASK_SIZE};
        DATA_TYPE maskScaleX{1};
        DATA_TYPE maskScaleY{1};

        // Extract points
        size_t numPixelsPerCell_u{}, numPixelsPerCell_v{}, numPixelsPerCell{};
        DATA_TYPE numCellsMesh_u{sysSet.numCellsMesh_u}, numCellsMesh_v{sysSet.numCellsMesh_v};
        std::vector<size_t> meshExtractPoints{};

        //
        size_t numRowsMeshTemplate{10};
        size_t numColsMeshTemplate{10};
        std::vector<bool> meshTemplate{};
        bool* templateExtractPoints{};




    public:

        Camera() = default;

        void setMask(size_t mask_identifier);
        void apply_mask_2_point(fv::Pt &pt);

        void templateMeshMode(size_t idMode);
        void resetTemplate();

        virtual void setImageDimensions(const size_t& wValue, const size_t& hValue){};
        virtual void setCalibration(const DATA_TYPE& fxValue,const DATA_TYPE& fyValue,const DATA_TYPE& cxValue,const DATA_TYPE& cyValue){};
        virtual void setGlobalCalibration(){};
        virtual void show_calibration(){};
        virtual void setResolution(const int& pyrLevel_){};

        virtual void xyn_2_uv(fv::Pt &pt){};
        virtual bool xyn_2_uv_isPointIn(fv::Pt &pt, int pyrLevel){};

        virtual void uv_2_xyn(std::vector<Pt>* points){};
        virtual void uv_2_xyn(fv::Pt &pt){};
        virtual bool isPointIn (fv::Pt pt){};

        virtual void compute_uv_coord(int& up, int& vp, const int i_pyr){}
        virtual size_t compute_uv(int up, int vp, int i_pyr){};
        virtual int compute_block(int up, int vp){};

        virtual void extractHGP(  cv::Mat Mat_grad_, std::vector<fv::Pt> &points_, size_t d, const std::vector<size_t>& meshExtractPoints, bool* templateExtractPoints){};

        virtual void computeJPhotometric(DATA_TYPE &J1, DATA_TYPE &J2, DATA_TYPE &J3, fv::Pt pt, size_t i_mask){};
        virtual void computeJCam(DATA_TYPE Ju[3], DATA_TYPE Jv[3],fv::Pt pt, size_t i_mask, int i_pyr){};

        virtual DATA_TYPE getFocalLengthX(){};
        virtual DATA_TYPE getFocalLengthY(){};
        virtual DATA_TYPE getCentrePointX(){};
        virtual DATA_TYPE getCentrePointY(){};

        #ifndef ROS
        virtual void readAndUndistortImage(cv::Mat& image, const std::string& imagePath ){};
        virtual void readAndUndistortDepthMap(cv::Mat& depthMap, const std::string&depthMapPath ){};
        #else
        virtual void readAndUndistortImage(cv::Mat& image,long long& imageTimestamp, const sensor_msgs::ImageConstPtr& imageMsg){};
        virtual void readAndUndistortDepthMap(cv::Mat& depthMap,const sensor_msgs::ImageConstPtr& depthMapMsg){};
        #endif
    };

    class CamPinhole: public Camera{

    protected:

        DATA_TYPE fx{0.0},  fy{0.0},  cx{0.0},  cy{0.0};
        DATA_TYPE fxi{0.0}, fyi{0.0}, cxi{0.0}, cyi{0.0};

        DATA_TYPE *fxG{}, *fyG{}, *cxG{}, *cyG{};
        DATA_TYPE *fxiG{}, *fyiG{}, *cxiG{}, *cyiG{};

        DATA_TYPE u_min{}, u_max{}, v_min{}, v_max{};
    public:

        CamPinhole(DATA_TYPE fxValue, DATA_TYPE fyValue, DATA_TYPE cxValue, DATA_TYPE cyValue, size_t wValue, size_t hValue);
        CamPinhole() = default;

        void setImageDimensions(const size_t& wValue, const size_t& hValue) override;
        void setCalibration(const DATA_TYPE& fxValue,const DATA_TYPE& fyValue,const DATA_TYPE& cxValue,const DATA_TYPE& cyValue) override;
        void setGlobalCalibration() override;
        void show_calibration() override;
        void setResolution(const int& pyrLevel_) override;

        void xyn_2_uv(fv::Pt& pt) override;
        bool xyn_2_uv_isPointIn(fv::Pt &pt, int pyrLevel) override;
        bool isPointIn(fv::Pt pt) override;

        void uv_2_xyn(std::vector<Pt>* points) override;
        void uv_2_xyn(fv::Pt& pt) override;

        void compute_uv_coord(int& up, int& vp, const int i_pyr) override;
        size_t compute_uv(int up, int vp, int i_pyr) override;
        int compute_block(int up, int vp) override;
        
        void extractHGP(cv::Mat Mat_grad_, std::vector<fv::Pt> &points_, size_t d, const std::vector<size_t>& meshExtractPoints, bool* templateExtractPoints) override;

        void computeJPhotometric(DATA_TYPE& J1,DATA_TYPE&J2,DATA_TYPE&J3, fv::Pt pt, size_t i_mask) override;
        void computeJCam(DATA_TYPE Ju[3], DATA_TYPE Jv[3],fv::Pt pt, size_t i_mask, int i_pyr) override;

        DATA_TYPE getFocalLengthX() override {return fx;};
        DATA_TYPE getFocalLengthY() override {return fy;};
        DATA_TYPE getCentrePointX() override {return cx;};
        DATA_TYPE getCentrePointY() override {return cy;};

        #ifndef ROS
        void readAndUndistortImage(cv::Mat& image, const std::string& imagePath ) override;
        void readAndUndistortDepthMap(cv::Mat& depthMap, const std::string&depthMapPath) override;
        #else
        void readAndUndistortImage(cv::Mat& image,long long& imageTimestamp, const sensor_msgs::ImageConstPtr& imageMsg);
        void readAndUndistortDepthMap(cv::Mat& depthMap,const sensor_msgs::ImageConstPtr& depthMapMsg);
        #endif
    };

    class CamPinholeDist: public CamPinhole{

    protected:
        cv::Mat distCoeffs;
        cv::Mat cameraMatrix;
        cv::Mat cameraMatrixDist;
        cv::Size imageSize;
        cv::Rect* rect{};

    public:

        CamPinholeDist(const DATA_TYPE& fxValue, const DATA_TYPE& fyValue, const DATA_TYPE& cxValue, const DATA_TYPE& cyValue,const cv::Mat distCoeffsValue, const size_t& wValue, const size_t& hValue);
        CamPinholeDist() = default;

        void setCalibration(const DATA_TYPE& fxValue,const DATA_TYPE& fyValue,const DATA_TYPE& cxValue,const DATA_TYPE& cyValue) override;

        void show_calibration() override;

        #ifndef ROS
        void readAndUndistortImage(cv::Mat& image, const std::string& imagePath ) override;
        void readAndUndistortDepthMap(cv::Mat& depthMap, const std::string&depthMapPath ) override;
        #else
        void readAndUndistortImage(cv::Mat& image,long long& imageTimestamp, const sensor_msgs::ImageConstPtr& imageMsg);
        void readAndUndistortDepthMap(cv::Mat& depthMap,const sensor_msgs::ImageConstPtr& depthMapMsg);
        #endif

        /*void xyn_2_uv(fv::Pt& pt) override;
        bool xyn_2_uv_isPointIn(fv::Pt &pt, int pyrLevel) override;
        void xyn_2_uv_ref(fv::Pt &pt);

        void uv_2_xyn(std::vector<Pt>* points) override;
        void uv_2_xyn(fv::Pt& pt) override;

        void computeJPhotometric(DATA_TYPE& J1,DATA_TYPE&J2,DATA_TYPE&J3, fv::Pt pt, size_t i_mask) override;
        void computeJCam(DATA_TYPE Ju[3], DATA_TYPE Jv[3],fv::Pt pt, size_t i_mask, int i_pyr) override;
        */
    };
}
#endif //CAMERA_H
