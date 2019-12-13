//
// Created by font_al on 1/11/19.
//

#ifndef DVO_H
#define DVO_H

#include "trackingFrontEnd.h"
#include "backEnd.h"

namespace fv{

    class DVO {

    public:

        bool DVOinitialized{false};

        fv::TrackingFrontEnd* trackingFrontEnd{};
        fv::BackEnd* backEnd{};

        fv::Camera* camera{};

        std::vector<Frame*> keyframes{};
        size_t numKeyframes{};

        DVO(){
            initializeCameras();
            camera->show_calibration();
        };

        void initializeDVO(fv::Frame& frame){
            initializeFrontEnd(frame);
            initializeBackEnd();
            DVOinitialized = true;
        };

        void initializeFrontEnd(fv::Frame& frame){
            trackingFrontEnd = new TrackingFrontEnd(frame,&keyframes,&numKeyframes);
        }

        void initializeBackEnd(){
            backEnd = new BackEnd(&keyframes,&numKeyframes);
        }

        void initializeCameras(){
            cv::FileStorage fs_camera =  fv::openFileStorage(seqSet.camera_yamlFile);
            cv::FileNode n;
            n = fs_camera["camera0"];
            std::string cameraModel = (std::string)(n["model"]);

            if(cameraModel == "Pinhole"){
                camera = new fv::CamPinhole((DATA_TYPE)(n["fx"]), (DATA_TYPE)(n["fy"]),(DATA_TYPE)(n["cx"]), (DATA_TYPE)(n["cy"]) ,(int)(n["w"]),(int)(n["h"]));
            }
            if(cameraModel == "distPinhole"){
                cv::Mat kc = (cv::Mat1d(1,5) << (DATA_TYPE)(n["k1"]),(DATA_TYPE)(n["k2"]),(DATA_TYPE)(n["k3"]),(DATA_TYPE)(n["k4"]),(DATA_TYPE)(n["k5"]));
                camera = new fv::CamPinholeDist((DATA_TYPE)(n["fx"]), (DATA_TYPE)(n["fy"]),(DATA_TYPE)(n["cx"]), (DATA_TYPE)(n["cy"]) ,kc,(int)(n["w"]),(int)(n["h"]));
            }
        }

        #ifdef VISUALIZATION
        void showKeyframes(size_t size_mask_draw);
        cv::Mat makeCanvas(std::vector<cv::Mat>& vecMat, int windowHeight, int nRows);
        #endif
    };

}
#endif //DVO_H
