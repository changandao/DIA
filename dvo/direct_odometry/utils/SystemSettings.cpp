//
// Created by font_al on 11/6/18.
//

#include "SystemSettings.h"

fv::SystemSettings sysSet{};

void fv::SystemSettings::initialize(std::string& patsettingsSystem_yamlFile){

    cv::FileStorage fs_settings =  fv::openFileStorage(patsettingsSystem_yamlFile);
    cv::FileNode n;

    n = fs_settings["extractPoints"];
    if(not(n.empty())){
        extractPointsMode = (int)(n["extractPointsMode"]);
        GminForHgpExtract = (DATA_TYPE)(n["GminForHgpExtract"]);
        dForExtractHgp    = (int)(n["dForExtractHgp"]);
    }

    n = fs_settings["meshSettings"];
    if(not(n.empty())){
        numPointsPerCell    = (int)(n["numPointsPerCell"]);
        numCellsMesh_u      = (DATA_TYPE)(n["numCellsMesh_u"]);
        numCellsMesh_v      = (DATA_TYPE)(n["numCellsMesh_v"]);
        marginsOfTheImage   = (DATA_TYPE)(n["marginsOfTheImage"]);
        if(marginsOfTheImage < MASK_SURFACE){
            marginsOfTheImage  = MASK_SURFACE;
        }
    }

    n = fs_settings["resolutionPyramid"];
    if(not(n.empty())){
        pyrLevelScaleSlope      = (DATA_TYPE)(n["pyrLevelScaleSlope"]);
    }

    n = fs_settings["cannyEdges"];
    if(not(n.empty())){
        canny_lowThreshold = (int)(n["canny_lowThreshold"]);
        canny_ratio        = (int)(n["canny_ratio"]);
        canny_kernel_size  = (int)(n["canny_kernel_size"]);
    }

    n = fs_settings["kinectSensor"];
    if(not(n.empty())){
        u_SizeKinectPatch = (int)(n["u_SizeKinectPatch"]);
        v_SizeKinectPatch = (int)(n["v_SizeKinectPatch"]);
        depthMax  = (DATA_TYPE)(n["depthMax"]);
        depthMin  = (DATA_TYPE)(n["depthMin"]);
    }

    n = fs_settings["cinematicModel"];
    if(not(n.empty())){
        numTrackedFramesToInitCinematicModel = (int)(n["numTrackedFramesToInitCinematicModel"]);
        maxLinearAcceleration  = (DATA_TYPE)(n["maxLinearAcceleration"]);
        maxAngularAcceleration = (DATA_TYPE)(n["maxAngularAcceleration"]);
        maxPhScalarVariability = (DATA_TYPE)(n["maxPhScalarVariability"]);
        maxPhBiasVariability   = (DATA_TYPE)(n["maxPhBiasVariability"]);
    }

    n = fs_settings["pointVisibility"];
    if(not(n.empty())){
        maxGradientErrorForVisibility = (DATA_TYPE)(n["maxGradientErrorForVisibility"]);
        maxCosAngleForVisibility      = (DATA_TYPE)(n["maxCosAngleForVisibility"]);
    }

    n = fs_settings["isKeyframe"];
    if(not(n.empty())){
        numKeyframesFrontEnd          = (int)(n["numKeyframesFrontEnd"]);
        minDistanceForCreateKeyframe  = (DATA_TYPE)(n["minDistanceForCreateKeyframe"]);
    }

    numIterationsDistortion        = (int) fs_settings["numIterationsDistortion"];
    visiblePointsForLocalOptWindow = (int) fs_settings["visiblePointsForLocalOptWindow"];
    numPointsRefMinForKeyframe     = (int) fs_settings["numPointsRefMinForKeyframe"];
    numHgpPointsInitalization      = (int) fs_settings["numHgpPointsInitalization"];
    distanceRelatedKeyframe        = (DATA_TYPE) fs_settings["distanceRelatedKeyframe"];

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//LOG FILES
#ifdef LOGFILES
std::ofstream* gtLog;
    #ifdef ENTROPY
    std::ofstream* entropyLog;
    #endif

    #ifdef SYNC_DEPTH_BLOCK_DEBUG
    std::ofstream* syncDepthBlockLog;
    #endif

    #ifdef CINEMATIC_MODEL_BLOCK_DEBUG
    std::ofstream* cinematicModelBlockLog;
    #endif

    #ifdef EXCEPTIONS_BLOCK_DEBUG
    std::ofstream* exceptionsBlockLog;
    #endif

#endif
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
    std::cout<<"\nDirect Odometry fv Settings: "                                       <<std::endl;

    std::cout<<"\n//Extract points                 "<< std::endl;
    std::cout<<"  extractPointsMode:               "<< extractPointsMode               <<std::endl;
    std::cout<<"  GminForHgpExtract:               "<< GminForHgpExtract               <<std::endl;
    std::cout<<"  dForExtractHgp:                  "<< dForExtractHgp                  <<std::endl;

    std::cout<<"\n//Mesh Settings                  "<< std::endl;
    std::cout<<"  numPointsPerCell:                "<< numPointsPerCell_               <<std::endl;
    std::cout<<"  numCellsMesh_u_:                 "<< numCellsMesh_u_                 <<std::endl;
    std::cout<<"  numCellsMesh_v_:                 "<< numCellsMesh_v_                 <<std::endl;
    std::cout<<"  marginsOfTheImage:               "<< marginsOfTheImage_              <<std::endl;

    std::cout<<"\n//Resolution Pyramid             "<< std::endl;
    std::cout<<"  pyrLevelScaleSlope:              "<< pyrLevelScaleSlope_               <<std::endl;

    std::cout<<"\n//Canny Edges                    "                                   <<std::endl;
    std::cout<<"  canny_lowThreshold:              "<< canny_lowThreshold              <<std::endl;
    std::cout<<"  canny_ratio:                     "<< canny_ratio                     <<std::endl;
    std::cout<<"  canny_kernel_size:               "<< canny_kernel_size               <<std::endl;

    std::cout<<"\n//Kinect Sensor                 "                                    <<std::endl;
    std::cout<<"  u_SizeKinectPatch:              "<< u_SizeKinectPatch                <<std::endl;
    std::cout<<"  v_SizeKinectPatch:              "<< v_SizeKinectPatch                <<std::endl;
    std::cout<<"  depthMax:                       "<< depthMax                         <<std::endl;
    std::cout<<"  depthMin:                       "<< depthMin                         <<std::endl;

    std::cout<<"\n//Cinematic Model               "                                    << std::endl;
    std::cout<<"  numTrackedFramesToInitCinematicModel: "<<numTrackedFramesToInitCinematicModel<<std::endl;
    std::cout<<"  maxLinearAcceleration:          "<< maxLinearAcceleration            <<std::endl;
    std::cout<<"  maxAngularAcceleration:         "<< maxAngularAcceleration           <<std::endl;
    std::cout<<"  maxPhScalarVariability:         "<< maxPhScalarVariability           <<std::endl;
    std::cout<<"  maxPhBiasVariability:           "<< maxPhBiasVariability             <<std::endl;

    std::cout<<"\n//Point Visibility              "                                    << std::endl;
    std::cout<<"  maxGradientErrorForVisibility:  "<< maxGradientErrorForVisibility    <<std::endl;
    std::cout<<"  maxCosAngleForVisibility:       "<< maxCosAngleForVisibility         <<std::endl;

    std::cout<<"\n//Is keyframe                   "                                    << std::endl;
    std::cout<<"  numKeyframesFrontEnd:           "<< numKeyframesFrontEnd             <<std::endl;
    std::cout<<"  minDistanceForCreateKeyframe:   "<< minDistanceForCreateKeyframe     <<std::endl;

    std::cout<<"\nnumIterationsDistortion:         "<< numIterationsDistortion         <<std::endl;
    std::cout<<"  visiblePointsForLocalOptWindow:  "<< visiblePointsForLocalOptWindow  <<std::endl;
    std::cout<<"  numPointsRefMinForKeyframe    :  "<< numPointsRefMinForKeyframe      <<std::endl;
    std::cout<<"  numHgpPointsInitalization     :  "<< numHgpPointsInitalization       <<std::endl;

    std::cout<<"\n//Related Keyframes                "<< std::endl;
    std::cout<<"  distanceRelatedKeyframe       :  "<< distanceRelatedKeyframe         <<std::endl;

    std::cout<<"\n "<< std::endl;
}
*/
