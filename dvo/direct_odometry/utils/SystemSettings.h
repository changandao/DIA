//
// Created by font_al on 10/5/18.
//

#ifndef SETTINGS_H
#define SETTINGS_H

#include "SequenceSettings.h"

#ifdef LOGFILES
extern std::ofstream* gtLog;
extern std::ofstream* entropyLog;
#ifdef SYNC_DEPTH_BLOCK_DEBUG
extern std::ofstream* syncDepthBlockLog;
#endif

#ifdef CINEMATIC_MODEL_BLOCK_DEBUG
extern std::ofstream* cinematicModelBlockLog;
#endif

#ifdef EXCEPTIONS_BLOCK_DEBUG
extern std::ofstream* exceptionsBlockLog;
#endif
#endif

namespace fv{

    class SystemSettings {

    public:

        void* camera{};

        // Extract Points
        size_t extractPointsMode{0};
        DATA_TYPE GminForHgpExtract{20};
        size_t dForExtractHgp{4};

        // Mesh Settings
        size_t numPointsPerCell{1};
        DATA_TYPE numCellsMesh_u{20};
        DATA_TYPE numCellsMesh_v{15};
        DATA_TYPE marginsOfTheImage{2};

        // Resolution pyramid
        DATA_TYPE pyrLevelScaleSlope{2};

        // Canny Edges
        int canny_lowThreshold{100};
        int canny_ratio{3};
        int canny_kernel_size{3};

        //Kinect Sensor
        size_t u_SizeKinectPatch{3};
        size_t v_SizeKinectPatch{3};
        DATA_TYPE depthMax{50};
        DATA_TYPE depthMin{0.01};

        //Cinematic Model
        size_t numTrackedFramesToInitCinematicModel{5};
        DATA_TYPE maxLinearAcceleration{0.0084};
        DATA_TYPE maxAngularAcceleration{0.0059};
        DATA_TYPE maxPhScalarVariability{0.01};
        DATA_TYPE maxPhBiasVariability{20};

        //Point Visibility
        DATA_TYPE maxGradientErrorForVisibility{10};
        DATA_TYPE maxCosAngleForVisibility{1.0472};

        //Is keyframe
        size_t  numKeyframesFrontEnd{7};
        DATA_TYPE minDistanceForCreateKeyframe{0.02};

        size_t numIterationsDistortion{10};
        size_t visiblePointsForLocalOptWindow{0};
        size_t numPointsRefMinForKeyframe{50};
        size_t numHgpPointsInitalization{2000};

        // Cinematic Model

        //Related Keyframes
        DATA_TYPE distanceRelatedKeyframe{0.1};

        SystemSettings() = default;
        void initialize(std::string& patsettingsSystem_yamlFile);
    };
}

extern fv::SystemSettings sysSet;

#endif //SETTINGS_H
