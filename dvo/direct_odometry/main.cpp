

#include "classes/dvo.h"

int main(int argc, char** argv) {

    std::string systemSettings_yamlFile = argv[1];
    std::string sequenceSettings_yamlFile = argv[2];

    std::cout << "\nHighest version of cpp standard supported by the compiler: "<<__cplusplus<< std::endl;

    //Initialize system settings
    #ifdef LOGFILES
    fv::setLogFiles();
    #endif
    sysSet.initialize(systemSettings_yamlFile);

    //Initialize sequence settings
    seqSet.initialize(sequenceSettings_yamlFile);
    //Create DVO
    fv::DVO dvoSystem{};

    //Tracking process
    #ifdef GT
    DATA_TYPE distanceError{}, orientationError{};
    #endif
    size_t kf_id{};
    fv::Frame keyframe(dvoSystem.camera);

    size_t lastDepthIndex{0};
    for (size_t trFrame_id = seqSet.firstRGB ; trFrame_id < seqSet.lastRGB-3000 ; ++trFrame_id){

        if(dvoSystem.DVOinitialized){
            dvoSystem.trackingFrontEnd->trackedFrame->loadImage(seqSet.imageList[trFrame_id], trFrame_id,seqSet.imageList_timestamp[trFrame_id]);
            dvoSystem.trackingFrontEnd->trackedFrame->findDepthImageAdress(seqSet.depthList,seqSet.depthList_timestamp,seqSet.RGB_frequency,lastDepthIndex);

            dvoSystem.trackingFrontEnd->trackFrame();
            #ifdef GT
            dvoSystem.trackingFrontEnd->trackedFrame->compute_error(distanceError, orientationError,seqSet.gt[trFrame_id].t_wc,seqSet.gt[trFrame_id].R_wc);
            #endif
            if(dvoSystem.trackingFrontEnd->isKeyframe()){
                dvoSystem.trackingFrontEnd->addKeyframe();
            }

            #ifdef VISUALIZATION
            dvoSystem.showKeyframes(MASK_SIZE);
            #endif
        }
        else{
            //Initialize dvoSistem
            kf_id = trFrame_id;
            keyframe.loadImage(seqSet.imageList[kf_id], kf_id, seqSet.imageList_timestamp[kf_id]);
            keyframe.findDepthImageAdress(seqSet.depthList,seqSet.depthList_timestamp,seqSet.RGB_frequency,lastDepthIndex);

            if(keyframe.validDepth) dvoSystem.initializeDVO(keyframe);

            #ifdef LOGFILES
            *gtLog << std::setprecision(16)<<
                   double(keyframe.timestamp)*seqSet.timeUnitToSeconds
                   << std::setprecision(5)<<
                   " " << 0.0 << " " << 0.0 << " " << 0.0 <<
                   " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << "\n";
            #endif
        }
        std::cout<<"frame = " << dvoSystem.trackingFrontEnd->trackedFrame->id << std::endl;
    }

    #ifdef PROFILING
    dvoSystem.trackingFrontEnd->trackingProfile.functionProfile();
    dvoSystem.trackingFrontEnd->addKeyFrameProfile.functionProfile();
    #endif

    #ifdef LOGFILES
    fv::closeLogFiles();
    #endif

    return 0;
}