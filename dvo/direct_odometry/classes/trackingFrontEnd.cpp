//
// Created by font_al on 10/15/18.
//

#include "trackingFrontEnd.h"

//
bool fv::TrackingFrontEnd::trackFrame(){
    #ifdef PROFILING
    trackingProfile.functionBegin();
    #endif
    cinematicModel();
    trackingSucceed = false;


    int numTry{0};
    while(not trackingSucceed){
        if(numTry>5)
            break;
        try {
            numTry++;
            selectPointsForTracking();
            trackFrameDir();
            optimizeTrackingParameters();

            size_t indexPt{};
            for (fv::Pt *pt: hgpForTracking) {
                if (e.segment(indexPt * MASK_SIZE, MASK_SIZE).mean() > ZeroPh) {
                    pt->isVisible = false;
                    ++indexPt;
                }
            }
            #ifdef LOGFILES
            writeLogPose();
            #ifdef ENTROPY
            writeLogEntropy();
            #endif
            #endif

            ++numTrackedFrames;
            #ifdef PROFILING
            trackingProfile.functionEnd();
             #endif
            return trackingSucceed;
        }

        catch (const SelectPointsForTrackingFail &t) {
            std::cout << t.what() << std::endl;
            std::cout << "numHgpForTracking = "<< numHgpForTracking << std::endl;
            //fv::waitFunction("y");
        }
        catch (const PhotometricErrorFail &t) {
            debugTracking = true;
            std::cout << t.what() << std::endl;
            //fv::waitFunction("y");
            resetTracking();
            conservativeTrackingParameters();
        }
        catch (const CinematicModelFail &t) {
            debugTracking = true;
            std::cout << t.what() << std::endl;
            return false;
            //fv::waitFunction("y");
            //resetTracking();
            //conservativeTrackingParameters();
        }
    }
}


void fv::TrackingFrontEnd::cinematicModel(){

    inc_t1 = {DATA_TYPE(trackedFrame->timestamp-timestamp_initial-timestamp_1)*seqSet.timeUnitToSeconds};
    if(inc_t1 <= 0 ){
        throw cinematicModelFail;
    }

    if (numTrackedFrames > sysSet.numTrackedFramesToInitCinematicModel){
        v_cw_track = trackedFrame->v_cw + linearVelocity*inc_t1;
        w_cw_track = trackedFrame->w_cw + angularVelocity*inc_t1;
        fv::exp_lie(t_cw_track,R_cw_track,v_cw_track,w_cw_track);
    }

    timestamp_1 = trackedFrame->timestamp-timestamp_initial;
}


void fv::TrackingFrontEnd::selectPointsForTracking(){


    hgpForTracking.clear();
    numHgpForTracking = 0;

    size_t* meshPointsForTracking;
    fv::Camera* cam{trackedFrame->cam};
    meshPointsForTracking = new size_t[cam->numPixelsPerCell]{};
    int indexMesh{};
    for (fv::Frame* keyframe: *keyframesForTracking) {
        for (fv::Pt &pt_ref: keyframe->HGP_ref) {
            pt_ref.resetProjectionPointers();
            if ((pt_ref.isVisible)&&(trackedFrame->isGeoVisible(pt_ref))) {
                indexMesh = cam->compute_block(int(std::round(pt_ref.u[0])), int(std::round(pt_ref.v[0])));
                if (meshPointsForTracking[indexMesh] < numPointsPerCell) {
                    hgpForTracking.push_back(&pt_ref);
                    meshPointsForTracking[indexMesh] += 1;
                    ++numHgpForTracking;
                    continue;
                }
            }
            pt_ref.deleteProjectionPointers();
        }
    }

    if(numHgpForTracking < 30){
        throw selectPointsForTrackingFail;
    }
}

void fv::TrackingFrontEnd::addKeyframe(){
    #ifdef PROFILING
    addKeyFrameProfile.functionBegin();
    #endif

    int iKeyframeToMarginalize{};
    std::vector <cv::Mat> mat_grayG;
    if(keyframesForTracking->size() >= sysSet.numKeyframesFrontEnd){
        iKeyframeToMarginalize = selectKeyframeToMarginalize();
        marginalizeKeyframe(iKeyframeToMarginalize);
        isFrontEndInitialized = true;
    }

    keyframesForTracking->push_back(new Frame{*trackedFrame});
    fv::Frame* newKeyframe{keyframesForTracking->back()};

    newKeyframe->mat_grayG = trackedFrame->mat_grayG;

    newKeyframe->extractPts(sysSet.extractPointsMode,newKeyframe->cam->meshExtractPoints, newKeyframe->cam->templateExtractPoints);
    newKeyframe->cam->uv_2_xyn(&(newKeyframe->HGP_refNotInitialized));
    newKeyframe->loadDepthMap();
    newKeyframe->getLambdaRef_static();
    newKeyframe->extract_I_and_G_reference(&(newKeyframe->HGP_ref));
    newKeyframe->xynlambda_2_XYZ(&newKeyframe->HGP_ref);

    *numKeyframesForTracking = keyframesForTracking->size();

    #ifdef PROFILING
    addKeyFrameProfile.functionEnd();
    #endif
}

int fv::TrackingFrontEnd::selectKeyframeToMarginalize(){

    DATA_TYPE distance{};
    int iKeyToMarginalize{0};
    DATA_TYPE maxDistanceScore{};
    DATA_TYPE distanceScore[6]{};

    for(size_t iKey{0} ; iKey < *numKeyframesForTracking-1 ; ++iKey){
        for(size_t jKey{0} ; jKey < *numKeyframesForTracking-1; ++jKey){
            distance = (*keyframesForTracking)[iKey]->computeDistance((*keyframesForTracking)[jKey]);
            distanceScore[iKey] += 1/(distance+0.00001);
        }
        distanceScore[iKey] *= SQRT((*keyframesForTracking)[iKey]->computeDistance(keyframesForTracking->back()));
    }
    maxDistanceScore = distanceScore[0];
    for(size_t iKey{1} ; iKey < *numKeyframesForTracking-1 ; ++iKey){
        if(maxDistanceScore < distanceScore[iKey]){
            maxDistanceScore = distanceScore[iKey];
            iKeyToMarginalize = iKey;
        }
    }

    return iKeyToMarginalize;
}

void fv::TrackingFrontEnd::marginalizeKeyframe(const int& iKeyframeToMarginalize){

    fv::Frame* keyframeToMarginalize{(*keyframesForTracking)[iKeyframeToMarginalize]};

    for(fv::Pt pt: keyframeToMarginalize->HGP_ref) {
        pt.deletePointers();
    }

    keyframeToMarginalize->mat_grayG.clear();
    keyframeToMarginalize->frame_depth.release();

    delete keyframeToMarginalize;

    keyframesForTracking->erase(keyframesForTracking->begin()+iKeyframeToMarginalize);
}


bool fv::TrackingFrontEnd::isKeyframe() {

    if(not trackingSucceed)
        return false;

    if(not trackedFrame->validDepth)
        return false;

    DATA_TYPE relativeBrightnessFactor{DATA_TYPE(0.333)*ABS(LOGARITHM(EXP(keyframesForTracking->back()->phScalar-trackedFrame->phScalar)))};

    if(relativeBrightnessFactor > 1){
        return true;
    }

    DATA_TYPE t[3];
    t[0] =   trackedFrame->R_cw[0]*keyframesForTracking->back()->t_wc[0]
             +trackedFrame->R_cw[1]*keyframesForTracking->back()->t_wc[1]
             +trackedFrame->R_cw[2]*keyframesForTracking->back()->t_wc[2]
             +trackedFrame->t_cw[0];
    t[1] =    trackedFrame->R_cw[3]*keyframesForTracking->back()->t_wc[0]
              +trackedFrame->R_cw[4]*keyframesForTracking->back()->t_wc[1]
              +trackedFrame->R_cw[5]*keyframesForTracking->back()->t_wc[2]
              +trackedFrame->t_cw[1];
    t[2] =    trackedFrame->R_cw[6]*keyframesForTracking->back()->t_wc[0]
              +trackedFrame->R_cw[7]*keyframesForTracking->back()->t_wc[1]
              +trackedFrame->R_cw[8]*keyframesForTracking->back()->t_wc[2]
              +trackedFrame->t_cw[2];

    /*t[0] =   keyframesForTracking->back()->R_cw[0]*keyframesForTracking->back()->t_wc[0]
             +keyframesForTracking->back()->R_cw[1]*keyframesForTracking->back()->t_wc[1]
             +keyframesForTracking->back()->R_cw[2]*keyframesForTracking->back()->t_wc[2]
             +trackedFrame->t_cw[0];
    t[1] =    keyframesForTracking->back()->R_cw[3]*keyframesForTracking->back()->t_wc[0]
              +keyframesForTracking->back()->R_cw[4]*keyframesForTracking->back()->t_wc[1]
              +keyframesForTracking->back()->R_cw[5]*keyframesForTracking->back()->t_wc[2]
              +trackedFrame->t_cw[1];
    t[2] =    keyframesForTracking->back()->R_cw[6]*keyframesForTracking->back()->t_wc[0]
              +keyframesForTracking->back()->R_cw[7]*keyframesForTracking->back()->t_wc[1]
              +keyframesForTracking->back()->R_cw[8]*keyframesForTracking->back()->t_wc[2]
              +trackedFrame->t_cw[2];*/

    DATA_TYPE opticalFlow{};
    DATA_TYPE opticalFlowWithoutRotation{};
    DATA_TYPE T_kf{};
    DATA_TYPE u,v,xc,yc,zc,xn,yn;

    for(fv::Pt* pt: hgpForTracking){

        opticalFlow += (pt->u_ref[0]-pt->u[0])*(pt->u_ref[0]-pt->u[0])+(pt->v_ref[0]-pt->v[0])*(pt->v_ref[0]-pt->v[0]);

        xc = ((pt->xn_ref[0]/1/pt->lambda_ref)+t[0]);
        yc = ((pt->yn_ref[0]/1/pt->lambda_ref)+t[1]);
        zc = ((1/pt->lambda_ref)+t[2]);
        xn = xc/zc;
        yn = yc/zc;
        u = xn*trackedFrame->cam->getFocalLengthX()+trackedFrame->cam->getCentrePointX();
        v=  yn*trackedFrame->cam->getFocalLengthY()+trackedFrame->cam->getCentrePointY();

        opticalFlowWithoutRotation += (pt->u_ref[0]-u)*(pt->u_ref[0]-u)+(pt->v_ref[0]-v)*(pt->v_ref[0]-v);
    }

    opticalFlow = SQRT(opticalFlow/hgpForTracking.size());
    opticalFlowWithoutRotation = SQRT(opticalFlowWithoutRotation/hgpForTracking.size());

    T_kf = 0.333*opticalFlow + 0.333*opticalFlowWithoutRotation + relativeBrightnessFactor;

    return T_kf > 3;

}