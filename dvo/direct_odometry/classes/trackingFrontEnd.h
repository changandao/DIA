//
// Created by font_al on 10/15/18.
//

#ifndef TRACKINGFRONTEND_H
#define TRACKINGFRONTEND_H

#include "frame.h"
#include "../test_functions.h"

namespace fv {

    class TrackingFrontEnd {

    private:

    public:

        //Initialization variables
        bool debugTracking{false};
        bool isFrontEndInitialized{false};

        // Keyframes
        std::vector<Frame*>* keyframesForTracking{};
        size_t* numKeyframesForTracking{};

        //Tracking members
        fv::Frame* trackedFrame;
        size_t numTrackedFrames{};

        //Select Points for Tracking
        size_t numPointsPerCell{sysSet.numPointsPerCell};
        std::vector <fv::Pt*> hgpForTracking{};
        size_t numHgpForTracking{};

        //
        DATA_TYPE ePhMean{10000};
        DATA_TYPE HuberPh{};
        DATA_TYPE ZeroPh{};

        bool trackingSucceed{true};
        DATA_TYPE lastPhError{ePhMean};

        #ifdef VISUALIZATION
        EIGEN_MATRIX_X J;
        #endif

        EIGEN_VECTOR_X e;
        EIGEN_VECTOR_X eAbs;

        EIGEN_MATRIX_X A{EIGEN_MATRIX_X::Zero(8,8)};
        EIGEN_VECTOR_X b{EIGEN_VECTOR_X::Zero(8)};
        EIGEN_VECTOR_X delta{EIGEN_VECTOR_X::Zero(8)};

        DATA_TYPE maxRelativePhotoError{2};
        DATA_TYPE minRelativePhotoError{0.5};
        size_t numTries{};

        //Cinematic model
        bool cinematicModelInitialized{false};

        VECTOR3 t_cw_track{0,0,0};
        MATRIX3 R_cw_track{1,0,0,0,1,0,0,0,1};
        EIGEN_VECTOR_3 v_cw_track{EIGEN_VECTOR_3::Zero(3)};
        EIGEN_VECTOR_3 w_cw_track{EIGEN_VECTOR_3::Zero(3)};

        VECTOR3 t_cw_track_pre{0,0,0};
        MATRIX3 R_cw_track_pre{1,0,0,0,1,0,0,0,1};
        EIGEN_VECTOR_3 v_cw_track_pre{EIGEN_VECTOR_3::Zero(3)};
        EIGEN_VECTOR_3 w_cw_track_pre{EIGEN_VECTOR_3::Zero(3)};

        DATA_TYPE phScalar_track{};
        DATA_TYPE phBias_track{};

        EIGEN_VECTOR_3 linearVelocity{EIGEN_VECTOR_3::Zero(3)};
        EIGEN_VECTOR_3 angularVelocity{EIGEN_VECTOR_3::Zero(3)};
        std::vector< EIGEN_VECTOR_3> linearVelocities;
        std::vector< EIGEN_VECTOR_3> angularVelocities;

        EIGEN_MATRIX_X accelerationMax{EIGEN_MATRIX_X::Zero(8,8)};
        EIGEN_MATRIX_X accelerationMaxTrack{EIGEN_MATRIX_X::Zero(8,8)};

        long long  timestamp_initial{};
        long long timestamp_1{0};
        DATA_TYPE inc_t1{0.03f};

        EIGEN_VECTOR_X delta_cw;
        EIGEN_VECTOR_X delta_cw_ref;

        VECTOR3 delta_t{};
        MATRIX3 delta_R{};

        EIGEN_MATRIX_X D;
        EIGEN_MATRIX_X V;

        DATA_TYPE ePhPrevious{}, ePhRelative{};

        #ifdef PROFILING
        fv::functionProfiling trackingProfile{"tracking"};
        fv::functionProfiling addKeyFrameProfile{"addKeyFrame"};
        #endif

        //Constructor
        explicit TrackingFrontEnd(fv::Frame& initialFrame,std::vector<Frame*>* keyframesForTracking_, size_t* numKeyframesForTracking_):
                                    keyframesForTracking{keyframesForTracking_},numKeyframesForTracking{numKeyframesForTracking_}{

            trackedFrame = new fv::Frame{initialFrame};
            trackedFrame->cam->templateMeshMode(0);
            trackedFrame->mat_grayG = initialFrame.mat_grayG;

            addKeyframe();

            DATA_TYPE covLinearAcceleration = 1/(sysSet.maxLinearAcceleration*sysSet.maxLinearAcceleration);
            DATA_TYPE covAngularAcceleration = 1/(sysSet.maxAngularAcceleration*sysSet.maxAngularAcceleration);
            DATA_TYPE covPhScalar = 1/(sysSet.maxPhScalarVariability*sysSet.maxPhScalarVariability);
            DATA_TYPE covPhBias = 1/(sysSet.maxPhBiasVariability*sysSet.maxPhBiasVariability);

            accelerationMax.diagonal() << covLinearAcceleration,covLinearAcceleration,covLinearAcceleration,
                                        covAngularAcceleration,covAngularAcceleration,covAngularAcceleration,
                                        covPhScalar,
                                        covPhBias;

            accelerationMaxTrack = accelerationMax;
        };

        bool trackFrame();
        void cinematicModel();
        void selectPointsForTracking();

        void trackFrameDir();
        void initOptimizationVariables();
        void updateOptimizationVariables();
        void getOptimizeraztionVariablesDif();
        void setOptimizationVariables();

        void solveOptimizationProblem();
        void setPyrLevelOptimization(int iPyr);
        void photometricErrorDistribution();
        void computePhotometricError();
        void computePhotometricJacobian();
        bool trackFrameDirSucceed();
        static void update_T_relative(VECTOR3& t_relative, MATRIX3& R_relative, VECTOR3 delta_t_cw,MATRIX3 delta_R_cw);
        bool cinematicModelConditions();
        bool satisfiesCinematicModelConditions(EIGEN_VECTOR_X& delta_);
        void resetTracking();
        void g2oOpimizer(int iPyr);

        bool isKeyframe();
        void addKeyframe();

        int selectKeyframeToMarginalize();
        void marginalizeKeyframe(const int& iKeyframeToMarginalize);

        void conservativeTrackingParameters(){
            numTries = 0;
            numPointsPerCell += 1;
            trackedFrame->cam->marginsOfTheImage = 3;

            maxRelativePhotoError = 3;
            minRelativePhotoError = 0.1;

            linearVelocities.clear();
            angularVelocities.clear();
            v_cw_track = trackedFrame->v_cw;
            w_cw_track = trackedFrame->w_cw;
            fv::exp_lie(t_cw_track,R_cw_track,v_cw_track,w_cw_track);
        }

        void optimizeTrackingParameters(){
            ++numTries;
            debugTracking = false;
            if((numTries > 5)||(numTrackedFrames == 0)){

                trackedFrame->cam->marginsOfTheImage = 1;
                numPointsPerCell = sysSet.numPointsPerCell;

                accelerationMaxTrack = accelerationMax;
                maxRelativePhotoError = 2;
                minRelativePhotoError = 0.25;
            }
        }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        class TrackingFrontendFail{
        public:
            explicit TrackingFrontendFail(std::string errorMessage = "\nTracking Frontend Failed"): message(errorMessage){};
            virtual ~TrackingFrontendFail() = default;
            virtual std::string what() const {return message;};
        private:
            std::string message;
        };

        class PhotometricErrorFail : public TrackingFrontendFail{
        public:
            explicit PhotometricErrorFail(std::string errorMessage = "\nTracking Frontend Failed: Photometric Error"):TrackingFrontendFail(errorMessage){};
        };

        class SelectPointsForTrackingFail : public TrackingFrontendFail{
        public:
            explicit SelectPointsForTrackingFail(std::string errorMessage = "\nTracking Frontend Failed: Select Points For Tracking Error"):TrackingFrontendFail(errorMessage){};
        };

        class CinematicModelFail : public TrackingFrontendFail{
        public:
            explicit CinematicModelFail(std::string errorMessage = "\nTracking Frontend Failed: Cinematic Model Error"):TrackingFrontendFail(errorMessage){};
        };

        SelectPointsForTrackingFail selectPointsForTrackingFail;
        PhotometricErrorFail photometricErrorFail;
        CinematicModelFail cinematicModelFail;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef LOGFILES

        void writeLogPose(){
            Eigen::Matrix3f mat;
            mat(0, 0) = trackedFrame->R_wc[0];
            mat(0, 1) = trackedFrame->R_wc[1];
            mat(0, 2) = trackedFrame->R_wc[2];
            mat(1, 0) = trackedFrame->R_wc[3];
            mat(1, 1) = trackedFrame->R_wc[4];
            mat(1, 2) = trackedFrame->R_wc[5];
            mat(2, 0) = trackedFrame->R_wc[6];
            mat(2, 1) = trackedFrame->R_wc[7];
            mat(2, 2) = trackedFrame->R_wc[8];
            Eigen::Quaternionf q(mat);

            *gtLog << std::setprecision(16) <<
            double(trackedFrame->timestamp)*seqSet.timeUnitToSeconds
                << std::setprecision(5) <<
                " " << trackedFrame->t_wc[0] <<
                " " << trackedFrame->t_wc[1] <<
                " " << trackedFrame->t_wc[2] <<
                " " << q.x() <<
                " " << q.y() <<
                " " << q.z() <<
                " " << q.w() << "\n";
        };

        #ifdef ENTROPY

        void writeLogEntropy(){
            *entropyLog <<
                trackedFrame->id<<
                std::setprecision(5)<<
                " " << trackedFrame->globalEntropy<<
                " " << trackedFrame->entropy[0]<<
                " " << trackedFrame->entropy[1]<<
                " " << trackedFrame->entropy[2]<<
                " " << trackedFrame->entropy[3]<<
                " " << trackedFrame->entropy[4]<<
                " " << trackedFrame->entropy[5]<<
                " " << trackedFrame->entropyTraslation<<
                " " << trackedFrame->entropyRotation <<"\n";
        }
        #endif
    #endif

    #ifdef ENTROPY
    void computeEntropy(const EIGEN_MATRIX_X& COV, std::vector <size_t > iDOF_Vector);
    #endif

    };

}



#endif //TRACKINGFRONTEND_H
