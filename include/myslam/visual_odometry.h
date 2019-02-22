//
// Created by changandao on 15.12.18.
//

#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/pyrframes.h"

#include <opencv2/features2d/features2d.hpp>

namespace myslam 
{


    enum optimizerOp{
        gradientDecent = 0,
        gaussNewton,
        LevenbergMarquadt
    }optimizerOpt=gaussNewton;
    const int MAX_ITERATION = 200;
    const int NUM_LEVELS =3;

    struct Measurement
    {
        Measurement ( Eigen::Vector3d p, float g );
        Eigen::Vector3d pos_world;
        float grayscale;
    };

    class VisualOdometry
    {
    public:
        typedef shared_ptr<VisualOdometry> Ptr;
        enum VOState {
            INITIALIZING=-1,
            OK=0,
            LOST
        };

        VOState        state_;     // current VO status

        Frame::Ptr     ref_;       // reference key-frame
        Frame::Ptr     curr_;      // current frame
//        PyrFrames::Ptr  pyrframeRef_;
        PyrFrames::Ptr  pyrframe_;

        vector<cv::KeyPoint>    keypoints;
        vector<vector<cv::KeyPoint>>    keypointsLevels;

        vector<Measurement> Points3dwithIntersity;
        vector<vector<Measurement>> Points3dwithIntensityLevels;


        cv::Ptr<cv::FastFeatureDetector> detector;

        SE3 T_c_r_estimated_;    // the estimated pose of current frame
        int num_lost_;           // number of lost times

        // parameters
        double scale_factor_;   // scale in image pyramid
        int max_num_lost_;      // max number of continuous lost times
        double key_frame_min_rot;   // minimal rotation of two key-frames
        double key_frame_min_trans; // minimal translation of two key-frames
        int num_inliers_;

    public: // functions
        VisualOdometry();
        ~VisualOdometry();

        bool addFrame( Frame::Ptr frame );      // add a new frame

    protected:
        // inner operation
        void buildpyramid();
        void extractKeyPoints();
        //deriveAnalytic()
        void setMessurement();
        void naiveOptimizer(optimizerOp optimizer);
        void g2oOptimizer();
        SE3 poseEstimationDirect ( const vector<Measurement>& pointswithintensity, Frame& frame, const SE3& TCW );

        bool checkEstimatedPose();
        bool checkKeyFrame();
        void featureshow();

    };

}

#endif // VISUALODOMETRY_H
