//
// Created by changandao on 13.12.18.
//

#ifndef FRAME_H
#define FRAME_H

#include "myslam/common_include.h"
#include "myslam/camera.h"
#include <opencv2/imgproc/imgproc.hpp>

namespace myslam {

    typedef Eigen::Matrix<double, 1, Eigen::Dynamic> VectorXd;
    typedef Eigen::Matrix<double, 1, 6> Vector6d;
// forward declare
    class MapPoint;

    class Frame {
    public:
        typedef std::shared_ptr<Frame> Ptr;
        unsigned long id_;         // id of this frame
        double time_stamp_; // when it is recorded
        SE3 T_c_w_;      // transform from world to camera
        Camera::Ptr camera_;     // Pinhole RGBD Camera model
        Mat color_, gray_, depth_; // color and depth image
        // std::vector<cv::KeyPoint>      keypoints_;  // key points in image
        bool is_key_frame_;  // whether a key-frame

    public: // data members
        Frame();

        Frame(long id, double time_stamp = 0, SE3 T_c_w = SE3(), Camera::Ptr camera = nullptr, Mat color = Mat(),
              Mat depth = Mat());

        ~Frame();

        static Frame::Ptr createFrame();
        static Frame::Ptr createFrame(Camera::Ptr newCamera);

        // find the depth in depth map
        double findDepth(const cv::KeyPoint &kp);

        // Get Camera Center
        Vector3d getCamCenter() const;

        void setPose(const SE3 &T_c_w);

        // check if a point is in this frame
        bool isInFrame(const Vector3d &pt_world);

        Vector6d calcJacobianWithT(Vector3d transformedPt);
        Vector6d calcJacobianWithoutT(Vector3d ref_pt);
        double getIntensity(Vector3d ref_pt);

        inline void getGray()
        {
            cvtColor(color_, gray_, cv::COLOR_BGR2GRAY);
        }



    protected:
        template <typename T>
        void derivativeX(const Mat& img, Mat& img_dx);
        template <typename T>
        void derivativeY(const Mat& img, Mat& img_dy);

        inline double getPixelValue (const Mat* image_, float x, float y )
        {

            uchar* data = & image_->data[ int ( y ) * image_->step + int ( x ) ];
            float xx = x - floor ( x );
            float yy = y - floor ( y );
            return float (
                    ( 1-xx ) * ( 1-yy ) * data[0] +
                    xx* ( 1-yy ) * data[1] +
                    ( 1-xx ) *yy*data[ image_->step ] +
                    xx*yy*data[image_->step+1]
            );
        }
//        double bilinear(const Mat& img, double x, double y);
//        double bilinearWithDepthBuffer(const cv::Mat& intensity, const cv::Mat& depth, double x, double y, double z);

    };




}/*namespace myslam*/

#endif // FRAME_H
