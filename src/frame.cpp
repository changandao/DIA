//
// Created by changandao on 13.12.18.
//


#include "myslam/frame.h"
//#include <opencv2/imgproc/imgproc.hpp>
#include <assert.h>

namespace myslam {

    Frame::Frame()
            : id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false) {

    }

    Frame::Frame(long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, Mat color, Mat depth)
            : id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth),
              is_key_frame_(false) {
        //camera->scale(0.5);
    }

    Frame::~Frame() {

    }

    Frame::Ptr Frame::createFrame() {
        static long factory_id = 0;
        return Frame::Ptr(new Frame(factory_id++));
    }

    Frame::Ptr Frame::createFrame(Camera::Ptr newCamera) {
        Frame::Ptr subframe(new Frame());
        subframe->camera_ = newCamera;
        return subframe;
    }

    double Frame::findDepth(const cv::KeyPoint &kp) {
        int x = cvRound(kp.pt.x);
        int y = cvRound(kp.pt.y);
        ushort d = depth_.ptr<ushort>(y)[x];
        if (d != 0) {
            return double(d) / camera_->depth_scale_;
        } else {
            // check the nearby points
            int dx[4] = {-1, 0, 1, 0};
            int dy[4] = {0, -1, 0, 1};
            for (int i = 0; i < 4; i++) {
                d = depth_.ptr<ushort>(y + dy[i])[x + dx[i]];
                if (d != 0) {
                    return double(d) / camera_->depth_scale_;
                }
            }
        }
        return -1.0;
    }

    void Frame::setPose(const SE3 &T_c_w) {
        T_c_w_ = T_c_w;
    }

    Vector3d Frame::getCamCenter() const {
        return T_c_w_.inverse().translation();
    }

    bool Frame::isInFrame(const Vector3d &pt_world) {
        Vector3d p_cam = camera_->world2camera(pt_world, T_c_w_);
        if (p_cam(2, 0) < 0) return false;
        Vector2d pixel = camera_->world2pixel(pt_world, T_c_w_);
        return pixel(0, 0) > 0 && pixel(1, 0) > 0
               && pixel(0, 0) < color_.cols
               && pixel(1, 0) < color_.rows;
    }


    Vector6d Frame::calcJacobianWithoutT(Vector3d ref_pt) {


        Vector3d transformedPt = camera_->world2camera(ref_pt, T_c_w_);
        Vector2d projectedpt = camera_->camera2pixel(transformedPt);

        double x = transformedPt[0];
        double y = transformedPt[1];
        double invz = 1.0 / transformedPt[2];
        double invz_2 = invz * invz;

        float u = projectedpt(0);
        float v = projectedpt(1);

        Vector6d jacobian;
        jacobian.setZero(1, 6);
        Eigen::Matrix<double, 2, 6> jacobian_uv_ksai;

        jacobian_uv_ksai ( 0,0 ) = - x*y*invz_2;
        jacobian_uv_ksai ( 0,1 ) = ( 1+ ( x*x*invz_2 ) );
        jacobian_uv_ksai ( 0,2 ) = - y*invz;
        jacobian_uv_ksai ( 0,3 ) = invz;
        jacobian_uv_ksai ( 0,4 ) = 0;
        jacobian_uv_ksai ( 0,5 ) = -x*invz_2 ;

        jacobian_uv_ksai ( 1,0 ) = - ( 1+y*y*invz_2 );
        jacobian_uv_ksai ( 1,1 ) = x*y*invz_2;
        jacobian_uv_ksai ( 1,2 ) = x*invz;
        jacobian_uv_ksai ( 1,3 ) = 0;
        jacobian_uv_ksai ( 1,4 ) = invz;
        jacobian_uv_ksai ( 1,5 ) = -y*invz_2;

        Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;
        jacobian_pixel_uv ( 0,0 ) = ( getPixelValue (&gray_, u+1,v )-getPixelValue (&gray_, u-1,v ) ) /2 * camera_->fx_;
        jacobian_pixel_uv ( 0,1 ) = ( getPixelValue (&gray_, u,v+1 )-getPixelValue (&gray_, u,v-1 ) ) /2 * camera_->fy_;

        jacobian = jacobian_pixel_uv*jacobian_uv_ksai;

        return jacobian;

    }


    Vector6d Frame::calcJacobianWithT(Vector3d transformedPt) {

        Vector2d projectedpt = camera_->camera2pixel(transformedPt);

        double x = transformedPt[0];
        double y = transformedPt[1];
        double invz = 1.0 / transformedPt[2];
        double invz_2 = invz * invz;

        float u = projectedpt(0);
        float v = projectedpt(1);
 //        jacobian = jacobian_pixel_uv*jacobian_uv_ksai;

//        Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;\

//        double dx = ( getPixelValue (&gray_, u+1,v )-getPixelValue (&gray_, u-1,v ) ) /2 * camera_->fx_;
//        double dy = ( getPixelValue (&gray_, u,v+1 )-getPixelValue (&gray_, u,v-1 ) ) /2 * camera_->fy_ ;
//        Vector6d jacobian;
//        jacobian.setZero(1, 6);

//        double dxfx = dx * camera_->fx_;
//        double dyfy = dy * camera_->fy_;
//
//        jacobian(0) = dxfx * invz;
//        jacobian(1) = dyfy * invz;
//        jacobian(2) = -(dxfx * x + dyfy * y) * invz_2;
//        jacobian(3) = -dxfx * x * y * invz_2 - dyfy * (1 + y * y * invz_2);
//        jacobian(4) = dxfx * (1 + x * x * invz_2) + dyfy * x * y * invz_2;
//        jacobian(5) = (-dxfx * y + dyfy * x) * invz;
//        Vector2d uv = camera_->camera2pixel(xyz_trans);
//
//        float u = uv(0);
//        float v = uv(1);
//
        Vector6d jacobian;
        jacobian.setZero(1, 6);
        Eigen::Matrix<double, 2, 6> jacobian_uv_ksai;

        jacobian_uv_ksai ( 0,0 ) = - x*y*invz_2;
        jacobian_uv_ksai ( 0,1 ) = ( 1+ ( x*x*invz_2 ) );
        jacobian_uv_ksai ( 0,2 ) = - y*invz;
        jacobian_uv_ksai ( 0,3 ) = invz;
        jacobian_uv_ksai ( 0,4 ) = 0;
        jacobian_uv_ksai ( 0,5 ) = -x*invz_2 ;

        jacobian_uv_ksai ( 1,0 ) = - ( 1+y*y*invz_2 );
        jacobian_uv_ksai ( 1,1 ) = x*y*invz_2;
        jacobian_uv_ksai ( 1,2 ) = x*invz;
        jacobian_uv_ksai ( 1,3 ) = 0;
        jacobian_uv_ksai ( 1,4 ) = invz;
        jacobian_uv_ksai ( 1,5 ) = -y*invz_2;

        Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;


        jacobian_pixel_uv ( 0,0 ) = ( getPixelValue (&gray_, u+1,v )-getPixelValue (&gray_, u-1,v ) ) /2 * camera_->fx_;
        jacobian_pixel_uv ( 0,1 ) = ( getPixelValue (&gray_, u,v+1 )-getPixelValue (&gray_, u,v-1 ) ) /2 * camera_->fy_ ;
        jacobian = jacobian_pixel_uv*jacobian_uv_ksai;

        return jacobian;
    }

    double Frame::getIntensity(Vector3d ref_pt) {
        Vector2d projectedpt = camera_->world2pixel(ref_pt, T_c_w_);
        double Intensity = getPixelValue(&gray_, projectedpt(0), projectedpt(1));
        return Intensity;
    }






//                                                                template<typename T>
//    void Frame::derivativeX(const Mat &img, Mat &img_dx) {
//        img_dx.create(img.size(), img.type());
//
//        for (int y = 0; y < img.rows; ++y) {
//            for (int x = 0; x < img.cols; ++x) {
//                int prev = std::max(x - 1, 0);
//                int next = std::min(x + 1, img.cols - 1);
//
//                img_dx.at<T>(y, x) = (T) (img.at<T>(y, next) - img.at<T>(y, prev)) * 0.5f;
//            }
//        }
//    }
//
//    template<typename T>
//    void Frame::derivativeY(const Mat &img, Mat &img_dy) {
//        img_dy.create(img.size(), img.type());
//
//        for (int y = 0; y < img.rows; ++y) {
//            for (int x = 0; x < img.cols; ++x) {
//                int prev = std::max(y - 1, 0);
//                int next = std::min(y + 1, img.rows - 1);
//
//                img_dy.at<T>(y, x) = (T) (img.at<T>(next, x) - img.at<T>(prev, x)) * 0.5f;
//            }
//        }
//    }
//
//    void Frame::derivative()
//    {
//
//        derivativeX<uchar>(gray_, gray_dx_);
//        derivativeY<uchar>(gray_, gray_dy_);
//    }


}


