//
// Created by changandao on 15.12.18.
//

#include "myslam/pyrframes.h"

namespace myslam
{


    // namespace PyrFrames
    PyrFrames::PyrFrames()
            :frame_(nullptr),camera_(nullptr),num_levels_(-1)
    {
        //camera_->scale(0.5);
    }

    PyrFrames::PyrFrames(Frame::Ptr base_frame, int num_levels)
            :frame_(base_frame),camera_(base_frame->camera_),num_levels_(num_levels)
    {
        //camera_->scale(0.5);
    }


    PyrFrames::~PyrFrames() {

    }


    PyrFrames::Ptr PyrFrames::createFrame()
    {
        //static long factory_id = 0;
        return PyrFrames::Ptr( new PyrFrames() );
    }

    void PyrFrames::initilize(const Frame::Ptr frame, int num_levels)
    {
        levels_.clear();
        frame_ = frame;
        num_levels_= num_levels;
        levels_.push_back(frame_);
        camera_ = frame_->camera_;

    }

    template<typename T>
    void PyrFrames::pyrDownMeanSmooth(const cv::Mat& in, cv::Mat& out)
    {
        out.create(cv::Size(in.size().width / 2, in.size().height / 2), in.type());

        for(int y = 0; y < out.rows; ++y)
        {
            for(int x = 0; x < out.cols; ++x)
            {
                int x0 = x * 2;
                int x1 = x0 + 1;
                int y0 = y * 2;
                int y1 = y0 + 1;

                out.at<T>(y, x) = (T) ( (in.at<T>(y0, x0) + in.at<T>(y0, x1) + in.at<T>(y1, x0) + in.at<T>(y1, x1)) / 4.0f );
            }
        }
    }

    template<typename T>
    void PyrFrames::pyrDownSubsample(const cv::Mat& in, cv::Mat& out)
    {
        out.create(cv::Size(in.size().width / 2, in.size().height / 2), in.type());


        for(int y = 0; y < out.rows; ++y)
        {
            for(int x = 0; x < out.cols; ++x)
            {

                out.at<T>(y, x) = in.at<T>(y * 2, x * 2);
            }
        }
    }


    void PyrFrames::build() {
        size_t start = levels_.size();
        for(size_t idx= start; idx<num_levels_; idx++)
        {
            levels_.emplace_back(compute(*levels_[idx-1],*(levels_[idx-1]->camera_),idx));
        }

    }

    Frame::Ptr PyrFrames::compute(const Frame& frame, const Camera& camera, int level)
    {
        Camera::Ptr subcamera = make_shared<Camera>(camera);
        subcamera->scale(0.5f);
        Frame::Ptr subframe = make_shared<Frame>(frame);
        subframe->camera_ = subcamera;
        Mat subtempt;
        pyrDownSubsample<unsigned short>(frame.depth_, subtempt);
        subframe->depth_ = subtempt;
        pyrDownMeanSmooth<cv::Vec3b>(frame.color_, subtempt);
        subframe->color_ = subtempt;
        subframe->getGray();
        return subframe;



//        if(level < 2)
//            return make_shared<Frame>(frame);
//        Camera::Ptr subcamera = make_shared<Camera>(camera);
//        subcamera->scale(0.5f);
//        Frame::Ptr subframe = make_shared<Frame>(frame);
//        subframe->camera_ = subcamera;
//        Mat subdepth;
//        pyrDownSubsample<unsigned short>(frame.depth_, subdepth);
//        subframe->depth_ = subdepth;
//        Mat subcolor;
//        pyrDownMeanSmooth<cv::Vec3b>(frame.color_, subcolor);
//        subframe->getGray();
//        compute(*subframe, *subframe->camera_, level-1);

    }

    Frame& PyrFrames::level(int idx)
    {
        return *levels_[idx];
    }


}



