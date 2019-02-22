//
// Created by changandao on 15.12.18.
//

#ifndef PyrFrames_H
#define PyrFrames_H



#include "myslam/common_include.h"
#include "myslam/frame.h"

namespace myslam
{
    class Frame;

    typedef std::shared_ptr<Frame> FramePtr;

    class PyrFrames{
    public:
        typedef std::shared_ptr<PyrFrames> Ptr;
        PyrFrames();
        PyrFrames(const Frame::Ptr base_frame, int num_levels);
        virtual  ~PyrFrames();

        static PyrFrames::Ptr createFrame();

        void initilize(const Frame::Ptr frame, int num_levels);
        Frame::Ptr compute(const Frame& frame, const Camera& camera, int level);
        void build();
        Frame& level(int idx);

        //double timestamp() const;
        Camera::Ptr camera_;     // Pinhole RGBD Camera model
        Frame::Ptr frame_;


    private:
        int num_levels_;

        std::vector<FramePtr> levels_;
        template<typename T>
        static void pyrDownMeanSmooth(const cv::Mat& in, cv::Mat& out);
        template<typename T>
        static void pyrDownSubsample(const cv::Mat& in, cv::Mat& out);



    };
}

#endif //PyrFrames_H
