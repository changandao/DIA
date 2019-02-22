//
// Created by changandao on 14.12.18.
//

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include "myslam/g2o_types.h"

namespace myslam
{


    Measurement::Measurement ( Vector3d p, float g ) : pos_world ( p ), grayscale ( g ) {}

    VisualOdometry::VisualOdometry() :
        state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ),  num_lost_ ( 0 ), num_inliers_ ( 0 )
    {
        scale_factor_       = Config::get<double> ( "scale_factor" );
        max_num_lost_       = Config::get<float> ( "max_num_lost" );
        key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
        key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
        pyrframe_ = PyrFrames::createFrame();
        detector = cv::FastFeatureDetector::create();
    }

    VisualOdometry::~VisualOdometry()
    {

    }

    bool VisualOdometry::addFrame ( Frame::Ptr frame )
    {
        switch ( state_ )
        {
        case INITIALIZING:
        {
            state_ = OK;
            curr_ = ref_ = frame;
            pyrframe_->initilize(ref_, NUM_LEVELS);
            buildpyramid();
            //featureshow();
            extractKeyPoints();
            setMessurement();
            break;
        }
        case OK:
        {
            curr_ = frame;
            pyrframe_->initilize(curr_, NUM_LEVELS);
            buildpyramid();
            g2oOptimizer();
            //naiveOptimizer(gaussNewton);
            //featureshow();
            if ( checkEstimatedPose() == true ) // a good estimation
            {
                curr_->T_c_w_ = T_c_r_estimated_ ;
                //curr_->setPose(T_c_r_estimated_);
                Vector6d poseEsetimated = curr_->T_c_w_.log();
                Vector6d poseRef = ref_->T_c_w_.log();

                cout<<"reference pose: "<< poseRef<<endl;
                cout<<"estimated pose: " <<poseEsetimated<<endl;
                //ref_ = curr_;
                num_lost_ = 0;
            }
            else // bad estimation due to various reasons
            {
                num_lost_++;
                if ( num_lost_ > max_num_lost_ )
                {
                    state_ = LOST;
                }
                return false;
            }
            break;
        }
        case LOST:
        {
            cout<<"vo has lost."<<endl;
            break;
        }
        }


        return true;
    }


    void VisualOdometry::extractKeyPoints()
    {
        keypointsLevels.clear();
        for(int i = 0; i<NUM_LEVELS; i++)
        {
            keypoints.clear();
            detector->detect( pyrframe_->level(i).color_, keypoints);
            keypointsLevels.emplace_back(keypoints);
        }

    }

    void VisualOdometry::setMessurement()
    {
        // select the features with depth measurements
        int edge_size = 30;
        Points3dwithIntensityLevels.clear();

        for(int i = 0; i<NUM_LEVELS; i++)
        {
            Points3dwithIntersity.clear();
            for(auto kp:keypointsLevels[i]) {
                if ( kp.pt.x < edge_size || kp.pt.y < edge_size || ( kp.pt.x+edge_size ) >pyrframe_->level(i).gray_.cols || ( kp.pt.y+edge_size ) >pyrframe_->level(i).gray_.rows )
                    continue;
                double d = ref_->findDepth(kp);
                if ( d==0 )
                    continue;
                Vector3d p_cam = pyrframe_->level(i).camera_->pixel2camera(Vector2d(kp.pt.x, kp.pt.y), d);
                auto grayscale = static_cast<double>(pyrframe_->level(i).gray_.at<uchar>(cvRound(kp.pt.y), cvRound(kp.pt.x)));
                Points3dwithIntersity.emplace_back(Measurement(p_cam, grayscale));
            }
            //int m = Points3dwithIntersity.size();

            Points3dwithIntensityLevels.emplace_back(Points3dwithIntersity);
        }

    }

    void VisualOdometry::buildpyramid()
    {
        pyrframe_->build();

        std::cout<<"buildpyrmid done"<<std::endl;
    }

    void VisualOdometry::naiveOptimizer(optimizerOp optimizer)
    {

        T_c_r_estimated_ = curr_->T_c_w_;
        Vector6d poseDelta;
        auto Points3dwithIntersity_ = Points3dwithIntensityLevels[0];
        auto m = int(Points3dwithIntersity_.size());

        // construct jacobian and residual
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> jacobian;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> residual;
        jacobian.setZero(m, 6);
        residual.setZero(m, 1);
        //T_c_r_estimated_ = curr_->T_c_w_;
        double error_last = 1e-10, error = 0.;

        bool inframe = true;
        for(int iter = 0; iter<MAX_ITERATION; iter++)
        {
            for(int i = 0; i< m; i++)
            {

                if(!curr_->isInFrame(curr_->camera_->world2camera(Points3dwithIntersity_[i].pos_world, curr_->T_c_w_)))
                    inframe = false;
                if(! inframe)
                {
                    jacobian.row(i) = Eigen::Matrix<double, 1, 6>::Zero();
                    residual(i) = 0.;
                }
                else
                {
                    jacobian.row(i) = curr_->calcJacobianWithoutT(Points3dwithIntersity_[i].pos_world);
                    residual(i) = curr_->getIntensity(Points3dwithIntersity_[i].pos_world) - Points3dwithIntersity_[i].grayscale;
                }

                if(residual(i) == Invalid) {residual(i) = 0;
                    jacobian.row(i) = Eigen::Matrix<double, 1, 6>::Zero();
                }
                error+=residual(i);
                inframe = true;
            }
            error/=m;
            cout<<"iteration: "<<iter<<"\terror: "<<error<<endl;

            if(fabs(error-error_last)<1e-5)
            {
                break;
            }

            if(optimizer==1)
            {
                poseDelta << (jacobian.transpose()*jacobian).inverse() * jacobian.transpose()*residual;
            }
            else if(optimizer==0)
            {
                poseDelta << -residual.transpose() * jacobian;
                poseDelta.normalize();
                poseDelta = 0.1 * poseDelta;
            }

            SE3 T_c_r_update = Sophus::SE3::exp(poseDelta) * T_c_r_estimated_;
            T_c_r_estimated_ = T_c_r_update;
            curr_->setPose( T_c_r_estimated_);

            error_last = error;


        }


    }


    void VisualOdometry::g2oOptimizer()
    {
        T_c_r_estimated_ = curr_->T_c_w_;

        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 1>> DirectBlock;
        std::unique_ptr<DirectBlock::LinearSolverType> linearSolver(
                new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>);
        std::unique_ptr<DirectBlock> solver_ptr(new DirectBlock(std::move(linearSolver)));

        //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::move(solver_ptr) );
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
                std::move(solver_ptr));

        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);
        auto *pose = new g2o::VertexSE3Expmap();
        pose->setEstimate(g2o::SE3Quat(T_c_r_estimated_.rotation_matrix(), T_c_r_estimated_.translation()));
        pose->setId(0);
        optimizer.addVertex(pose);
        int id = 1;

        for(int i = NUM_LEVELS-1; i > 0; i--) {

            for (const Measurement m: Points3dwithIntensityLevels[i-1]) {
                auto *edge = new EdgeProjectXYZRGBDPoseOnly();
                edge->setVertex(0, pose);
                edge->setMeasurement(m.grayscale);
                //edge->camera_ = curr_->camera_.get();
                edge->point_ = m.pos_world;
                //edge->image_ = &(curr_->gray_);
                edge->frame_ = &(pyrframe_->level(i-1));
                edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
                edge->setId(id++);
                optimizer.addEdge(edge);
                

            }
	
        }
cout << "edges in graph: " << optimizer.edges().size() << endl;
        optimizer.initializeOptimization();
        optimizer.optimize(300);
        T_c_r_estimated_ = SE3(pose->estimate().rotation(), pose->estimate().translation());



    }


    SE3 VisualOdometry::poseEstimationDirect ( const vector<Measurement>& pointswithintensity, Frame& frame, const SE3& TCW )
    {
        cout<<"what is the size of this level's image? rows: "<<frame.gray_.rows<<" and cols: "<<frame.gray_.cols<<endl;
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 1>> DirectBlock;
        std::unique_ptr<DirectBlock::LinearSolverType> linearSolver(
                new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>);
        std::unique_ptr<DirectBlock> solver_ptr(new DirectBlock(std::move(linearSolver)));

//        g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::move(solver_ptr) );
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
                std::move(solver_ptr));

        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        auto *pose = new g2o::VertexSE3Expmap();
        pose->setEstimate(g2o::SE3Quat(TCW.rotation_matrix(), TCW.translation()));
        pose->setId(0);
        optimizer.addVertex(pose);

        int id = 1;
        for (const Measurement m: pointswithintensity) {
            auto *edge = new EdgeProjectXYZRGBDPoseOnly();
            edge->setVertex(0, pose);
            edge->setMeasurement(m.grayscale);
            //edge->camera_ = curr_->camera_.get();
            edge->point_ = m.pos_world;
            //edge->image_ = &(curr_->gray_);
            edge->frame_ = &frame;
            edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
            edge->setId(id++);
            optimizer.addEdge(edge);
        }
        cout << "edges in graph: " << optimizer.edges().size() << endl;
        optimizer.initializeOptimization();
        optimizer.optimize(300);
        SE3 TCWestimated(pose->estimate().rotation(), pose->estimate().translation());
        return TCWestimated;
    }

    void VisualOdometry::featureshow()
    {
        boost::timer timer;
        for(int i = 0; i<3; i++)
        {

            cv::Mat out_img;
            out_img = pyrframe_->level(i).color_;
            cv::resize(out_img, out_img, cv::Size(), 1.0, 1.);
            cv::imshow("origin", out_img);

            cv::Mat out_gray;
            out_gray = pyrframe_->level(i).gray_;
            cv::resize(out_gray, out_gray, cv::Size(), 1.0, 1.);
            cv::imshow("gray", out_gray);
//            cv::resize(out_img, out_img, cv::Size(), 0.5, 0.5);
//            cv::imshow("halfsize", out_img);

            cv::Mat out_depth;
            out_depth = pyrframe_->level(i).depth_;
            cv::resize(out_depth, out_depth, cv::Size(), 1.0, 1.);
            cv::imshow("origin DEPTH", out_depth);
//            cv::resize(out_depth, out_depth, cv::Size(), 0.5, 0.5);
//            cv::imshow("halfsize depth", out_depth);

            cv::waitKey(0);
        }

    }



    bool VisualOdometry::checkEstimatedPose()
    {
        // if the motion is too large, it is probably wrong
        Sophus::Vector6d d = T_c_r_estimated_.log();
        if ( d.norm() > 5.0 )
        {
            cout<<"reject because motion is too large: "<<d.norm()<<endl;
            return false;
        }
        return true;
    }

    //
    bool VisualOdometry::checkKeyFrame()
    {
        Sophus::Vector6d d = T_c_r_estimated_.log();
        Vector3d trans = d.head<3>();
        Vector3d rot = d.tail<3>();
        if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
            return true;
        return false;
    }

}
