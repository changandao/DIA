//
// Created by font_al on 10/5/18.
//

#ifndef FRAME_H
#define FRAME_H

#include "camera.h"

namespace fv {

    class Frame {

    private:

    public:

        size_t id{};
        long long timestamp{};

        // Pose transformations
        VECTOR3 t_cw{0,0,0};
        MATRIX3 R_cw{1,0,0,0,1,0,0,0,1};
        VECTOR3 t_wc{0,0,0};
        MATRIX3 R_wc{1,0,0,0,1,0,0,0,1};

        EIGEN_VECTOR_3 v_cw{EIGEN_VECTOR_3::Zero(3)};
        EIGEN_VECTOR_3 w_cw{EIGEN_VECTOR_3::Zero(3)};
        EIGEN_VECTOR_3 v_wc{EIGEN_VECTOR_3::Zero(3)};
        EIGEN_VECTOR_3 w_wc{EIGEN_VECTOR_3::Zero(3)};

        // Images
        Camera* cam{};
        std::vector <cv::Mat> mat_grayG{};
        cv::Mat Mat_gray;

        DATA_TYPE phScalar{};
        DATA_TYPE phBias{};

        //Visualization
        #ifdef VISUALIZATION
        cv::Mat Mat_gray_clone;
        #endif

        // Depth images
        cv::Mat frame_depth{};

        #ifndef ROS
        std::string depthImageAdress{};
        #else
        sensor_msgs::ImageConstPtr depthImageAdress{};
        #endif
        size_t validDepth{0};
        DATA_TYPE lambdaRefMean{};

        // Map
        size_t numHGP_ref{};
        size_t numHGP_refNotInitialized{};
        std::vector<fv::Pt> HGP_ref{};
        std::vector<fv::Pt> HGP_refNotInitialized{};

        #ifdef ENTROPY
        // Entropy
        DATA_TYPE globalEntropy{200};

        EIGEN_VECTOR_X entropy{EIGEN_VECTOR_X::Zero(6)};

        DATA_TYPE entropyTraslation{};
        DATA_TYPE entropyRotation{};
        #endif

        //Constructor
        explicit Frame(Camera* camera_) {
            cam = camera_;
        };

        // Pose and Pixel operations
        void xynlambda_2_XYZ(std::vector<Pt>* points);
        void xynlambda_2_XYZ(fv::Pt& pt);

        void XYZ_2_xynlambda(fv::Pt& pt);
        void XYZ_2_xynlambda(fv::Pt& pt, const VECTOR3 t_cw, const MATRIX3 R_cw);


        void computePhotometricError(EIGEN_VECTOR_X& e, fv::Pt& pt, DATA_TYPE phScalar_ = 0, DATA_TYPE phBias_ = 0);
        DATA_TYPE computePhotometricError(fv::Pt& pt, DATA_TYPE measurement, size_t i_mask,  DATA_TYPE phScalar_ = 0, DATA_TYPE phBias_ = 0);
        void computePhotometricJacobian(fv::Pt& pt, EIGEN_VECTOR_X& J, size_t i_mask);
        void computePhotometricJacobian(fv::Pt& pt, EIGEN_MATRIX_X& J, EIGEN_VECTOR_X& e, EIGEN_VECTOR_X& eAbs, DATA_TYPE ZeroPh = 300, DATA_TYPE HuberPh = 300);

        void extractPts(const size_t& mode, const std::vector<size_t>& meshExtractPoints, bool* templateExtractPoints);

        void extract_I_and_G_reference(std::vector<Pt>* points);
        void extract_I_and_G_reference(fv::Pt& point, int i_pyr);
        void extract_I_and_G(Pt& point);

        void getLambdaRef_static();

        //This function update the pose of the frame.
        void set_T_cw(const VECTOR3 t_cw_, const MATRIX3 R_cw_) {
            std::copy_n(t_cw_,3,t_cw);
            std::copy_n(R_cw_,9,R_cw);

            fv::inv_T(t_wc, R_wc,t_cw ,R_cw);
            fv::log_lie(v_cw,w_cw,t_cw,R_cw);

            v_wc = -v_cw;
            w_wc = -w_cw;
        };

        //This function return the euclidean distance among another frame.
        DATA_TYPE computeDistance(fv::Frame* frame){
            return euclideanDistance(t_wc,frame->t_wc);
        }

        // Checks the point is inside the limits of the image.
        bool isInsideImage(fv::Pt& pt){
            XYZ_2_xynlambda(pt);
            return (cam->xyn_2_uv_isPointIn(pt,cam->iPyr));
        }

        //Checks the point satisfied geometric visual restrictions.
        bool isGeoVisible(fv::Pt& pt){

            if(not isInsideImage(pt))
                return false;

            return true;

            // Checks the point is not too far or too close
            /*if((pt.zc_ref > 3)||(pt.zc[0] > 1.5*pt.zc_ref)||(pt.zc[0] < 0.5*pt.zc_ref))
                return false;

            return true;*/

        };

        // Load image and depth maps functions
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        #ifdef DATASET
        // This function loads an image from a folder given the path.
        void loadImage(const std::string& imagePath, const size_t id_ = 0, const long long timestamp_ = 0) {
            id = id_;
            timestamp = timestamp_;

            cam->readAndUndistortImage(Mat_gray,imagePath);
            #ifdef VISUALIZATION
            Mat_gray_clone = Mat_gray.clone();
            #endif

            cv::Mat m;
            cv::Mat Mat_gray_temporal;
            mat_grayG.clear();
            for(size_t iPyr{0}; iPyr < PYR_LEVELS_USED; ++iPyr){
                //Mat_gray_temporal.release();
                cv::resize(Mat_gray, Mat_gray_temporal, cv::Size(),cam->scaleFactorG[iPyr],cam->scaleFactorG[iPyr]);
                Mat_gray_temporal.convertTo(m, DATA_TYPE_MAT);
                mat_grayG.push_back(m.clone());
                m.release();
                //memcpy(frame_grayG[iPyr], m.data, size_t(SIZE_DATA_TYPE_MAT* m.rows * m.cols));
            }
        };

        //
        void findDepthImageAdress(const std::vector<std::string> &depthList,const std::vector<long long> &depthTimestampList, const DATA_TYPE &RGBD_frequence, size_t &lastDepthIndex ){

            DATA_TYPE minDifference{};
            DATA_TYPE minDifference_i{};
            validDepth = 0;

            for(size_t indexDepth{lastDepthIndex}; indexDepth < indexDepth+2; ++indexDepth){

                if (indexDepth >= depthList.size()) break;
                minDifference_i = ABS(depthTimestampList[indexDepth]-timestamp);
                if((minDifference_i < minDifference)||(validDepth == 0)){
                    minDifference = minDifference_i;
                    depthImageAdress = depthList[indexDepth];
                    if(minDifference_i < RGBD_frequence/2) {
                        lastDepthIndex = indexDepth;
                        validDepth = 1;
                    }
                }
            }

            #ifdef SYNC_DEPTH_BLOCK_DEBUG
            *syncDepthBlockLog << validDepth <<  " , "<< minDifference << "\n";
            #endif
        }

        //This function loads a depth/disparity map from a folder given a path.
        void loadDepthMap() {
            cam->readAndUndistortDepthMap(frame_depth,depthImageAdress);
            frame_depth.convertTo(frame_depth, DATA_TYPE_MAT);
        };
        #endif

        #ifdef ROS
        // This function loads an image from a folder given the path.
        void loadImage(const sensor_msgs::ImageConstPtr& image){
            //id = id_;//?????
            cam->readAndUndistortImage(Mat_gray,timestamp,image);

            #ifdef VISUALIZATION
            Mat_gray_clone = Mat_gray.clone();
            #endif

            cv::Mat Mat_gray_temporal;
            cv::Mat m;
            for(size_t iPyr{0}; iPyr < PYR_LEVELS_USED; ++iPyr){
                Mat_gray_temporal.release();
                m.release();

                cv::resize(Mat_gray, Mat_gray_temporal, cv::Size(),cam->scaleFactorG[iPyr],cam->scaleFactorG[iPyr]);
                Mat_gray_temporal.convertTo(m, DATA_TYPE_MAT);
                memcpy(frame_grayG[iPyr], m.data, size_t(SIZE_DATA_TYPE_MAT* m.rows * m.cols));
            }
        };

        void findDepthImageAdress(const sensor_msgs::ImageConstPtr& depthImageAdress_){

            depthImageAdress = depthImageAdress_;
            validDepth = 1;

            #ifdef SYNC_DEPTH_BLOCK_DEBUG
            *syncDepthBlockLog << validDepth <<  " , "<< minDifference << "\n";
            #endif
        }

        //This function loads a depth/disparity map from a folder given a path.
        void loadDepthMap() {
            cam->readAndUndistortDepthMap(frame_depth,depthImageAdress);
            frame_depth.convertTo(frame_depth, DATA_TYPE_MAT);
        };
        #endif

        // VISUALIZATION FUNCTIONS
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        #ifdef VISUALIZATION
        void drawPointRef(fv::Pt& pt,size_t size_draw_mask = 1, cv::Scalar color = cv::Scalar(0, 255, 0)){
            for(size_t i_mask{}; i_mask < size_draw_mask; ++i_mask) {
                circle(Mat_gray_clone, cv::Point(int(pt.u_ref[i_mask]), int(pt.v_ref[i_mask])), 0, cv::Scalar(255, 0, 0), -1);
            }
        }

        void drawPoint(fv::Pt& pt,size_t size_draw_mask = 1, cv::Scalar color = cv::Scalar(0, 255, 0)){
            for(size_t i_mask{}; i_mask < size_draw_mask; ++i_mask) {
                circle(Mat_gray_clone, cv::Point(int(pt.u[i_mask]), int(pt.v[i_mask])), 0, color , -1);
            }
        }

        void drawPoints(std::vector<fv::Pt*>& points, size_t size_draw_mask = 1){
            for(fv::Pt* pt: points) {
                drawPoint(*pt, size_draw_mask);
            }
        }

        void resizeImage(int iPyr = 0){
            Mat_gray_clone.release();
            DATA_TYPE scale = 1/(pow(2,iPyr));
            cv::resize(Mat_gray.clone(), Mat_gray_clone, cv::Size(), scale,scale);
            //cvtColor(Mat_gray_clone,Mat_gray_clone, cv::COLOR_GRAY2RGB);
        }

        void show_image(const std::string& windowName = "Display Frame") {

            std::cout << "\nShowing frame...  ,id = " << id << " ,timestamp = " << timestamp << std::endl;
            namedWindow(windowName, cv::WINDOW_NORMAL);
            imshow(windowName, Mat_gray_clone);    // Show our image inside it.
            #ifndef ROS
            cv::waitKey(0);
            #else
            cv::waitKey(1);
            #endif
        };

        void draw_points_ref(size_t size_draw_mask = 1) {
            Mat_gray_clone = Mat_gray.clone();
            cvtColor(Mat_gray, Mat_gray_clone, cv::COLOR_GRAY2RGB);

            for (size_t i_pt{}; i_pt < numHGP_ref; ++i_pt) {
                for(size_t i_mask{}; i_mask < size_draw_mask; ++i_mask){
                    circle(Mat_gray_clone, cv::Point(int(HGP_ref[i_pt].u_ref[i_mask]), int(HGP_ref[i_pt].v_ref[i_mask])), 0, cv::Scalar(0, 255, 0), -1);
                }
            }
        };

        void drawDepthMap(){
            Mat_gray_clone = Mat_gray.clone();
            cvtColor(Mat_gray, Mat_gray_clone, cv::COLOR_GRAY2RGB);

            for(fv::Pt pt: HGP_ref){
                if((1/pt.lambda_ref)< 1){
                    circle(Mat_gray_clone, cv::Point(int(pt.u_ref[0]), int(pt.v_ref[0])), 0, cv::Scalar(0, 255, 0), -1);
                }
                else{
                    if((1/pt.lambda_ref) < 4){
                        circle(Mat_gray_clone, cv::Point(int(pt.u_ref[0]), int(pt.v_ref[0])), 0, cv::Scalar(255, 0, 0), -1);
                    }
                    else{
                        circle(Mat_gray_clone, cv::Point(int(pt.u_ref[0]), int(pt.v_ref[0])), 0, cv::Scalar(0, 0, 255), -1);}
                }
            }
            show_image();
        }

        void drawPoints(std::vector<Pt> points) {
            Mat_gray_clone = Mat_gray.clone();
            cvtColor(Mat_gray, Mat_gray_clone, cv::COLOR_GRAY2RGB);

            for (fv::Pt pt : points) {
                circle(Mat_gray_clone, cv::Point(int(pt.u[0]), int(pt.v[0])), 2, cv::Scalar(0, 255, 0), -1);
            }
        };

            void draw_points_and_Jacobians(std::vector<Pt*> pointsForTracking, EIGEN_MATRIX_X& J, size_t i_pyr = 0,size_t Jacobian_dimension = 0,size_t size_draw_mask = 1) {
                Mat_gray_clone.release();
                Mat_gray_clone = Mat_gray.clone();
                DATA_TYPE scale = cam->scaleFactorG[i_pyr];
                cv::resize(Mat_gray.clone(), Mat_gray_clone, cv::Size(), scale,scale);

                cvtColor(Mat_gray_clone,Mat_gray_clone, cv::COLOR_GRAY2RGB);
                double color{255};

                std::vector<DATA_TYPE> Jacobians{},ordered_Jacobians;
                DATA_TYPE maxJ{0}, mean{0},Ji{};

                size_t i_pt = 0;
                for (fv::Pt* pt: pointsForTracking) {

                        if (Jacobian_dimension > 5) {
                            Ji = 0;
                            for (size_t i_J{}; i_J < 6; ++i_J) {
                                Ji += J(pt->Jrow,i_J)*J(pt->Jrow,i_J);
                            }
                            Jacobians.push_back(SQRT(Ji));
                        } else {
                            Jacobians.push_back(std::abs(J(pt->Jrow,Jacobian_dimension)));
                        }
                        mean += Jacobians[i_pt];

                        if (Jacobians[i_pt] > maxJ) {
                            maxJ = Jacobians[i_pt];
                        }
                        ++i_pt;
                }
                ordered_Jacobians = Jacobians;
                std::sort (ordered_Jacobians.begin(), ordered_Jacobians.end());

                DATA_TYPE first_4  = ordered_Jacobians[ordered_Jacobians.size()/4];
                DATA_TYPE second_4 = ordered_Jacobians[2*ordered_Jacobians.size()/4];
                DATA_TYPE third_4  = ordered_Jacobians[3*ordered_Jacobians.size()/4];

                mean = (mean/(Jacobians.size()));

                i_pt = 0;
               // for (std::vector<fv::Pt*>  pointsPerKeyframe: pointsForTracking) {
                //    for (fv::Pt* pt: pointsPerKeyframe) {
                for (fv::Pt* pt: pointsForTracking) {
                        for (size_t i_mask{}; i_mask < size_draw_mask; ++i_mask) {

                            if (Jacobians[i_pt] < first_4) {
                                circle(Mat_gray_clone, cv::Point(int(std::round(pt->u[i_mask])),int(std::round(pt->v[i_mask]))), 0,
                                       cv::Scalar(0, 0, 0), -1);
                            } else {
                                if (Jacobians[i_pt] < second_4) {
                                    circle(Mat_gray_clone, cv::Point(int(std::round(pt->u[i_mask])),int(std::round(pt->v[i_mask]))), 0,
                                           cv::Scalar(255, 0, 0), -1);
                                } else {
                                    if (Jacobians[i_pt] < third_4) {
                                        circle(Mat_gray_clone, cv::Point(int(std::round(pt->u[i_mask])),int(std::round(pt->v[i_mask]))),
                                               0, cv::Scalar(0, 0, 255), -1);
                                    } else {
                                        circle(Mat_gray_clone, cv::Point(int(std::round(pt->u[i_mask])),int(std::round(pt->v[i_mask]))),
                                               0, cv::Scalar(0, 255, 0), -1);
                                    }
                                }
                            }
                            //color = double(255*std::abs(points[i_pt].J[0]/maxJ));
                            //std::cout << "maxJ: " << color<< std::endl;
                        }
                        ++i_pt;
                    //}
                }
            };
        #endif

        // GROUNTRUTH FUNCTIONS
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void compute_error(DATA_TYPE& distance, DATA_TYPE& attitude, VECTOR3 t_wc_gt, MATRIX3 R_wc_gt){
            distance = fv::euclideanDistance(t_wc,t_wc_gt);
            std::cout << "The translation error of Frame " << id <<" is = " << distance*100/METRIC_UNIT_CONV <<" cm " << std::endl;
        }

    };
}
#endif //FRAME_H
