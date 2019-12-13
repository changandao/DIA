//
// Created by font_al on 4/12/19.
//

#ifndef DEFINESANDINCLUDES_H
#define DEFINESANDINCLUDES_H

/////////////////////////////////////////////////////////////////////////////////// DEFINES
//////////////////////////////////////////// LOGFILES
#define LOGFILES
    #ifdef LOGFILES
        //#define DEBUG
        //#define SYNC_DEPTH_BLOCK_DEBUG
        //#define CINEMATIC_MODEL_BLOCK_DEBUG
        //#define EXCEPTIONS_BLOCK_DEBUG
        //#define GRAPH_DEBUG
        //#define ENTROPY
    #endif
//////////////////////////////////////////// ALLOCATION SETTINGS
#define TYPE_DOUBLE
#define MASK_SIZE 9
#define MASK_SURFACE 4
#define PYR_LEVELS_USED 6
#define NUM_KEYFRAMES_MAX 7
//////////////////////////////////////////// DEPTH MAP
#define KINECT
    #ifndef KINECT
        #define DISPARITY
    #endif
//////////////////////////////////////////// DATASET/ROS
#define DATASET
    #ifdef DATASET
        //#define GT
        //#define  GT_DEBUG
    #else
        #define ROS
    #endif
////////////////////////////////////////////
#define ORIENTED_MASK
#define FEATURES
////////////////////////////////////////////
#define VISUALIZATION
#define PROFILING
/////////////////////////////////////////////////////////////////////////////////// INCLUDES
//////////////////////////////////////////// ROS INCLUDES
#ifdef ROS
#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/String.h"
#include "stereo_msgs/DisparityImage.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#endif
//////////////////////////////////////////// fv_SLAM INCLUDES

#include <iomanip>
#include <fstream>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#ifdef DATASET
    #include <dirent.h>
    #include <sys/stat.h>
#endif

#ifdef PROFILING
    #include <chrono>
#endif
////////////////////////////////////////////////////////// Optimizer choosing
//#define g2o
//#define naive
/////////////////////////////////////////////////////////////////////////////////// DATA TYPES

#ifdef TYPE_DOUBLE

    #define DATA_TYPE double
    #define DATA_TYPE_MAT CV_64F
    #define SIZE_DATA_TYPE_MAT 8
    #define CONVERSION_STRING_TO_DATA_TYPE std::stod

    #define EIGEN_MATRIX_X Eigen::MatrixXd
    #define EIGEN_VECTOR_X Eigen::VectorXd
    #define EIGEN_VECTOR_3 Eigen::Vector3d
    #define EIGEN_ARRAY_X  Eigen::ArrayXXd

    #define SQRT sqrt
    #define LOGARITHM log
    #define ABS fabs
    #define ISNAN isnan
    #define METRIC_UNIT_CONV 1
    #define ACOS acos
    #define COS cos
    #define ASIN asin
    #define SIN sin
    #define ATAN atan
    #define ATAN2 atan2
    #define POW pow
    #define ROUND round
    #define EXP exp

#else

    #define DATA_TYPE float
    #define DATA_TYPE_MAT CV_32F
    #define SIZE_DATA_TYPE_MAT 4
    #define CONVERSION_STRING_TO_DATA_TYPE std::stof

    #define EIGEN_MATRIX_X Eigen::MatrixXf
    #define EIGEN_VECTOR_X Eigen::VectorXf
    #define EIGEN_VECTOR_3 Eigen::Vector3f
    #define EIGEN_ARRAY_X  Eigen::ArrayXXf

    #define SQRT sqrtf
    #define LOGARITHM logf
    #define ABS fabsf
    #define ISNAN isnanf
    #define METRIC_UNIT_CONV 1
    #define ACOS acosf
    #define COS cosf
    #define ASIN asinf
    #define SIN sinf
    #define ATAN atanf
    #define ATAN2 atan2f
    #define POW powf
    #define ROUND roundf
    #define EXP expf

#endif

#endif //DEFINESANDINCLUDES_H
