//
// Created by font_al on 10/9/18.
//

#include "common_functions.h"

//This function...
void fv::grad_function(cv::Mat &Mat_grad_, cv::Mat Mat_gray_){
    int scale = 1;
    int delta = 0;
    int ddepth = DATA_TYPE_MAT;//CV_32F;
    cv::Mat grad_x, grad_y;

    /// Gradient X
    Scharr( Mat_gray_, grad_x, ddepth, 1, 0, scale, delta, cv::BORDER_DEFAULT );

    /// Gradient Y
    Scharr( Mat_gray_, grad_y, ddepth, 0, 1, scale, delta, cv::BORDER_DEFAULT );

    /// Total Gradient (approximate)
    magnitude(grad_x,grad_y, Mat_grad_ );
    Mat_grad_ = Mat_grad_/32;
    //divide(grad_y,grad_x,im_dir);
}