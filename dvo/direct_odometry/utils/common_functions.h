//
// Created by font_al on 10/9/18.
//

#ifndef COMMON_FUNCTIONS_H
#define COMMON_FUNCTIONS_H

#include "data_structures.h"

namespace fv {

    //This function transforms a quaternion q = [qw;qx;qy;qz] into its corresponding rotational matrix.
    inline void quat2rot(MATRIX3& R, DATA_TYPE const q[4]){
        R[0] = q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3]; R[1] = 2*q[1]*q[2]-2*q[0]*q[3];                 R[2] = 2*q[1]*q[3]+2*q[0]*q[2];
        R[3] = 2*q[1]*q[2]+2*q[0]*q[3];                 R[4] = q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3]; R[5] = 2*q[2]*q[3]-2*q[0]*q[1];
        R[6] = 2*q[1]*q[3]-2*q[0]*q[2];                 R[7] = 2*q[2]*q[3]+2*q[0]*q[1];                 R[8] = q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
    }

    inline void assign_Zero_Vector3(MATRIX3& result){
        result[0] = 0; result[1] = 0; result[2] = 0;
    }

    inline void showVector3(const VECTOR3 v) {
        std::cout << v[0]<<" "<< v[1] << " " << v[2] << std::endl;
    }

    inline void showMatrix3(const MATRIX3 M) {
        std::cout << "M = "<<  std::endl;

        std::cout << M[0]<<" "<< M[1] << " " << M[2] << std::endl;
        std::cout << M[3]<<" "<< M[4] << " " << M[5] << std::endl;
        std::cout << M[6]<<" "<< M[7] << " " << M[8] << std::endl;
    }

    inline void assign_Matrix3(MATRIX3& result, const MATRIX3 M){
        std::copy_n(M,9,result);
    }

    inline void assign_Vector3(VECTOR3& result, const VECTOR3 v){
        std::copy_n(v,3,result);}

    inline void transpose3(MATRIX3 Rt, MATRIX3 const R){
        DATA_TYPE aux{};
        Rt[0] =  R[0]; Rt[4] =  R[4]; Rt[8] =  R[8];
        aux = R[1]; Rt[1] =  R[3];Rt[3] =  aux;
        aux = R[2]; Rt[2] =  R[6];Rt[6] =  aux;
        aux = R[5]; Rt[5] =  R[7];Rt[7] =  aux;
    }

    inline void Matrix3_x_vector3(VECTOR3& result, const MATRIX3 M, const VECTOR3 v){
        result[0] = M[0]*v[0]+M[1]*v[1]+M[2]*v[2];
        result[1] = M[3]*v[0]+M[4]*v[1]+M[5]*v[2];
        result[2] = M[6]*v[0]+M[7]*v[1]+M[8]*v[2];
    }

    inline void Vector3_x_Matrix3(VECTOR3& result, const VECTOR3 v, const MATRIX3 M ){
        result[0] = v[0]*M[0]+v[1]*M[3]+v[2]*M[6];
        result[1] = v[0]*M[1]+v[1]*M[4]+v[2]*M[7];
        result[2] = v[0]*M[2]+v[1]*M[5]+v[2]*M[8];
    };

    inline void Matrix3_x_Vector3(VECTOR3& result,const MATRIX3 M, const VECTOR3 v ){
        result[0] = M[0]*v[0]+M[1]*v[1]+M[2]*v[2];
        result[1] = M[3]*v[0]+M[4]*v[1]+M[5]*v[2];
        result[2] = M[6]*v[0]+M[7]*v[1]+M[8]*v[2];
    };

    inline void Matrix3_x_Matrix3(MATRIX3& result, const MATRIX3 M1, const MATRIX3 M2){
        result[0] = M1[0]*M2[0]+M1[1]*M2[3]+M1[2]*M2[6];
        result[1] = M1[0]*M2[1]+M1[1]*M2[4]+M1[2]*M2[7];
        result[2] = M1[0]*M2[2]+M1[1]*M2[5]+M1[2]*M2[8];

        result[3] = M1[3]*M2[0]+M1[4]*M2[3]+M1[5]*M2[6];
        result[4] = M1[3]*M2[1]+M1[4]*M2[4]+M1[5]*M2[7];
        result[5] = M1[3]*M2[2]+M1[4]*M2[5]+M1[5]*M2[8];

        result[6] = M1[6]*M2[0]+M1[7]*M2[3]+M1[8]*M2[6];
        result[7] = M1[6]*M2[1]+M1[7]*M2[4]+M1[8]*M2[7];
        result[8] = M1[6]*M2[2]+M1[7]*M2[5]+M1[8]*M2[8];
    }

    inline void rotate_translate_v(VECTOR3& result, const VECTOR3 v, const MATRIX3 R, const VECTOR3 t){

        result[0] = R[0] * v[0] + R[1] * v[1] + R[2] * v[2] + t[0];
        result[1] = R[3] * v[0] + R[4] * v[1] + R[5] * v[2] + t[1];
        result[2] = R[6] * v[0] + R[7] * v[1] + R[8] * v[2] + t[2];

    };

    inline void inv_T(VECTOR3& t_inv, MATRIX3& R_inv, const VECTOR3 t, const MATRIX3 R){
        transpose3(R_inv,R);
        Matrix3_x_vector3(t_inv,R_inv,t);
        t_inv[0] *= -1; t_inv[1] *= -1; t_inv[2] *= -1;
    }

    inline DATA_TYPE euclideanDistance(const VECTOR3 v1, const VECTOR3 v2) {

        DATA_TYPE distance_i{v1[0] - v2[0]};
        DATA_TYPE distance{distance_i * distance_i};
        distance_i = v1[1] - v2[1];
        distance += (distance_i * distance_i);
        distance_i = v1[2] - v2[2];
        distance += (distance_i * distance_i);

        return SQRT(distance);
    }

    inline DATA_TYPE dotProduct3(const VECTOR3 v1,const VECTOR3 v2) {
        return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2];
    }

    inline void angle3(DATA_TYPE& ang1,DATA_TYPE& ang2,DATA_TYPE& ang3,const VECTOR3 v1,const VECTOR3 v2) {

        ang1 = ((v1[0]*v2[0]+v1[1]*v2[1])/(SQRT(v1[0]*v1[0]+v1[1]*v1[1])*SQRT(v2[0]*v2[0]+v2[1]*v2[1])));
        ang2 = ((v1[0]*v2[0]+v1[2]*v2[2])/(SQRT(v1[0]*v1[0]+v1[2]*v1[2])*SQRT(v2[0]*v2[0]+v2[2]*v2[2])));
        ang3 = ((v1[1]*v2[1]+v1[2]*v2[2])/(SQRT(v1[1]*v1[1]+v1[2]*v1[2])*SQRT(v2[1]*v2[1]+v2[2]*v2[2])));

    }


    inline DATA_TYPE vector3Magnitude(const VECTOR3 v) {
        return SQRT(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    }

    inline DATA_TYPE vector3Magnitude(const DATA_TYPE& v0, const DATA_TYPE& v1 ,const DATA_TYPE& v2) {
        return SQRT(v0*v0+v1*v1+v2*v2);
    }

    inline void exp_lie(VECTOR3& delta_t, MATRIX3& delta_R, const EIGEN_VECTOR_3& v, const EIGEN_VECTOR_3& w) {

        DATA_TYPE w0_2{w[0] * w[0]}, w1_2{w[1] * w[1]}, w2_2{w[2] * w[2]};

        DATA_TYPE theta = SQRT(w0_2 + w1_2 + w2_2);
        DATA_TYPE coef1{1}, coef2{0.5}, coef3{1/6};

        if (theta > 0.0001) {
            coef1 = (SIN(theta)) / (theta);
            coef2 = (1 - COS(theta)) / (theta * theta);
            coef3 = (theta - SIN(theta)) / (theta * theta * theta);
        }

        DATA_TYPE term0, term1, term2;
        /////////////
        term0 = -(w2_2 + w1_2);
        delta_R[0] =  1 + coef2 * term0;
        delta_t[0] = (1 + coef3 * term0) * v[0];
        /////////////
        term0 = -(w2_2 + w0_2);
        delta_R[4] =  1 + coef2 * term0;
        delta_t[1] = (1 + coef3 * term0) * v[1];
        /////////////
        term0 = -(w1_2 + w0_2);
        delta_R[8] =  1 + coef2 * term0;
        delta_t[2] = (1 + coef3 * term0) * v[2];
        /////////////
        term0 = w[0] * w[1];
        term1 = coef1 * w[2];
        term2 = coef2 * term0;
        delta_R[1] = -term1 + term2;
        delta_R[3] =  term1 + term2;
        /////////////
        term1 = coef2 * w[2];
        term2 = coef3 * term0;
        delta_t[0] += (-term1 + term2) * v[1];
        delta_t[1] += ( term1 + term2) * v[0];
        /////////////
        term0 = w[0] * w[2];
        term1 = coef1 * w[1];
        term2 = coef2 * term0;
        delta_R[2] =  term1 + term2;
        delta_R[6] = -term1 + term2;
        /////////////
        term1 = coef2 * w[1];
        term2 = coef3 * term0;
        delta_t[0] +=  (term1 + term2) * v[2];
        delta_t[2] += (-term1 + term2) * v[0];
        /////////////
        term0 = w[1] * w[2];
        term1 = coef1 * w[0];
        term2 = coef2 * term0;
        delta_R[5] = -term1 + term2;
        delta_R[7] =  term1 + term2;
        /////////////
        term1 = coef2 * w[0];
        term2 = coef3 * term0;
        delta_t[1] += (-term1 + term2) * v[2];
        delta_t[2] += ( term1 + term2) * v[1];
    }
    inline void log_lie(EIGEN_VECTOR_3& v, EIGEN_VECTOR_3& w, const VECTOR3 t, const MATRIX3 R) {

        DATA_TYPE theta = ACOS((R[0]+R[4]+R[8]-1)/2);
        DATA_TYPE factor = 0.5;

        if (ABS(theta) > 0.00000000001) {
            factor = theta/(2*SIN(theta));
        }

        w[0] = factor*(R[7]-R[5]);
        w[1] = factor*(R[2]-R[6]);
        w[2] = factor*(R[3]-R[1]);

        DATA_TYPE w0_2{w[0] * w[0]}, w1_2{w[1] * w[1]}, w2_2{w[2] * w[2]};
        DATA_TYPE term0, term1;

        factor = DATA_TYPE(1/12);
        theta = SQRT(w0_2 + w1_2 + w2_2);
        if (ABS(theta) > 0.00000000001) {
            factor = (1-((theta*COS(theta/2))/(2*SIN(theta/2))))/(theta*theta);
        }

        /////////////
        v[0] = (1-factor*(w2_2+w1_2))*t[0];
        v[1] = (1-factor*(w2_2+w0_2))*t[1];
        v[2] = (1-factor*(w1_2+w0_2))*t[2];
        /////////////
        term0 = 0.5f*w[2];
        term1 = factor*(w[0]*w[1]);
        v[0] += ( term0+term1)*t[1];
        v[1] += (-term0+term1)*t[0];
        /////////////
        term0 = 0.5f*w[1];
        term1 = factor*(w[0]*w[2]);
        v[0] += (-term0+term1)*t[2];
        v[2] += ( term0+term1)*t[0];
        /////////////
        term0 = 0.5f*w[0];
        term1 = factor*(w[1]*w[2]);
        v[1] += ( term0+term1)*t[2];
        v[2] += (-term0+term1)*t[1];

    }

    /*inline void mean_std_e(DATA_TYPE& mean_e, DATA_TYPE& std_e, std::vector <std::vector<fv::Pt>> points){
        mean_e = 0;
        std_e = 0;

        size_t numPoints{};
        for (std::vector<fv::Pt> &pointsPerKeyframe: points) {
            for (fv::Pt &pt: pointsPerKeyframe) {
                mean_e += pt.e;
                ++numPoints;
            }
        }
        mean_e /= numPoints;
        for (std::vector<fv::Pt> &pointsPerKeyframe: points) {
            for (fv::Pt &pt: pointsPerKeyframe) {
                std_e += ((pt.e - mean_e) * (pt.e - mean_e));
            }
        }
        std_e /= numPoints;
        std_e = SQRT(std_e);
    }*/

    void grad_function(cv::Mat &Mat_grad_, cv::Mat Mat_gray_);

}

#endif //COMMON_FUNCTIONS_H
