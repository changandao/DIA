//
// Created by font_al on 10/5/18.
//

#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include "SystemSettings.h"

namespace fv
{
    using VECTOR3 = DATA_TYPE[3];
    using MATRIX3 = DATA_TYPE[9];

    struct Pt {

        // General members
        void *refKeyframe{};
        bool isVisible{true};
        //Geometric members
        DATA_TYPE u_ref[MASK_SIZE]{} , v_ref[MASK_SIZE]{};
        DATA_TYPE xn_ref[MASK_SIZE]{}, yn_ref[MASK_SIZE]{};
        DATA_TYPE lambda_ref{0}, stdLambdaRef{0}, invCovLambdaRef{0};

        DATA_TYPE X[MASK_SIZE]{} , Y[MASK_SIZE]{} , Z[MASK_SIZE]{};

        DATA_TYPE *xc{}, *yc{}, *zc{};
        DATA_TYPE *xn{}, *yn{};
        DATA_TYPE *u{}, *v{};

        //Photometric members
        DATA_TYPE I_ref[MASK_SIZE][PYR_LEVELS_USED]{};
        DATA_TYPE Gu_ref[MASK_SIZE]{};
        DATA_TYPE Gv_ref[MASK_SIZE]{};

        DATA_TYPE* I{};
        DATA_TYPE* Gu{};
        DATA_TYPE* Gv{};

        #ifdef VISUALIZATION
        size_t Jrow{};
        #endif

        void setLambdaRef(DATA_TYPE lambdaRef_){
            lambda_ref = lambdaRef_;
        }

        void setStdLambdaRef(DATA_TYPE stdLambdaRef_ ){
            invCovLambdaRef += (1 / (stdLambdaRef_ * stdLambdaRef_));
            stdLambdaRef = SQRT(1/invCovLambdaRef);

        }

        void setProjectionPointers(){

            I  = new DATA_TYPE[MASK_SIZE];
            Gu = new DATA_TYPE[MASK_SIZE];
            Gv = new DATA_TYPE[MASK_SIZE];

            u  = new DATA_TYPE[MASK_SIZE];
            v  = new DATA_TYPE[MASK_SIZE];

            xn  = new DATA_TYPE[MASK_SIZE];
            yn  = new DATA_TYPE[MASK_SIZE];

            xc  = new DATA_TYPE[MASK_SIZE];
            yc  = new DATA_TYPE[MASK_SIZE];
            zc  = new DATA_TYPE[MASK_SIZE];
        }

        void deleteProjectionPointers(){

            delete [] I;  I  = nullptr;
            delete [] Gu; Gu = nullptr;
            delete [] Gv; Gv = nullptr;

            delete [] u; u  = nullptr;
            delete [] v; v  = nullptr;

            delete [] xn; xn  = nullptr;
            delete [] yn; yn  = nullptr;

            delete [] xc; xc  = nullptr;
            delete [] yc; yc  = nullptr;
            delete [] zc; zc  = nullptr;

        }

        void resetProjectionPointers(){
            deleteProjectionPointers();
            setProjectionPointers();
        }

        void deletePointers(){
            deleteProjectionPointers();
        }
    };
}
#endif //DATA_STRUCTURES_H
