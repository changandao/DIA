//
// Created by font_al on 10/26/18.
//

#include "camera.h"


void fv::Camera::apply_mask_2_point(fv::Pt& pt){
    for (size_t i_mask = 1; i_mask < MASK_SIZE; ++i_mask) {
        pt.u_ref[i_mask] = pt.u_ref[0] + mask_u[i_mask];
        pt.v_ref[i_mask] = pt.v_ref[0] + mask_v[i_mask];
    }
}

void fv::Camera::setMask(size_t mask_identifier){

    if (mask_identifier > MASK_SIZE) {
        mask_identifier = MASK_SIZE;
    }

    if (mask_identifier <= 0) {
        mask_identifier = 1;
    }

    delete mask_u; mask_u = new DATA_TYPE[MASK_SIZE];
    delete mask_v; mask_v = new DATA_TYPE[MASK_SIZE];

    switch (mask_identifier)
    {
        case 1:{
            mask_u[0] = 0; mask_v[0] = 0;
            mask_surface = MASK_SURFACE;
            break;
        }

        case 5: {
            mask_u[0] = 0;  mask_v[0] = 0;  mask_u[1] = -1; mask_v[1] = -1; mask_u[2] = +1; mask_v[2] = -1;
            mask_u[3] = -1; mask_v[3] = +1; mask_u[4] = +1; mask_v[4] = +1;

            mask_surface = MASK_SURFACE;

            break;
        }

        case 7:{

            mask_u[0] = 0; mask_v[0] = 0;  mask_u[1] = -1;mask_v[1] = -1; mask_u[2] = +1;mask_v[2] = -1;
            mask_u[3] = +1;mask_v[3] = +1; mask_u[4] = -1;mask_v[4] = +1; mask_u[5] = -2 ;mask_v[5] = 0;
            mask_u[6] = +2 ;mask_v[6] = 0;

            mask_surface = MASK_SURFACE;

            break;
        }

        case 9:{

            mask_u[0] = 0; mask_v[0] = 0;  mask_u[1] = -1;mask_v[1] = -1; mask_u[2] = +1;mask_v[2] = -1;
            mask_u[3] = -1;mask_v[3] = +1; mask_u[4] = +1;mask_v[4] = +1; mask_u[5] = 0 ;mask_v[5] = -2;
            mask_u[6] = +2 ;mask_v[6] = 0; mask_u[7] = 0 ;mask_v[7] = +2; mask_u[8] = -2 ;mask_v[8] = 0;
            mask_surface = MASK_SURFACE;

            break;
        }

        default:{
            mask_u[0] = 0; mask_v[0] = 0;
            mask_surface = MASK_SURFACE;
            break;
        }
    }

    for (size_t iMask{}; iMask < MASK_SIZE; ++iMask){
        mask_u[iMask] = mask_u[iMask] * maskScaleX;
        mask_v[iMask] = mask_v[iMask] * maskScaleY;
    }
}

void fv::Camera::templateMeshMode(size_t idMode = 0){
    meshTemplate.clear();
    switch (idMode)
    {
        case 0:{
            for(size_t iMesh{}; iMesh < numPixelsPerCell; ++iMesh)
                meshTemplate.push_back(true);
            break;
        }
        case 1:{
            for(size_t iMesh{}; iMesh < numPixelsPerCell; ++iMesh)
                meshTemplate.push_back(false);

            for(size_t iMesh{}; iMesh < numRowsMeshTemplate*numPixelsPerCell_u; ++iMesh){
                meshTemplate[iMesh] = true;
                meshTemplate[iMesh+(numPixelsPerCell_v-numRowsMeshTemplate)*numPixelsPerCell_u] = true;
            }
            for(size_t iMesh{}; iMesh < numPixelsPerCell_v; ++iMesh){
                for(size_t iCol{};iCol < numColsMeshTemplate;++iCol){
                    meshTemplate[iMesh*numPixelsPerCell_u+iCol]    = true;
                    meshTemplate[iMesh*numPixelsPerCell_u+(numPixelsPerCell_u-iCol)] = true;
                }
            }
            break;
        }
    }
}

void fv::Camera::resetTemplate(){
    delete templateExtractPoints;
    templateExtractPoints = new bool[numPixels]{false};
}