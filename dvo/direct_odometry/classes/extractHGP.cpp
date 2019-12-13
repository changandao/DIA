//
// Created by font_al on 11/20/18.
//

#include "camera.h"

void fv::CamPinhole::extractHGP(cv::Mat Mat_grad,std::vector<fv::Pt> &points_, size_t d, const std::vector<size_t>& meshExtractPoints, bool* templateExtractPoints){

    fv::Pt pt{};

    size_t u0{} , v0{};
    size_t u00{} , v00{};
    size_t di{d};
    int uv{};
    bool add_pt{false};
    DATA_TYPE mean_G{} , Gi{} , Gmax{};
    int upRef{};
    int vpRef{};

    size_t u_block{};
    size_t v_block{};
    for(size_t i_mesh : meshExtractPoints){
        u_block = i_mesh%numPixelsPerCell_v;
        v_block = i_mesh/numPixelsPerCell_v;
        u0 = numCellsMesh_u*u_block;
        v0 = numCellsMesh_v*v_block;
        mean_G = 0.0;
        for(size_t u{}; u < numCellsMesh_u; ++u) {
            for(size_t v{}; v < numCellsMesh_v; ++v) {
                mean_G = mean_G + Mat_grad.at<DATA_TYPE>(int(v+v0), int(u+u0));
            }
        }
        mean_G = mean_G/(numCellsMesh_u*numCellsMesh_v);
        di = d;
        for (size_t ud{}; ud < numCellsMesh_u; ud += di) {
            u00 = u0 + ud;
            for (size_t vd{}; vd < numCellsMesh_v; vd += di) {
                v00 = v0 + vd;
                Gmax = 0.0;

                for (size_t u{}; u < di; ++u) {
                    for (size_t v{}; v < di; ++v) {
                        uv = int(compute_uv(int(u + u00),int(v + v00), 0));
                        Gi = Mat_grad.at<DATA_TYPE>(uv);
                        if ((templateExtractPoints[uv] == 0)&&(Gi > (-mean_G*LOGARITHM(DATA_TYPE(1-0.6))) && (Gi > Gmax)&&( Gi > sysSet.GminForHgpExtract))) {
                            Gmax = Gi;
                            pt.u_ref[0] = DATA_TYPE(u + u00);
                            pt.v_ref[0] = DATA_TYPE(v + v00);
                            upRef = int(pt.u_ref[0]);
                            vpRef = int(pt.v_ref[0]);
                            add_pt = true;
                        }
                    }
                }
                if ((add_pt)&(upRef+1> mask_surface)&(vpRef+1> mask_surface)&(upRef< wG[0]-mask_surface)&(vpRef< hG[0]-mask_surface)){
                    apply_mask_2_point(pt);
                    Mat_grad.at<DATA_TYPE>(vpRef, upRef) = 0.0;
                    points_.push_back(pt);
                    add_pt = false;
                    //templateExtractPoints[compute_uv(upRef,vpRef, 0)] = true;
                }
            }
        }
    }
}
