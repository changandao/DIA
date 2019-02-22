#include "myslam/g2o_types.h"

namespace myslam {




    void EdgeProjectXYZRGBDPoseOnly::computeError() {
        const g2o::VertexSE3Expmap *pose = static_cast<const g2o::VertexSE3Expmap *> ( _vertices[0] );

        Eigen::Vector3d x_local = pose->estimate().map(point_);
        Vector2d uv = frame_->camera_->camera2pixel(x_local);
        float x = uv(0);
        float y = uv(1);
        if (x - 4 < 0 || (x + 4) > frame_->gray_.cols || (y - 4) < 0 || (y + 4) > frame_->gray_.rows) {
            _error(0, 0) = 0.0;
            this->setLevel(1);
        } else {
            _error(0, 0) = getPixelValue(x,y) - _measurement;
        }

    }

    void EdgeProjectXYZRGBDPoseOnly::linearizeOplus() {
        g2o::VertexSE3Expmap *pose = static_cast<g2o::VertexSE3Expmap *> ( _vertices[0] );
        Vector3d xyz_trans = pose->estimate().map(point_);
        if ( level() == 1 )
        {
            _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
            return;
        }
        _jacobianOplusXi = frame_->calcJacobianWithT(xyz_trans);

    }



}
