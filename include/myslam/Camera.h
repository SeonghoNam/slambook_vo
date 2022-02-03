#ifndef CAMERA_H
#define CAMERA_H
#include "myslam/common_include.h"

namespace myslam
{
    class Camera
    {
        public:
        typedef shared_ptr<Camera> Ptr;
        double _fx, _fy, _cx, _cy, _depth_scale;

        Camera();
        Camera(double fx, double fy, double cx, double cy, double depth_scale = 0)
        :_fx(fx),_fy(fy),_cx(cx),_cy(cy),_depth_scale(depth_scale)
        {

        }

        Vector3d world2camera(const Vector3d& p_w, const SE3d& T_c_w);
        Vector3d camera2world(const Vector3d& p_c, const SE3d& T_c_w);
        Vector2d camera2pixel(const Vector3d& p_c);
        Vector3d pixel2camera(const Vector2d& p_p, double depth = 1);
        Vector3d pixel2word(const Vector2d& p_p, const SE3d& T_c_w, double depth = 1);
        Vector2d world2pixel(const Vector3d& p_w, const SE3d& T_c_w);
        
    };
}

#endif