#ifndef FRAME_H
#define FRAME_H

#include "myslam/common_include.h"
#include "myslam/Camera.h"

namespace myslam
{
    class Frame
    {
        public:
        typedef shared_ptr<Frame> Ptr;
        unsigned long _id;
        double _time_stamp;
        SE3d _T_c_w;
        Camera::Ptr _cam;
        Mat _color, _depth;
        
        Frame();
        Frame(long id, double time_stamp = 0, SE3d T_c_w=SE3d(), Camera::Ptr cam = nullptr, Mat color = Mat(), Mat depth = Mat());
        ~Frame();

        static Frame::Ptr createFrame();
        double findDepth(const cv::KeyPoint& kp);
        Vector3d getCamCenter() const;
        bool isInFrame(const Vector3d& pt_world);
    };
}
#endif