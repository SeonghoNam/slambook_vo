#include "myslam/Frame.h"

namespace myslam{

    Frame::Frame()
    : _id(-1), _time_stamp(-1), _cam(nullptr)
    {

    }

    Frame::Frame ( long id, double time_stamp, SE3d T_c_w, Camera::Ptr camera, Mat color, Mat depth )
    : _id(id), _time_stamp(time_stamp), _T_c_w(T_c_w), _cam(camera), _color(color), _depth(depth)
    {

    }

    Frame::~Frame()
    {

    }

    Frame::Ptr Frame::createFrame()
    {
        static long factory_id = 0;
        return Frame::Ptr( new Frame(factory_id++) );
    }

    double Frame::findDepth ( const cv::KeyPoint& kp )
    {
        int x = cvRound(kp.pt.x);
        int y = cvRound(kp.pt.y);
        ushort d = _depth.ptr<ushort>(y)[x];
        if ( d!=0 )
        {
            return double(d)/_cam->_depth_scale;
        }
        else 
        {
            // check the nearby points 
            int dx[4] = {-1,0,1,0};
            int dy[4] = {0,-1,0,1};
            for ( int i=0; i<4; i++ )
            {
                d = _depth.ptr<ushort>( y+dy[i] )[x+dx[i]];
                if ( d!=0 )
                {
                    return double(d)/_cam->_depth_scale;
                }
            }
        }
        return -1.0;
    }


    Vector3d Frame::getCamCenter() const
    {
        return _T_c_w.inverse().translation();
    }

    bool Frame::isInFrame ( const Vector3d& pt_world )
    {
        Vector3d p_cam = _cam->world2camera( pt_world, _T_c_w );
        if ( p_cam(2,0)<0 ) 
            return false;
        Vector2d pixel = _cam->world2pixel( pt_world, _T_c_w );
        return pixel(0,0)>0 && pixel(1,0)>0 
            && pixel(0,0)<_color.cols 
            && pixel(1,0)<_color.rows;
    }
 

}