#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "myslam/VisualOdometry.h"
#include "myslam/Config.h"

namespace myslam
{
    VisualOdometry::VisualOdometry()
    :_state(INITIALIZING), _ref(nullptr), _cur(nullptr), _map(new Map), _num_lost(0), _num_inliers(0)
    {
        _num_of_features = Config::get<int> ("number_of_features");
        _scale_factor    = Config::get<double> ("scale_factor");
        _level_pyramid   = Config::get<int> ("level_pyramid");
        _match_ratio     = Config::get<float> ("match_ratio");
        _max_num_lost    = Config::get<int> ("max_num_lost");
        _min_inliers     = Config::get<int> ("min_inliers");

        key_frame_min_rot = Config::get<double> ("keyframe_rotation");
        key_frame_min_trans = Config::get<double> ("keyframe_translation");

        _orb = cv::ORB::create(_num_of_features, _scale_factor, _level_pyramid);
    }

    VisualOdometry::~VisualOdometry()
    {

    }

    bool VisualOdometry::addFrame(Frame::Ptr frame)
    {
        switch(_state)
        {
        case INITIALIZING:
            _state = OK;
            _ref = frame;
            _cur = frame;
            _map->insertKeyFrame(frame);
            extractKeyPoints();
            computeDescriptors();
            setRef3DPoints();
            break;
        case OK:
            _cur = frame;

            extractKeyPoints();
            computeDescriptors();
            featureMatching();
            poseEstimationPnP();
            if(checkEstimatedPose() == true)
            {
                _cur->_T_c_w = _T_c_r_estimated * _ref->_T_c_w;
                _ref = _cur;
                setRef3DPoints();
                _num_lost = 0;
                if(checkKeyFrame())
                {
                    addKeyFrame();
                }

            }
            else
            {
                _num_lost++;
                if(_num_lost > _max_num_lost)
                {
                    _state = LOST;
                }
                return false;

            }
            break;
        case LOST:
            cout << "vo has lost." << endl;
            break;
        }

       return true;
        
    }

    void VisualOdometry::extractKeyPoints()
    {
        _orb->detect(_cur->_color, _keypoints_curr);
    }

    void VisualOdometry::computeDescriptors()
    {
        _orb->compute(_cur->_color, _keypoints_curr, _descriptors_curr);
    }

    void VisualOdometry::featureMatching()
    {
        vector<cv::DMatch> matches;
        cv::BFMatcher matcher (cv::NORM_HAMMING);
        matcher.match(_descriptors_ref, _descriptors_curr, matches);

        // select the best matches
        float min_dis = std::min_element (
                            matches.begin(), matches.end(),
                            [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
        {
            return m1.distance < m2.distance;
        } )->distance;

        _feature_matches.clear();
        for ( cv::DMatch& m : matches )
        {
            if ( m.distance < max<float> ( min_dis*_match_ratio, 30.0 ) )
            {
                _feature_matches.push_back(m);
            }
        }
        cout<<"good matches: "<<_feature_matches.size()<<endl;
    }

    void VisualOdometry::poseEstimationPnP()
    {
     // construct the 3d 2d observations
        vector<cv::Point3f> pts3d;
        vector<cv::Point2f> pts2d;
        
        for ( cv::DMatch m:_feature_matches )
        {
            pts3d.push_back( _pts_3d_ref[m.queryIdx] );
            pts2d.push_back( _keypoints_curr[m.trainIdx].pt );
        }
        
        Mat K = ( cv::Mat_<double>(3,3)<<
            _ref->_cam->_fx, 0, _ref->_cam->_cx,
            0, _ref->_cam->_fy, _ref->_cam->_cy,
            0,0,1
        );
        Mat rvec, tvec, inliers;
        
        cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
        _num_inliers = inliers.rows;
        cout<<"pnp inliers: "<<_num_inliers<<endl;
        _T_c_r_estimated = SE3d(
            Sophus::SO3d::exp(Vector3d(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0))), 
            Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
        );
    }

    void VisualOdometry::setRef3DPoints()
    {
        _pts_3d_ref.clear();
        _descriptors_ref = Mat();
        for(size_t i = 0; i < _keypoints_curr.size(); i++)
        {
            double d = _ref->findDepth(_keypoints_curr[i]);
            if( d > 0)
            {
                Vector3d p_cam = _ref->_cam->pixel2camera(
                    Vector2d(_keypoints_curr[i].pt.x, _keypoints_curr[i].pt.y),d);
                _pts_3d_ref.push_back(cv::Point3f(p_cam(0,0), p_cam(1,0), p_cam(2,0) ));
                _descriptors_ref.push_back(_descriptors_curr.row(i));
            }
        }
    }

    bool VisualOdometry::checkEstimatedPose()
    {
        // check if the estimated pose is good
        if ( _num_inliers < _min_inliers )
        {
            cout<<"reject because inlier is too small:"<<_num_inliers<<endl;
            return false;
        }
        // if the motion is too large, it is probably wrong
        Sophus::Vector6d d = _T_c_r_estimated.log();
        if ( d.norm() > 5.0 )
        {
            cout<<"reject because motion is too large: "<<d.norm()<<endl;
            return false;
        }
        return true;
    }

    bool VisualOdometry::checkKeyFrame()
    {
        Sophus::Vector6d d = _T_c_r_estimated.log();
        Vector3d trans = d.head<3>();
        Vector3d rot = d.tail<3>();
        if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
            return true;
        return false;
    }

    void VisualOdometry::addKeyFrame()
    {
        cout<<"adding a key-frame"<<endl;
        _map->insertKeyFrame(_cur);
    }
}