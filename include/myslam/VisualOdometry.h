#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/Map.h"

#include <opencv2/features2d/features2d.hpp>

namespace myslam 
{
class VisualOdometry
{
public:
    typedef shared_ptr<VisualOdometry> Ptr;
    enum VOState {
        INITIALIZING=-1,
        OK=0,
        LOST
    };
    
    VOState     _state;     // current VO status
    Map::Ptr    _map;       // map with all frames and map points
    Frame::Ptr  _ref;       // reference frame 
    Frame::Ptr  _cur;      // current frame 
    
    cv::Ptr<cv::ORB> _orb;  // orb detector and computer 
    vector<cv::Point3f>     _pts_3d_ref;        // 3d points in reference frame 
    vector<cv::KeyPoint>    _keypoints_curr;    // keypoints in current frame
    Mat                     _descriptors_curr;  // descriptor in current frame 
    Mat                     _descriptors_ref;   // descriptor in reference frame 
    vector<cv::DMatch>      _feature_matches;
    
    SE3d _T_c_r_estimated;  // the estimated pose of current frame 
    int _num_inliers;        // number of inlier features in icp
    int _num_lost;           // number of lost times
    
    // parameters 
    int _num_of_features;   // number of features
    double _scale_factor;   // scale in image pyramid
    int _level_pyramid;     // number of pyramid levels
    float _match_ratio;      // ratio for selecting  good matches
    int _max_num_lost;      // max number of continuous lost times
    int _min_inliers;       // minimum inliers
    
    double key_frame_min_rot;   // minimal rotation of two key-frames
    double key_frame_min_trans; // minimal translation of two key-frames
    
public: // functions 
    VisualOdometry();
    ~VisualOdometry();
    
    bool addFrame( Frame::Ptr frame );      // add a new frame 
    
protected:  
    // inner operation 
    void extractKeyPoints();
    void computeDescriptors(); 
    void featureMatching();
    void poseEstimationPnP(); 
    void setRef3DPoints();
    
    void addKeyFrame();
    bool checkEstimatedPose(); 
    bool checkKeyFrame();
    
};
}

#endif // VISUALODOMETRY_H
