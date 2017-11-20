#ifndef POSETRACKER_H
#define POSETRACKER_H

#endif // POSETRACKER_H
#include <opencv2/opencv.hpp>
#include "dot.h"
struct KeyFrames
{
    cv::Mat Rotation;   //known w.r.t first triangulated keyframe pair
    cv::Mat Translation; // known w.r.t first triangulated keyframe pair
    std::vector<Dot> Markers; //detected 2d markers
    std::vector<cv::Point3f> pos_3d; //Marker locations after triangulation
};

class PoseTracker
{

    cv::Mat CameraMatrix;
    std::vector<KeyFrames> Views;


};
