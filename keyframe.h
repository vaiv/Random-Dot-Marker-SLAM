#ifndef KEYFRAME_H
#define KEYFRAME_H
#include "dot.h"
#include "dotmarkers.h"

struct Reorder
{
   bool operator() (Dot D, Dot E)
   {
       return D.getId() > E.getId();
   }
};

enum pose_estimation
{
    SolvePnp =0 ,IPPE
};

class KeyFrame
{
    cv::Mat Rot; //w.r.t originally triangulated keyframe
    cv::Mat Trans; // w.r.t originally triangulated keyframe
    cv::Mat Tcw; // transformation matrix world to camera
    cv::Mat Twc; // transformation matrix camera to world
    cv::Mat CameraMatrix; //optimised CameraMatrix
    cv::Mat DistCoeffs; //optimised Distortion Coefficents
    std::vector<Dot> detectedMarkers; //  all detected Map Points
    std::vector<std::pair<KeyFrame,float>> CovisibleKFs; //Covisibility Graph: G(KF,w) where w is reproj error
    std::vector<cv::Point3f> MapPoints;
    std::vector<cv::Point2f> imagePoints;
    std::vector<long> BagofIds; // Ids for loop closure detection
    float min_thresh;
    bool reliable;
    //// Methods

    cv::Mat computePose(pose_estimation choice);
    bool insertCovisibileKF(KeyFrame KF,std::vector<cv::Point2f> orderedKF1, std::vector<cv::Point2f> orderedKF2);
    bool isReliable();
    float reprojErrorRMS(std::vector<cv::Point2f> imgPoints, std::vector<cv::Point2f> projPts);
    void orderById(DotMarkers Map);

public:
    cv::Mat getCamProjectionMat();
    cv::Mat getCamPosition();
    bool checkCovisibleKF(KeyFrame KF);
    std::vector<Dot> getDetectedMarkers();
    void setBadKF();
    cv::Mat getRotMat();
    cv::Mat getTrans();
    bool addReciprocalEdge(KeyFrame KF,float wt);





};

#endif // KEYFRAME_H
