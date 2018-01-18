#include "keyframe.h"

void KeyFrame::orderById(DotMarkers Map)
{
    std::sort(detectedMarkers.begin(),detectedMarkers.end(),Reorder());
    for(int i=0;i<detectedMarkers.size();i++)
    {
        Dot curr = detectedMarkers[i];
        long curr_id;
        cv::Point3f pos_3D;
        if(curr_id=curr.getId()>0)
           {
            Map.get3DPos(curr_id,pos_3D);
            MapPoints.push_back(pos_3D);
            imagePoints.push_back(curr.getCenter());
        }
    }
}

cv::Mat KeyFrame::computePose(pose_estimation choice)
{
    switch(choice)
    {
        case SolvePnp:
        cv::solvePnP(MapPoints,imagePoints,CameraMatrix,DistCoeffs,Rot,Trans,true);
        break;
        case IPPE:
        cv::solvePnP(MapPoints,imagePoints,CameraMatrix,DistCoeffs,Rot,Trans);

    }
}

bool KeyFrame::checkCovisibleKF(KeyFrame KF)
{
    int commonObservations=0;
    std::vector<Dot> candidate_Markers = KF.getDetectedMarkers();
    std::vector<cv::Point2f> orderedKF1,orderedKF2;

    /// note : time complexity of order O(n+m)
    int i=0,j=0;
    while(i<detectedMarkers.size())
    {
        if(detectedMarkers[i].getId() > candidate_Markers[j].getId())
            j++;
        else if (detectedMarkers[i].getId() < candidate_Markers[j].getId())
            i++;
        else
        {
            orderedKF2.push_back(detectedMarkers[i].getCenter());
            orderedKF1.push_back(candidate_Markers[j].getCenter());
            commonObservations++;

        }
    }

    if(commonObservations > min_thresh)
    {
        /// check possibility of loop closure
        //// TODO:
        /// find shortest path from KF to this
        /// if length > thresh_cycle_length
        /// close loop


        ///Add covisible KF
        insertCovisibileKF(KF,orderedKF1,orderedKF2);

        return true;
    }
        return false;

}

bool KeyFrame::insertCovisibileKF(KeyFrame KF,std::vector<cv::Point2f> orderedKF1, std::vector<cv::Point2f> orderedKF2)
{
    ////TODO: unit testing
    cv::Mat rel_Rot = Rot.t()*KF.getRotMat();
    cv::Mat rel_Trans = Rot.t()*(KF.getTrans() - Trans);

    cv::Mat KFtoCurr_transform = rel_Rot*rel_Trans;
    std::vector<cv::Point2f> transformedKF1;
    cv::perspectiveTransform(orderedKF1,transformedKF1,KFtoCurr_transform);

    float reproj_err = reprojErrorRMS(orderedKF2,transformedKF1);

    CovisibleKFs.push_back(std::pair<KeyFrame,float>(KF,reproj_err)); // directed edge from curr to KF
    KF.addReciprocalEdge(*this,reproj_err); // reciprocal edge back from KF
}

bool KeyFrame::addReciprocalEdge(KeyFrame KF, float wt)
{
    CovisibleKFs.push_back(std::pair<KeyFrame,float>(KF,wt));
}

float KeyFrame::reprojErrorRMS(std::vector<cv::Point2f> imgPoints, std::vector<cv::Point2f> projPts)
{
    float sum=0;
    for(int i=0;i<imgPoints.size();i++)
    {
        sum += (pow(imgPoints[i].x-projPts[i].x,2)+pow(imgPoints[i].y-projPts[i].y,2));
    }
    sum /= imgPoints.size();
    sum = sqrt(sum);

    return sum;

}

void KeyFrame::setBadKF()
{
    reliable = false;
}

bool KeyFrame::isReliable()
{
    return reliable;
}

cv::Mat KeyFrame::getCamPosition()
{
    return -Rot.t()*Trans;
}

cv::Mat KeyFrame::getCamProjectionMat()
{
    return CameraMatrix*Rot*Trans;
}

std::vector<Dot> KeyFrame::getDetectedMarkers()
{
    return detectedMarkers;
}

cv::Mat KeyFrame::getRotMat()
{
    return Rot;
}

cv::Mat KeyFrame::getTrans()
{
    return Trans;
}
