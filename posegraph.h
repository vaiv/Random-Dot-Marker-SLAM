#ifndef POSEGRAPH_H
#define POSEGRAPH_H
#include "keyframe.h"

class PoseGraph
{
    //Baseline Setup
    std::vector<Dot> BF_1;
    std::vector<Dot> BF_2;
    cv::Mat CameraMatrix;
    cv::Mat EssentialMatrix;
    bool baseLineSetup;
    std::vector<KeyFrame> LocalBACandiates;

    //Graph Components

    std::vector<Keyframe> poseQuiver; //PoseGraph Adjacency List

    bool checkKF(KeyFrame KF);
    void addNewKF(KeyFrame KF);
    bool cullBadKF(KeyFrame KF);
    void lynchRedundantKFs();





};

#endif // POSEGRAPH_H
