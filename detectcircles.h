#ifndef DETECTCIRCLES_H
#define DETECTCIRCLES_H
#include<opencv2/opencv.hpp>

using namespace std;

class DetectCircles
{
    cv::Mat inImage;
    cv::Mat binaryMask;
    std::vector<cv::Vec3f> Circles;
    std::vector< std::vector<cv::Point> > Elliptical_Contours;
    std::vector<cv::RotatedRect> Ellipses;

    void binarize();
    cv::Mat filterImage();
    void retrieveEllipses();

public:

    DetectCircles(cv::Mat img);
    std::vector<cv::Vec3f> getDetectedCircles();
    std::vector<cv::Point2f> getCenters();
    void drawCircles();
};

#endif // DETECTCIRCLES_H
