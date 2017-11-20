#ifndef DOT_H
#define DOT_H

#include <opencv2/opencv.hpp>
#include <iostream>

struct DistComp
{
    float distance;
    cv::Point2f Center;

    DistComp(float _distance,cv::Point2f _Center)
    {
        distance = _distance;
        Center = _Center;
    }

    int operator() (DistComp D)
    {
        return distance < D.distance;
    }
    int operator< (DistComp D)
    {
        return distance < D.distance;
    }

};
struct Comparator
{
   bool operator() (DistComp D, DistComp E)
   {
       return D.distance > E.distance;
   }
};

class Dot
{
    std::vector<DistComp> Neighbors;
    float resol;                                //scaling parameter
    float thresh1;                              //threshold for confidence of individual descriptor
    float thresh2;                              //threshold for voting scheme
    int nn_count;                               //count of nearest neighbors considered
    int selector;                               //size of subset considered for a descriptor
    std::vector<cv::Point2f> Centers;           //Neighbor Centers
    float tolerance;                            // tolerance factor for bruteforce descriptor matcher
    long int idx;                               //Marker Id
    cv::Point2f COM;                            //Centroid of the Neighbor-Point Locus
    cv::Vec3f Circle;                           //circle inscribing the dot
    cv::Mat frame;
    std::vector< std::vector<int> > Descriptors;
    cv::Point3f Point_3D;


void findNN();
void computeCOM();
void computeDescriptors(std::vector<DistComp> m_Neighbors);
double getMatch(std::vector<int> candidate);
float computeCrossRatio(std::vector<cv::Point2f> Vertices);
float computeCrossRatio_4Points(std::vector<cv::Point2f> Vertices);
float area(cv::Point2f pt1,cv::Point2f pt2, cv::Point2f pt3);

public:

Dot(cv::Vec3f Circle,cv::Mat frame,int nn_count,int selector,float thresh1,float thresh2,float resol);
cv::Point2f getCOM();
std::vector< std::vector<int> > getDescriptors();
void setCenters(std::vector<cv::Point2f> Centers);
void draw(cv::Mat& img);
void assignDescriptors();
void setId(long int id){this->idx = id;}
long int getId(){return idx;}
bool matchDescriptors(std::vector< std::vector<int> > poss_match);
cv::Point2f getCenter();
void set3DPos(cv::Point3f pt);
void get3DPos(cv::Point3f& pt);
cv::Point3f get3DPos();
};

#endif // DOT_H
