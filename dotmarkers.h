#ifndef DOTMARKERS_H
#define DOTMARKERS_H
#include "dot.h"
class DotMarkers
{
    std::vector<Dot> Markers;
    std::map<long,cv::Point3f> triangulatedPoints;
    std::map<long,cv::Point3f> scaledPoints;
    long Max_ID;


public:

//    DotMarkers();
//    bool writePatternFile();
//    bool loadPatternFile(std::string fname);
//    void storeMarker(long int id,Dot Marker);
//    bool checkId(long int id);
//    Dot  findMarkerById(long int id);
//    cv::Mat scaleToWorld();

    DotMarkers();
    long int getMaxId(){return Max_ID;}
    bool find(Dot& Marker);
    bool insert(Dot& Marker);
    bool set3DPos(Dot Marker,cv::Point3f pt);
    bool get3DPos(long id,cv::Point3f& pt);
    void debug();
};

#endif // DOTMARKERS_H
