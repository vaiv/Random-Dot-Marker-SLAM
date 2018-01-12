#include "dotmarkers.h"

DotMarkers::DotMarkers()
{
    Max_ID=1;
}

bool DotMarkers::find(Dot& Marker)
{
     int count = 0;
    for(int i=0;i<Markers.size();i++)
    {
        Dot curr_Marker = Markers[i];

        if(count+=curr_Marker.withinROI(Marker.getCenter()) && curr_Marker.matchDescriptors(Marker.getDescriptors()))
        {
            Marker.setId(curr_Marker.getId());
            Markers[i].updateMotionModel(Marker);
            std::cout<<"number of candidates evaluated"<<count<<std::endl;
            return true;
        }
    }

    return false;
}

bool DotMarkers::insert(Dot& Marker)
{
    Marker.setId(Max_ID);
    Marker.resetMotionModel();
    Max_ID++;
    Markers.push_back(Marker);
}

bool DotMarkers::set3DPos(Dot Marker, cv::Point3f pt)
{
    for(int i=0;i<Markers.size();i++)
    {
        if(Markers[i].matchDescriptors(Marker.getDescriptors()))
        {
            Markers[i].set3DPos(pt);
            return true;
        }
    }
    return false;

}

bool DotMarkers::get3DPos(long id, cv::Point3f &pt)
{
    for(int i=0;i<Markers.size();i++)
    {
        Dot curr_Marker = Markers[i];
        if(curr_Marker.getId() == id)
        {
            curr_Marker.get3DPos(pt);
            if(pt.x!=0 || pt.y!=0 || pt.z!=0)
            return true;
        }
    }

    return false;
}

//Mat DotMarkers::getAllDescriptors()
//{
//    int DescSize;
//    cv::Mat allDesc(Markers.size(),DescSize,CV_32FC1);
//    for(int i=0;i<Markers.size();i++)
//    {
//         std::vector< std::vector<int> >> Desc = Markers[i].getDescriptors();

//        for(int j=0;j<DescSize;j++)
//        {

//        }
//    }
//}

void DotMarkers::debug()
{
    std::vector< std::vector<int> > Desc2 = Markers[1].getDescriptors();
    std::vector< std::vector<int> > Desc12 = Markers[20].getDescriptors();
    std::vector< std::vector<int> > Desc20 = Markers[24].getDescriptors();
    //std::vector< std::vector<int> > Desc27 = Markers[26].getDescriptors();

}

