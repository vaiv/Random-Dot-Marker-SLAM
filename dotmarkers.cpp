#include "dotmarkers.h"

DotMarkers::DotMarkers()
{
    Max_ID=1;
}

bool DotMarkers::find(Dot& Marker)
{
    for(int i=0;i<Markers.size();i++)
    {
        Dot curr_Marker = Markers[i];
        if(curr_Marker.matchDescriptors(Marker.getDescriptors()))
        {
            Marker.setId(curr_Marker.getId());
            return true;
        }
    }

    return false;
}

bool DotMarkers::insert(Dot& Marker)
{
    Marker.setId(Max_ID);
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
