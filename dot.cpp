#include "dot.h"

float distBw(cv::Point2f pt1,cv::Point2f pt2)
{
    return sqrt(pow(pt1.x-pt2.x,2) + pow(pt1.y-pt2.y,2));
}

Dot::Dot(cv::Vec3f Circle,cv::Mat frame,int nn_count,int selector,float thresh1,float thresh2,float resol)
{
    this->Circle = Circle;
    this->frame = frame.clone();
    this->thresh1 = thresh1;
    this->thresh2 = thresh2;
    this->resol = resol;
    this->nn_count = nn_count;
    this->selector = selector;
    tolerance = 0.01;
    idx=-1;
}

void Dot::findNN()
{
   std::priority_queue< DistComp,std::vector<DistComp>,Comparator > minHeap;
   std::vector<DistComp> dist_arr;
   for(int i=0;i<Centers.size();i++)
   {
       cv::Point2f  N_Circle = Centers[i];
       float distance = (float)sqrt(pow(Circle[0]-N_Circle.x,2) + pow(Circle[1]-N_Circle.y,2)); //eucledian
       //float distance = fabs(Circle[0]-N_Circle.x) + fabs(Circle[1]-N_Circle.y);
       DistComp obj(distance,Centers[i]);
       if(obj.distance>0)
       //    dist_arr.push_back(obj);
       minHeap.push(obj);
   }

   std::sort(dist_arr.begin(),dist_arr.end(),Comparator());
   if(minHeap.size()>0)
   {
    for(int i=0;i<nn_count;i++)
    {
       //DistComp N = dist_arr[i];
       DistComp N = minHeap.top();
       minHeap.pop();
       Neighbors.push_back(N);
    }
   }

}


void Dot::computeCOM()
{
    for(int i=0;i<Neighbors.size();i++)
    {
        DistComp tmp = Neighbors[i];
        COM.x += tmp.Center.x;
        COM.y += tmp.Center.y;
    }

    COM.x/=nn_count;
    COM.y/=nn_count;

}

float Dot::area(cv::Point2f pt1,cv::Point2f pt2, cv::Point2f pt3)
{
    float a = distBw(pt1,pt2);
    float b = distBw(pt1,pt3);
    float c = distBw(pt2,pt3);

    float S = (a+b+c)/2;

    return sqrt(S*(S-a)*(S-b)*(S-c));
}

float Dot::computeCrossRatio(std::vector<cv::Point2f> Vertices)
{
    float val1 = area(Vertices[0],Vertices[1],Vertices[2]);
    float val2 = area(Vertices[0],Vertices[3],Vertices[4]);
    float val3 = area(Vertices[0],Vertices[1],Vertices[3]);
    float val4 = area(Vertices[0],Vertices[2],Vertices[4]);
    if(val1*val2*val3*val4 > 0)
        return (val1*val2)/(val3*val4);
    else
        return 0;
}

//float Dot::computeCrossRatio_4Points(std::vector<cv::Point2f> Vertices)
//{
//    float val1 = distBw(Vertices[0],Vertices[2]);
//    float val2 = distBw(Vertices[1],Vertices[3]);
//    float val3 = distBw(Vertices[0],Vertices[3]);
//    float val4 = distBw(Vertices[1],Vertices[2]);

//    if(val1*val2*val3*val4 > 0)
//        return (val1*val2)/(val3*val4);
//    else
//        return 0;
//}

float Magnitude(cv::Vec3f vector)
{
    return sqrt(pow(vector[0],2) + pow(vector[1],2) + pow(vector[2],2));
}

float Dot::computeCrossRatio_4Points(std::vector<cv::Point2f> Vertices)
{
    cv::Vec3f concurrent_lines[4], cross[4];

    for(int i=0;i<4;i++)
    {
        cv::Vec3f dst;
        cv::Point2f vtx, src;
        vtx = Vertices[i+1];
        src = Vertices[0];
        dst[0] = vtx.x - src.x;
        dst[1] = vtx.y - src.y;
        dst[2] = 0;

        concurrent_lines[i] = dst;
        concurrent_lines[i] = dst;
    }

    cross[0] = concurrent_lines[0].cross(concurrent_lines[2]); //ac
    cross[1] = concurrent_lines[1].cross(concurrent_lines[2]); //bc
    cross[2] = concurrent_lines[0].cross(concurrent_lines[3]); //ad
    cross[3] = concurrent_lines[1].cross(concurrent_lines[3]); //bd



    float val1 = sin(Magnitude(cross[0])/(Magnitude(concurrent_lines[0])*Magnitude(concurrent_lines[2])));
    float val2 = sin(Magnitude(cross[1])/(Magnitude(concurrent_lines[1])*Magnitude(concurrent_lines[2])));
    float val3 = sin(Magnitude(cross[2])/(Magnitude(concurrent_lines[0])*Magnitude(concurrent_lines[3])));
    float val4 = sin(Magnitude(cross[3])/(Magnitude(concurrent_lines[1])*Magnitude(concurrent_lines[3])));

    if(val1*val2*val3*val4 > 0)
        return (val1*val4)/(val2*val3);
    else
        return 0;
}

void Dot::computeDescriptors(std::vector<DistComp> m_Neighbors)
{
    std::vector<bool> comb(m_Neighbors.size());
    std::fill(comb.begin(),comb.begin()+4,true);
    std::vector<int> curr_Desc;
    cv::Point2f Center(Circle[0],Circle[1]);
    do
    {
        std::vector<cv::Point2f> Vertices;
        Vertices.push_back(Center);
        for(int i=0;i<m_Neighbors.size();i++)
        {
            if(comb[i])
            Vertices.push_back(m_Neighbors[i].Center);
         }

        float desc = computeCrossRatio_4Points(Vertices);
        curr_Desc.push_back(desc*resol);

    } while(std::prev_permutation(comb.begin(),comb.end()));

    Descriptors.push_back(curr_Desc);
}

//void Dot::computeDescriptors(std::vector<DistComp> m_Neighbors)
//{
//    std::vector<int> curr_Desc;
//    float dist_norm = m_Neighbors[0].distance;
//    m_Neighbors[0].distance = cvRound(m_Neighbors[0].distance);
//    for(int i=1;i<m_Neighbors.size();i++)
//    {
//        int val = cvRound((m_Neighbors[i].distance/dist_norm)*resol);
//        curr_Desc.push_back(val);
//    }

//    Descriptors.push_back(curr_Desc);
//}

//nCm descriptors generated for each marker

void Dot::assignDescriptors()
{
    if(Neighbors.size()<nn_count)
        return;
    std::vector<bool> comb(nn_count);
    std::fill(comb.begin(),comb.begin()+selector,true);

    do
    {
         std::vector<DistComp>m_Neighbors;
         for(int i=0;i<nn_count;i++)
         {
             if(comb[i])
             {
                 m_Neighbors.push_back(Neighbors[i]);               
             }
         }
          computeDescriptors(m_Neighbors);
    }
    while(std::prev_permutation(comb.begin(),comb.end()));

}

std::vector< std::vector<int> > Dot::getDescriptors()
{
    return Descriptors;
}

void Dot::draw(cv::Mat& img)
{
    cv::Point Center = cv::Point(Circle[0],Circle[1]);
    cv::arrowedLine(img,Center,COM,cvScalar(255,0,0));
    cv::putText(img,std::to_string(idx),Center,cv::FONT_HERSHEY_COMPLEX,0.4,cvScalar(0,0,255));
//    for(int i=0;i<Neighbors.size();i++)
//    {
//        DistComp tmp = Neighbors[i];
//        cv::arrowedLine(img,Center, tmp.Center,cvScalar(0,255,0));
//    }
}

bool Dot::matchDescriptors(std::vector< std::vector<int> > poss_match)
{
    //bruteforce matching complexity ((nCm)*m)^2
    if(poss_match.size()==0)
        return false;

    long int match_count=0;
    for(int i=0;i<poss_match.size();i++)
    {
        std::vector<int> candidate_descriptor = poss_match[i];
        double conf = getMatch(candidate_descriptor);
        if(conf>thresh1)
            match_count++;
    }

    if(match_count>thresh2*Descriptors.size())
    return true;

    else return false;
}

//alternative:- instead of exact matches compute descriptor distance and choose the one with minimum subject to a threshold.
double Dot::getMatch(std::vector<int> candidate)
{
    long int count=0;
    for(int i=0;i<Descriptors.size();i++)
    {
        std::vector<int> curr_Desc = Descriptors[i];
        for(int j=0;j<curr_Desc.size();j++)
        {
            if(fabs(candidate[j]-curr_Desc[j])<tolerance*candidate[j])
                count++;
        }
    }
    double conf = count/Descriptors.size();
    return conf;
}

void Dot::setCenters(std::vector<cv::Point2f> Centers)
{
     this->Centers = Centers;
    findNN();
    computeCOM();
}

cv::Point2f Dot::getCenter()
{
    return cv::Point2f(Circle[0],Circle[1]);
}

void Dot::set3DPos(cv::Point3f pt)
{
    Point_3D = pt;
}

void Dot::get3DPos(cv::Point3f& pt)
{
    pt = Point_3D;
}

cv::Point3f Dot::get3DPos()
{
    return Point_3D;
}
