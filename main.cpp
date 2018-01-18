#include <opencv2/opencv.hpp>
#include <iostream>
#include "detectcircles.h"
#include "dot.h"
#include "dotmarkers.h"
#include<chrono>
typedef std::chrono::high_resolution_clock Clock;


using namespace cv;
using namespace std;
class desc_match : public ParallelLoopBody
{
public:
    desc_match(std::vector<Dot>* BF_1, std::vector<Dot>* BF_2, std::vector<Dot>* Markers_1,std::vector<Dot>* Markers_2):_BF_1(BF_1),_BF_2(BF_2),_Markers_1(Markers_1),_Markers_2(Markers_2) {}
    virtual void operator ()(const Range& range) const
    {
        for(int r= range.start;r<range.end;r++)
     {   for(int j=0;j<(*_BF_2).size();j++)
        {
            if((*_BF_1)[r].matchDescriptors((*_BF_2)[j].getDescriptors()))
            {
                (*_Markers_1).push_back((*_BF_1)[r]);
                (*_Markers_2).push_back((*_BF_2)[j]);
                cout<<"Marker with id "<<(*_BF_1)[r].getId()<<" matched"<<endl;

            }
        }
        }
    }

private:
   std::vector<Dot>* _BF_1,*_BF_2,*_Markers_1,*_Markers_2;

  }  ;

Mat draw(cv::Mat Rot_Vec, cv::Mat Trans, cv::Mat Cam, cv::Mat DistCoeffs,cv::Mat img,std::vector<Point3f> axis,cv::VideoWriter &wcap)
{
    vector<cv::Point2f> imgpts;
//    vector<cv::Point3f> axis;
//    cv::Mat res = image.clone();
//        axis.push_back(cv::Point3f(0.0,0.0,0.5));
//        axis.push_back(cv::Point3f(0.2,0.0,0.5));
//        axis.push_back(cv::Point3f(0.0,-0.2,0.5));
//        axis.push_back(cv::Point3f(0.0,0.0,0.7));

    cv::Mat res = img.clone();
    cv::projectPoints(axis, Rot_Vec, Trans, Cam, DistCoeffs,imgpts);

    vector<Point> first,last;
    vector<vector<Point>> top,bottom,up,down,left,right;
    for(int i=0;i<imgpts.size();i++)
    {
        if(i>3)
            last.push_back(imgpts[i]);
        else
            first.push_back(imgpts[i]);
    }
     Mat cnt=Mat::zeros(cvSize(img.cols,img.rows),CV_8UC1);
    top.push_back(first);
    bottom.push_back(last);
///pyramid begins
vector<Point> F1,F2,F3,F4;
Point vertex;
vertex.x= (first[0].x+first[2].x)/2;
vertex.y= (first[0].y+first[2].y)/2;

F1.push_back(last[0]);
F1.push_back(last[1]);
F1.push_back(vertex);
up.push_back(F1);

F2.push_back(last[1]);
F2.push_back(last[2]);
F2.push_back(vertex);
right.push_back(F2);

F3.push_back(last[2]);
F3.push_back(last[3]);
F3.push_back(vertex);
down.push_back(F3);

F4.push_back(last[3]);
F4.push_back(last[0]);
F4.push_back(vertex);
left.push_back(F4);

drawContours(img,bottom,-1,cvScalar(0,255,0),CV_FILLED);
drawContours(img,down,-1,cvScalar(0,0,255),CV_FILLED);
drawContours(img,right,-1,cvScalar(255,0,0),CV_FILLED);
drawContours(img,left,-1,cvScalar(255,255,255),CV_FILLED);
drawContours(img,up,-1,cvScalar(125,125,0),CV_FILLED);

line(img,last[0],vertex,(255),3);
line(img,last[1],vertex,(255),3);
line(img,last[2],vertex,(255),3);
line(img,last[3],vertex,(255),3);

//for(int i=0;i<4;i++)
//{
//    line(img,first[i],last[i],(255),3);

//    int t= (i+1<4)?i+1:0;
//    line(img,first[i],first[t],(255),3);

//    line(img,last[i],last[t],(255),3);

//}
/// pyramid ends
    vector<Point> U,D,L,R,P;
    //up face points
    U.push_back(first[0]);
    U.push_back(first[1]);
    U.push_back(last[1]);
    U.push_back(last[0]);
    //up.push_back(U);

    //down face points
    D.push_back(first[2]);
    D.push_back(first[3]);
    D.push_back(last[3]);
    D.push_back(last[2]);
    //down.push_back(D);

    //left face points
    L.push_back(first[0]);
    L.push_back(first[3]);
    L.push_back(last[3]);
    L.push_back(last[0]);
    //left.push_back(L);

    //right face points
    R.push_back(first[1]);
    R.push_back(first[2]);
    R.push_back(last[2]);
    R.push_back(last[1]);
    //right.push_back(R);


//    drawContours(img,top,-1,cvScalar(0,0,255),CV_FILLED);
//    drawContours(img,up,-1,cvScalar(255,0,0),CV_FILLED);
//    drawContours(img,down,-1,cvScalar(125,125,0),CV_FILLED);
//    drawContours(img,left,-1,cvScalar(125,0,125),CV_FILLED);
//    drawContours(img,right,-1,cvScalar(0,125,125),CV_FILLED);
//    drawContours(img,bottom,-1,cvScalar(0,255,0),CV_FILLED);

    //drawContours(img,top,-1,cvScalar(0,255,0));
    imshow("AR",img);
    wcap<<img;
    return img;

}



////Boilerplate driver code. Not to be used in final application.

void DrawAxis(cv::Mat Rot_Vec, cv::Mat Trans, cv::Mat Cam, cv::Mat DistCoeffs,cv::Mat image,std::vector<Point3f> axis,cv::VideoWriter &wcap)
{
    //cout<<image.size()<<endl;
    vector<cv::Point2f> imagePoints;
//    vector<cv::Point3f> axis;
//    cv::Mat res = image.clone();
//        axis.push_back(cv::Point3f(0.0,0.0,0.5));
//        axis.push_back(cv::Point3f(0.2,0.0,0.5));
//        axis.push_back(cv::Point3f(0.0,-0.2,0.5));
//        axis.push_back(cv::Point3f(0.0,0.0,0.7));

    cv::Mat res = image.clone();
    cv::projectPoints(axis, Rot_Vec, Trans, Cam, DistCoeffs,imagePoints);
    cv::line(res,imagePoints[0],imagePoints[1],cvScalar(0,0,255),3);
    cv::line(res,imagePoints[0],imagePoints[2],cvScalar(0,255,0),3);
    cv::line(res,imagePoints[0],imagePoints[3],cvScalar(255,0,0),3);

    cv::imshow("AR",res);
    wcap<<res;

}


void getMatches(std::vector<Dot> BF_1, std::vector<Dot> BF_2, std::vector<Dot>& Markers_1,std::vector<Dot>& Markers_2)
{
    for(int i=0;i<BF_1.size();i++)
    {
        for(int j=0;j<BF_2.size();j++)
        {
            if(BF_1[i].matchDescriptors(BF_2[j].getDescriptors()))
            {
                Markers_1.push_back(BF_1[i]);
                Markers_2.push_back(BF_2[j]);
                cout<<"Marker with id "<<BF_1[i].getId()<<" matched"<<endl;

            }
        }
    }
}

bool withinROI(cv::Point2f pt, cv::Mat img)
{
    return (pt.x > img.size().width*0.25 && pt.x <img.size().width*0.75 && pt.y > img.size().height*0.25 && pt.y <img.size().height*0.75);
}

int main(int argc,char** argv)
{
    string source;
    VideoCapture reader;
        if(strcasecmp(argv[1],"live")==0)
        {
            std::cout<<"enter camera index"<<std::endl;
            int c;
            std::cin>>c;
            reader.open(c);
            std::cout<<"sourcing from video"<<source<<std::endl;
        }
        else
        reader.open(argv[1]);

    cv::Mat K,D; //CameraMatrix and Distortion Coefficients
    FileStorage fs(argv[2], cv::FileStorage::READ);
    fs["camera_matrix"] >> K;
    fs["distortion_coefficients"] >> D;

        ////K for ideal camera
//    cv::Mat K = (Mat_<double>(3,3) <<
//            500, 0, 1280/2,
//            0, 500, 720/2,
//            0, 0,    1);

    cv::VideoWriter  w_cap("./Augmented.avi",CV_FOURCC('M','J','P','G'),29.0,cvSize((int) reader.get(CV_CAP_PROP_FRAME_WIDTH),(int) reader.get(CV_CAP_PROP_FRAME_HEIGHT)));

    char kb=' ';
    DotMarkers Pattern;
    std::vector<Dot> BF_1;
    std::vector<Dot> BF_2;
    bool pos3D =false, setPattern =false;
    std::vector<cv::Point3f> Axis;
    int frame_ct=0;
    float fin_error;
        while(reader.grab() && kb!='q')
        {
            cv::Mat dist_frame,frame;
            reader.retrieve(dist_frame);
            cv::undistort(dist_frame,frame,K,D);
            //dist_frame.copyTo(frame);
            ////ellipse detection
            auto t1 = Clock::now();
            DetectCircles detector(frame);
            detector.drawCircles();
            vector<Vec3f> Circles = detector.getDetectedCircles();
            auto t2 = Clock::now();
//            std::cout << "Delta t2-t1: Detection Time:- "
//                         << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count()
//                         << " nanoseconds" << std::endl;
            std::vector<Dot> tmp;
            Mat Rotation_Vec,Translation_Vec;
            if(Circles.size()>7)
            {

                for(int i=0;i<Circles.size();i++)
                {
                    Dot Marker(Circles[i],frame,7,6,0.3f,0.8f,100);
                    auto t3 = Clock::now();
                    Marker.setCenters(detector.getCenters());
                    auto t4 = Clock::now();
//                    std::cout << "Delta t4-t3: NN computation time"
//                                 << std::chrono::duration_cast<std::chrono::nanoseconds>(t4 - t3).count()
//                                 << " nanoseconds" << std::endl;
                    Marker.assignDescriptors();
                    auto t5 =  Clock::now();
//                    std::cout << "Delta t5-t4: Descriptor Computation time "
//                                 << std::chrono::duration_cast<std::chrono::nanoseconds>(t5 - t4).count()
//                                 << " nanoseconds" << std::endl;
                    if( !Pattern.find(Marker) && !setPattern )
                        Pattern.insert(Marker);

                    auto t6 = Clock::now();
                    std::cout << "Delta t6-t5: single marker matching time: "
                                 << std::chrono::duration_cast<std::chrono::nanoseconds>(t6 - t5).count()
                                 << " nanoseconds" << std::endl;

                    if(Marker.getId()>0)
                       {
                        tmp.push_back(Marker);
                        Marker.draw(frame);
                    }

                }
                if(Pattern.getMaxId()>=25)
                    setPattern=true;

                cv::imshow("Detected Markers",frame);
                kb = cv::waitKey(20);

                if(kb=='s')
                {
                    BF_1.clear();
                    BF_1 = tmp;
                    cv::imshow("Baseline Frame 1",frame);
                    cv::imwrite("bf_frame1.png",frame);
                    kb = cv::waitKey(0);
                    if(kb == 'y')
                     {   kb==' ';
                        pos3D=false;
                    }
                    else continue;
                }

                else if(kb=='r')
                {
                    BF_2.clear();
                    BF_2 = tmp;
                    cv::imshow("Baseline Frame 2",frame);
                    cv::imwrite("bf_frame2.png",frame);
                    kb = cv::waitKey(0);
                    if(kb == 'y')
                     {   kb==' ';
                        pos3D=false;
                    }
                    else continue;
                }

                if(kb=='q' && frame_ct>0)
                    std::cout<<"avg reprojection error for all frames is"<<fin_error/frame_ct<<std::endl;

            }

            if(BF_1.size()*BF_2.size()>0 && !pos3D)
            {
                std::vector<Dot> Markers_1;
                std::vector<Dot> Markers_2;
                std::vector<cv::Point2f> corres_1;
                std::vector<cv::Point2f> corres_2;

                cv::Mat mask, Rot, Trans;
                cv::Mat triangulated_Points;
                getMatches(BF_1,BF_2,Markers_1,Markers_2);
                //parallel_for_(Range(0,BF_1.size()),desc_match(&BF_1,&BF_2,&Markers_1,&Markers_2));

                for(int i=0;i<Markers_1.size();i++)
                {
                    corres_1.push_back(Markers_1[i].getCenter());
                    corres_2.push_back(Markers_2[i].getCenter());
                }

                if(corres_1.size()>=5)
                {
                    cv::Mat EssentialMat = cv::findEssentialMat(corres_1,corres_2,K,cv::RANSAC,0.999,1.0,mask);
                    cv::recoverPose(EssentialMat,corres_1,corres_2,K,Rot,Trans,3,mask,triangulated_Points);

                    cout<<"triangulated points are "<<endl<<triangulated_Points<<endl;

                    cout<<"3D Coordinates"<<endl;
                    std::vector<Point3f> Coords3D;
                    std::vector<Point2f> imgpts;
                    for(int c=0;c < triangulated_Points.cols;c++)
                    {
                        cv::Vec4d triangCoords = triangulated_Points.col(c);
                        cv::Vec3f Point_3D;
                        for (unsigned int i = 0; i < 3; i++)
                            Point_3D[i] = triangCoords[i] / triangCoords[3];
                        Coords3D.push_back(Point_3D);
                    }
                    cout<<Coords3D<<endl;
                    cv::Mat Rot_Vec,empty_D;
                    cv::Rodrigues(Rot,Rot_Vec);
                    cv::projectPoints(Coords3D,Rot_Vec,Trans,K,empty_D,imgpts);
                    float sum=0;
                    for(int i=0;i<corres_2.size();i++)
                    {
                        sum += (pow(imgpts[i].x-corres_2[i].x,2)+pow(imgpts[i].y-corres_2[i].y,2));
                    }
                    sum /= corres_2.size();
                    sum = sqrt(sum);
                    std::cout<<"RMS reproj error is "<<sum<<std::endl;


                    for(int j=0;j<Coords3D.size();j++)
                    {
                        Dot Marker = Markers_2[j];
                        if(!Pattern.set3DPos(Marker,Coords3D[j]))
                            cout<<"this should not have happenend, an already located marker could not be found!"<<endl;
                    }

                    cv::Point3f origin = cv::Point3f(0,0,Coords3D[0].z);
                    //float side_length = 4.3;
                    Axis.push_back(origin);
                    Axis.push_back(cv::Point3f(origin.x + 0.46,origin.y,origin.z));
                    Axis.push_back(cv::Point3f(origin.x ,origin.y + 0.46,origin.z));
                    Axis.push_back(cv::Point3f(origin.x ,origin.y,origin.z - 0.46));

////pyramid
//                   Axis.push_back(origin);
//                   Axis.push_back(cv::Point3f(origin.x,origin.y+side_length,origin.z));
//                    Axis.push_back(cv::Point3f(origin.x+side_length,origin.y+side_length,origin.z));
//                     Axis.push_back(cv::Point3f(origin.x+side_length,origin.y,origin.z));

//                      Axis.push_back(cv::Point3f(origin.x,origin.y,origin.z-side_length));
//                      Axis.push_back(cv::Point3f(origin.x,origin.y+side_length,origin.z-side_length));
//                       Axis.push_back(cv::Point3f(origin.x+side_length,origin.y+side_length,origin.z-side_length));
//                        Axis.push_back(cv::Point3f(origin.x+side_length,origin.y,origin.z-side_length));

                    pos3D = true;
                }

            }

            if(pos3D) //find pose using PnP and check reprojection error
            {
                cv::Point3f Point3D;
                std::vector<Dot> known3DMarkers;
               for(int k=0;k<tmp.size();k++)
               {
                   if(Pattern.get3DPos(tmp[k].getId(),Point3D))
                   {
                       tmp[k].set3DPos(Point3D);
                       known3DMarkers.push_back(tmp[k]);
                   }

               }

               if(known3DMarkers.size()>5)
               {
                   std::vector<Point2f> imgPoints;
                   std::vector<Point3f> worldPoints;

                    frame_ct++;

                   for(int k=0;k<known3DMarkers.size();k++)
                   {
                       imgPoints.push_back(known3DMarkers[k].getCenter());
                       worldPoints.push_back(known3DMarkers[k].get3DPos());
                   }

                   if(Rotation_Vec.empty() || Translation_Vec.empty())
                        cv::solvePnP(worldPoints,imgPoints,K,cv::Mat(),Rotation_Vec,Translation_Vec);
                   else
                       cv::solvePnP(worldPoints,imgPoints,K,cv::Mat(),Rotation_Vec,Translation_Vec,true);

                   //cout<<"rotation vector is "<<Rotation_Vec<<endl<<"translation vector is"<<Translation_Vec<<endl;
                   std::vector<cv::Point2f> projPts;
                   cv::projectPoints(worldPoints,Rotation_Vec,Translation_Vec,K,cv::Mat(),projPts);

                   float sum=0;
                   for(int i=0;i<imgPoints.size();i++)
                   {
                       sum += (pow(imgPoints[i].x-projPts[i].x,2)+pow(imgPoints[i].y-projPts[i].y,2));
                   }
                   sum /= imgPoints.size();
                   sum = sqrt(sum);
                   std::cout<<"RMS reproj error is "<<sum<<std::endl;
                    fin_error += sum;
                   DrawAxis(Rotation_Vec,Translation_Vec,K,cv::Mat(),frame,Axis,w_cap);
                    //draw(Rotation_Vec,Translation_Vec,K,cv::Mat(),frame,Axis,w_cap);
               }

            }
        }

        Pattern.debug();
        w_cap.release();


//// Clustering of descriptors


}
