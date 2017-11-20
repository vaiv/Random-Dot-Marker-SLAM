#include <opencv2/opencv.hpp>
#include <iostream>
#include "detectcircles.h"
#include "dot.h"
#include "dotmarkers.h"
using namespace cv;


//int main(int argc, char *argv[])
//{
//    string source;
//    VideoCapture reader;
//    if(strcasecmp(argv[1],"live")==0)
//    {
//        std::cout<<"enter camera index"<<std::endl;
//        int c;
//        std::cin>>c;
//        reader.open(c);
//        std::cout<<"sourcing from video"<<source<<std::endl;
//    }
//    else
//    reader.open(argv[1]);

//    char kb;
//    DotMarkers Pattern;
//    while(reader.grab() && kb!='q')
//    {
//        Mat frame;
//        reader.retrieve(frame);
//        cv::imshow("original",frame);

//        DetectCircles detector(frame);
//        detector.drawCircles();
////        cv::waitKey(0);
//        vector<Vec3f> Circles = detector.getDetectedCircles();

//        for(int i=0;i<Circles.size();i++)
//        {   if(Circles.size()>8)
//            {
//               // cout<<"trying to locate markers"<<endl;
//                Dot Marker(Circles[i],frame,8,7,0.8f,0.9f,10);
//                Marker.setCenters(detector.getCenters());
//                //cout<<"getting Descriptors"<<endl;
//                Marker.assignDescriptors();
//                if(!Pattern.find(Marker))
//                    Pattern.insert(Marker);
//                if(Marker.getId()>0)
//                Marker.draw(frame);
//                //imshow("neighbors",frame);
//                //cv::waitKey(0);

//            }

//        }
//        imshow("detected_markers",frame);
//         kb=cv::waitKey(60);
//    }
//}

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
        while(reader.grab() && kb!='q')
        {
            cv::Mat dist_frame,frame;
            reader.retrieve(dist_frame);
            cv::undistort(dist_frame,frame,K,D);
            DetectCircles detector(frame);
            detector.drawCircles();
            vector<Vec3f> Circles = detector.getDetectedCircles();
            std::vector<Dot> tmp;
            Mat Rotation_Vec,Translation_Vec;
            if(Circles.size()>12)
            {

                for(int i=0;i<Circles.size();i++)
                {
                    Dot Marker(Circles[i],frame,8,7,0.8f,0.9f,100);
                    Marker.setCenters(detector.getCenters());
                    Marker.assignDescriptors();
                    if(!Pattern.find(Marker) && !setPattern )
                        Pattern.insert(Marker);

                    if(Marker.getId()>0)
                       {
                        tmp.push_back(Marker);
                        Marker.draw(frame);
                    }

                }
                if(Pattern.getMaxId()>100)
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

                for(int i=0;i<Markers_1.size();i++)
                {
                    corres_1.push_back(Markers_1[i].getCenter());
                    corres_2.push_back(Markers_2[i].getCenter());
                }

                if(corres_1.size()>8)
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
                    cout<<"RMS reproj error is "<<sum;


                    for(int j=0;j<Coords3D.size();j++)
                    {
                        Dot Marker = Markers_2[j];
                        if(!Pattern.set3DPos(Marker,Coords3D[j]))
                            cout<<"this should not have happenend, an already located marker could not be found!"<<endl;
                    }

                    cv::Point3f origin = cv::Point3f(0,0,0);
                    Axis.push_back(origin);
                    Axis.push_back(cv::Point3f(origin.x + 0.16,origin.y,origin.z));
                    Axis.push_back(cv::Point3f(origin.x ,origin.y + 0.16,origin.z));
                    Axis.push_back(cv::Point3f(origin.x ,origin.y,origin.z - 0.16));

                    pos3D = true;
                }

            }

            if(pos3D) //find pose using PnP and check reprojection error
            {
                cv::Point3f Point3D;
                std::vector<Dot> known3DMarkers;
               for(int k=0;k<tmp.size();k++)
               {
                   cv::Point3f point3D;
                   if(Pattern.get3DPos(tmp[k].getId(),Point3D))
                   {
                       tmp[k].set3DPos(Point3D);
                       known3DMarkers.push_back(tmp[k]);
                   }

               }

               if(known3DMarkers.size()>8)
               {
                   std::vector<Point2f> imgPoints;
                   std::vector<Point3f> worldPoints;


                   for(int k=0;k<known3DMarkers.size();k++)
                   {
                       imgPoints.push_back(known3DMarkers[k].getCenter());
                       worldPoints.push_back(known3DMarkers[k].get3DPos());
                   }

                   if(Rotation_Vec.empty() || Translation_Vec.empty())
                        cv::solvePnP(worldPoints,imgPoints,K,cv::Mat(),Rotation_Vec,Translation_Vec);
                   else
                       cv::solvePnP(worldPoints,imgPoints,K,cv::Mat(),Rotation_Vec,Translation_Vec,true);

                   cout<<"rotation vector is "<<Rotation_Vec<<endl<<"translation vector is"<<Translation_Vec<<endl;
                   DrawAxis(Rotation_Vec,Translation_Vec,K,cv::Mat(),frame,Axis,w_cap);
               }

            }
        }
        w_cap.release();
}
