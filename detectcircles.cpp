#include "detectcircles.h"

float distBetween(cv::Point2f pt1,cv::Point2f pt2)
{
    return sqrt(pow(pt1.x-pt2.x,2) + pow(pt1.y-pt2.y,2));
}

cv::Mat DetectCircles::filterImage()
{
    cv::Mat gray;
    cv::cvtColor(inImage,gray,cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray,gray,cvSize(9,9),2,2);
    return gray;
}

void DetectCircles::binarize()
{
    cv::Mat gray = filterImage();
    binaryMask = gray.clone();

}

//alternative strategy:- approx contour area by Pi*A*B with set relaxation threshold to select eligible contours

DetectCircles::DetectCircles(cv::Mat img)
{
    inImage = img.clone();
    binarize();
    cv::imshow("binarized",binaryMask);
//    cv::HoughCircles(binaryMask, Circles, CV_HOUGH_GRADIENT,
//             1,   // accumulator resolution (size of the image / 2)
//             20,  // minimum distance between two circles
//             200, // Canny high threshold
//             20, // minimum number of votes
//             50,200); // min and max radius
    vector<cv::Vec4i> hierarchy;
     std::vector< std::vector<cv::Point> > Contours;
    //cv::adaptiveThreshold(binaryMask,binaryMask,127,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY,21,2);
     cv::threshold(binaryMask,binaryMask,100,255,0);
    cv::imshow("binarized",binaryMask);
    cv::waitKey(20);
    cv::findContours(binaryMask,Contours,hierarchy,cv::RETR_CCOMP,cv::CHAIN_APPROX_NONE);
        for(int j=0;j<Contours.size();j++)
        {
//            cv::Point2f Center(Circles[i][0],Circles[i][1]);
////            if(cv::pointPolygonTest(Contours[j],Center,false) && cv::boundingRect(Contours[j]).area()<1000 && cv::boundingRect(Contours[j]).area()>100)
//            cv::Point2f boxCenter;
//            boxCenter.x = cv::boundingRect(Contours[j]).x + cv::boundingRect(Contours[j]).width/2;
//            boxCenter.y = cv::boundingRect(Contours[j]).y + cv::boundingRect(Contours[j]).height/2;
//             if(distBetween(boxCenter,Center)<10 && cv::boundingRect(Contours[j]).area()<1000 && cv::boundingRect(Contours[j]).area()>100)
//                Elliptical_Contours.push_back(Contours[j]);
            float Pi=3.14159;
            cv::Rect boundingBox = cv::boundingRect(Contours[j]);
            float area = Pi*(boundingBox.width/2)*(boundingBox.height/2);

            if(fabs(cv::contourArea(Contours[j])-area)<100 && cv::boundingRect(Contours[j]).area()>100)
               Elliptical_Contours.push_back(Contours[j]);
        }


    retrieveEllipses();
    //cv::HoughCircles(binaryMask,Circles,cv::HOUGH_GRADIENT,1,10);
    //cv::HoughCircles(binaryMask,Circles,cv::HOUGH_GRADIENT,1,10);
}

void DetectCircles::retrieveEllipses()
{
    for(int i=0;i<Elliptical_Contours.size();i++)
    {
       if(Elliptical_Contours[i].size()>5)
       {
           cv::RotatedRect Ellipse = cv::fitEllipse(Elliptical_Contours[i]);
           Ellipses.push_back(Ellipse);

           cv::Vec3f Circle;
           Circle[0] = Ellipse.center.x;
           Circle[1] = Ellipse.center.y;
           Circle[2] = Ellipse.boundingRect().width/2;
           Circles.push_back(Circle);
       }
    }
}

bool notRedundant(cv::Point Center,std::vector<cv::Point2f> Centers )
{
    for(int i=0;i<Centers.size();i++)
    {
        if(sqrt(pow(Center.x - Centers[i].x,2)+pow(Center.y-Centers[i].y,2))<10)
            return false;
    }
    return true;
}

std::vector<cv::Point2f> DetectCircles::getCenters()
{
    vector<cv::Point2f> Centers;

    for(int i=0;i<Ellipses.size();i++)
    {
        if(notRedundant(Ellipses[i].center,Centers))
        Centers.push_back(Ellipses[i].center);
    }


    return Centers;
}

std::vector<cv::Vec3f> DetectCircles::getDetectedCircles()
{
    return Circles;
}

void DetectCircles::drawCircles()
{
//    for( size_t i = 0; i < Circles.size(); i++ )
//        {
//             cv::Point center(cvRound(Circles[i][0]),cvRound(Circles[i][1]));
//             int radius = cvRound(Circles[i][2]);
//             // draw the circle center
//             //cv::circle( inImage, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
//             // draw the circle outline
//             cv::circle( inImage, center, radius, cv::Scalar(0,0,255), 1, 8, 0 );
//        }

    for(size_t i=0;i<Ellipses.size();i++)
    {
        cv::RotatedRect Ellipse=Ellipses[i];
        cv::ellipse(inImage,Ellipse,cvScalar(255,0,0));
        cv::circle( inImage, Ellipse.center, 1, cv::Scalar(0,255,0), -1, 8, 0 );
    }

        cv::imshow("detected circles",inImage);
}
