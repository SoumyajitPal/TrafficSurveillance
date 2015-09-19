#include <iostream>
#include <cstdio>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <cstdlib>

using namespace std;
using namespace cv;

/*
Okay I'll walk you through the code. This was an earlier draft. The tracking part has been slightly modified later. That code is a mess as of yet.
This code is functionally identical, so I guess this should be good enough to understand the approach
*/

typedef struct info
{
    Rect r;
    Point Centroid;
    int frameNumber;
}TrackRecord;

int main()
{
    //Mat is an openCV matrix object
    Mat frame;
    Mat roi;
    Mat resizeF;
    Mat foreGround;
    Mat temp;
    Mat test;
    Mat prev;
    Mat buffer;


    int skip;
    int j;
    int frameSequence = 0;
    int a;
    int numberOfCars = 0;
    int wait = 0;

    TrackRecord track[6];
    for(int p=0; p<6; p++)
    {
        track[p].r = Rect(10, 20, 11, 21);
        track[p].frameNumber = 0;
        track[p].Centroid.x = 0;
        track[p].Centroid.y = 0;
    }

    //BackgroundSubtractor Class has the Background modelling methods

    BackgroundSubtractorMOG2 bg;
    Scalar color;

    //path to the video
    VideoCapture videoObject("/home/soumyajit/Documents/MOV01B.MOD");
    char text[20];


    //Extract a frame from the video and resize for connecting loosely connected components and 1st level of noise removal

    videoObject.read(frame);
    resize(frame, resizeF, Size(frame.size().width/4, frame.size().height/4));

    //A buffer Mat(all black) to plot centroids
    buffer = Mat::zeros(resizeF.size(), CV_8UC1);


    //A queue of Mat to store last ten frames
    //vector<Mat> frameRecord;

    //Extracting the region of interest rom the current frame
    resizeF.copyTo(prev);
    prev = prev(Rect(10,20,140,100));
    cvtColor(prev, prev, CV_BGR2GRAY);


    int posX, posY;


    Mat imgL = Mat::zeros(resizeF.size(), CV_8UC1);
    imgL = imgL(Rect(10,20,140,100));

    int refreshRate = 100;
    int Frequency = 0;
    int flag = 0;


    //vector of Contours in a frame
    vector<vector<Point> > contours;


    //This loop continues till the video has a next frame
    while(true)
    {
        //Extract a frame
        videoObject.read(frame);


        //Resize to 1/16th the size
        resize(frame, resizeF, Size(frame.size().width/4, frame.size().height/4));


        //Extract the background. This is an inbuilt method of BackgroundSubtractor Class. I had previously used manual Gaussian Mixture which was slower. So switched to this
        bg.operator()(resizeF, foreGround);
        //Now foreground hold the current foreground objects


        //This is noise removal and stabilization
        erode(foreGround, foreGround, Mat());
        dilate(foreGround,  foreGround, Mat());


        //Binarize the foreground to extract moving objects and save to roi(Region of Interest)
        threshold(foreGround, roi,100, 255, 3);
        roi = roi(Rect(10,20,140,100));
        roi.copyTo(temp);



        //This method maps the blobs in the foreground as a listof x,y coordinates
        findContours(temp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);



		vector<Rect> boundRect(contours.size());
        vector<vector<Point> > PolyBound(contours.size());
        int centX, centY;

        flag = 0;
		j = 0;
        for(unsigned int i = 0; i<contours.size(); i++)
        {
			if(contourArea(contours[i])>50)
            {
                //The "contours" or blobs are approximated to polygons
            	approxPolyDP(Mat(contours[i]), PolyBound[i], 3, true);

            	// x and y coordinate maxima and minima of each polygon are used to find bounding rectangle
                boundRect[j] = boundingRect(Mat(PolyBound[i]));
                test = roi(boundRect[j]);

                //Further filtering on the bounding rectangle  to enhance the shapes
                //dilate(test, test, Mat());
                threshold(test, test, 150, 255, 3);
                erode(test, test, Mat());
                test.copyTo(roi(boundRect[j]));

                j++;
            }

        }
        Moments moment = moments(contours[0]);
        centY = moment.m01/moment.m00;
        centX = moment.m10/moment.m00;




        //Draws the blobs' outlines
        //drawContours(roi, contours, -1, Scalar(255,255,255), 1);

        /*for(unsigned int p=0; p<boundRect.size(); p++)
            rectangle(resizeF, boundRect[0].tl() + Point(0, 10), boundRect[0].br() + Point(20, 30), Scalar(100, 200, 255));
        */


        int Strip;
        if(boundRect.size()>0)
        {

            posY = boundRect[0].br().y -13*(boundRect[0].height/20);
            posX = boundRect[0].br().x - (boundRect[0].width/2);


            Strip = posX/30;
            //cout <<posX << "  " << Strip << endl;
            TrackRecord previousFrameRect = track[Strip];



            if(boundRect[0].area() >= 0.5*(Rect(0,0,180,120).area()))
            {
                continue;
            }

            if(previousFrameRect.Centroid.x == 0 && previousFrameRect.Centroid.y == 0)
            {
                //Previous record doesnot exist
                //cout << "New Car in Strip" << endl;
                track[Strip].r = boundRect[0];
                track[Strip].Centroid.x = posX;
                track[Strip].Centroid.y = posY;
            }
            else if(previousFrameRect.r.br().y>95)
            {
                if(boundRect[0].br().y > 95)
                {
                    //Old Car. Don't Consider
                    //cout << "Old Car" << endl;
                }
                else if(boundRect[0].br().y < 80)
                {
                    //New Car
                    //cout << "Reposition Centroid" << endl;
                    track[Strip].r = boundRect[0];
                    track[Strip].Centroid.x = posX;
                    track[Strip].Centroid.y = posY;
                }
            }
            else
            {
                if((previousFrameRect.r.area() < boundRect[0].area())&& (boundRect[0].tl().y - previousFrameRect.r.tl().y >= 0) && boundRect[0].area() < 2*previousFrameRect.r.area())
                {
                    //Motion estimate

                    track[Strip].r = boundRect[0];
                    track[Strip].frameNumber++;
                    track[Strip].Centroid.x = posX;
                    track[Strip].Centroid.y = posY;

                }
                else
                {
                    Rect predictedRect = track[Strip].r&boundRect[0];

                    if(predictedRect.area()>50)
                    {
                        track[Strip].r = boundRect[0];
                        track[Strip].frameNumber++;
                        track[Strip].Centroid.x = posX;
                        track[Strip].Centroid.y = posY;
                    }

                }
                if(track[Strip].frameNumber>=3 && boundRect[0].br().y>=95)
                    {
                        cout << numberOfCars++ << endl;
                        track[Strip].frameNumber = 0;
                    }
            }










            int pX = track[Strip].r.br().x - (track[Strip].r.width/2);
            int pY = track[Strip].r.br().y - 13*(track[Strip].r.height/20);

            /*if(previousFrameRect.area() < boundRect[0].area())
            {
                if((track[Strip].area() > 0.5*(Rect(0,0,180,120).area()) || boundRect[0].br().y > previousFrameRect.br().y)&&(boundRect[0].area() < 0.5*(Rect(0,0,180,120).area())))
                {
                    track[Strip] = boundRect[0];
                }
            }*/
            //cout << frameSequence << "    " << centX << "  " << centY << "  " << posX << "  " << posY << endl;
            //circle(resizeF, Point(posX + 10,  posY  + 20), 1, Scalar(50,100,150), 2);
            circle(resizeF, Point((centX + pX)/2 + 10,  (centY + pY)/2  + 20), 1, Scalar(255,100,10), 2);
            rectangle(resizeF, boundRect[0].tl() + Point(0, 10), boundRect[0].br() + Point(20, 30), Scalar(100, 200, 255));
            /*Rect r = boundRect[0]&track[Strip].r;
            rectangle(resizeF, r.tl() + Point(0, 10), r.br() + Point(20, 30), Scalar(180, 40, 200));
            if(r.area() > 0.9*boundRect[0].area() && r.br().y>110)
                cout << numberOfCars++ << endl;
            //rectangle(resizeF, r, Scalar(180, 40, 200));
            */

        }

        rectangle(resizeF, track[Strip].r.tl() + Point(0, 10), track[Strip].r.br() + Point(20, 30), Scalar(200, 200, 90));

        //This part just displays the formatted video with tracking and all.

        roi = roi + imgL;
        //imshow("Test", imgL);


        imshow("ROI", roi);
        //cout << numberOfCars << endl;

        if(waitKey(20)>=0)
            break;

        Frequency++;
        if(Frequency>=refreshRate)
        {
            Frequency = 0;
            imgL = Mat::zeros(resizeF.size(), CV_8UC1);
            imgL = imgL(Rect(10,20,140,100));
        }

        sprintf(text, " Frame %d", frameSequence++);
        putText(resizeF, text, Point(10, 10), FONT_HERSHEY_PLAIN, 0.8, Scalar(250, 210, 200, 255));
        imshow("Original", resizeF);
        //frameRecord.push_back(resizeF.clone());
        //cout << frameRecord.size() <<endl;
        /*if(frameRecord.size()==10)
        {
            frameRecord.erase(frameRecord.begin());
        }
        int x = 0;
        for(vector<Mat>::iterator v = frameRecord.begin(); v!=frameRecord.end(); v++)
        {
            imshow(str[x++], *v);
        }*/
        //getchar();





    }
}
