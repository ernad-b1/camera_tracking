#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>
#include "SimpleSerial.h"
#include <Windows.h>
#include <string.h>
#include <thread>
#include <mutex>

bool sendData(SimpleSerial& Serial, string data);
int getPoz(int error, int poz);
void procces(void);
int  getError(cv::Mat frame, cv::Rect bRect);
cv::Mat getMotion(cv::Mat firstFrame, cv::Mat frame);
cv::Mat getMotion(cv::Mat firstFrame, cv::Mat frame);
int getPozY(int error, int poz);



int movePozX = 0;
int errorXp = 0;
int errorYp = 0;

std::mutex g_i_mutex;

int main()
{
    cv::VideoCapture cap;  // Open the default camera or specify the camera index
    cap.open(0);
    if (!cap.isOpened()) {
        std::cout << "Error opening camera" << std::endl;
        return -1;
    } 

    cv::Mat prevFrame, currFrame;
    std::vector<cv::Point2f> prevPoints, currPoints;
    cv::Mat status, err;
    cv::Point2f pointX;

    // Read the first frame
    cap >> prevFrame;
    cv::cvtColor(prevFrame, prevFrame, cv::COLOR_BGR2GRAY);

    // Initialize feature points in the first frame
    cv::goodFeaturesToTrack(prevFrame, prevPoints, 500, 0.3, 7);
    prevPoints[0] = cv::Point(320, 120);

    thread proc(procces);
    proc.detach();

    while (true) {
        // Read the current frame
        cap >> currFrame;
        cv::Mat frame;
        frame = currFrame.clone();

        cv::GaussianBlur(currFrame, currFrame, cv::Size(21, 21), 0);
        if (currFrame.empty())
            break;
        cv::cvtColor(currFrame, currFrame, cv::COLOR_BGR2GRAY);

        // Calculate optical flow
        cv::calcOpticalFlowPyrLK(prevFrame, currFrame, prevPoints, currPoints, status, err);

        // Filter and draw the tracked points
        /*
        for (int i = 0; i < prevPoints.size(); i++) {
            if (status.at<uchar>(i) == 1) {
                pointX.x += currPoints[i].x;
                pointX.y += currPoints[i].y;
                cv::circle(currFrame, currPoints[i], 3, cv::Scalar(0, 255, 0), -1);
                cv::line(currFrame, prevPoints[i], currPoints[i], cv::Scalar(0, 255, 0), 1);
            } 
        }
        */ 
          
        

        if (status.at<uchar>(0) != 1) { 
            currPoints[0] = cv::Point(320, 120);
        }
        cv::circle(currFrame, currPoints[0], 10, cv::Scalar(0, 255, 0), -1);
        cv::circle(frame, currPoints[0], 10, cv::Scalar(0, 255, 0), -1);


        std::cout << "Point 0: " << currPoints[0].x << std::endl;
        errorXp = currPoints[0].x - 320;
        errorYp = currPoints[0].y - (frame.size().height / 2);
         

        // Update the previous frame and points
        prevFrame = currFrame.clone();
        prevPoints = currPoints;

        // Display the frame with tracked points
        cv::imshow("Object Tracking", currFrame);
        cv::imshow("frame", frame);

        if (cv::waitKey(1) == 'q')
            break;
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}


bool sendData(SimpleSerial& Serial, string data) {

    char* to_send = &data[0];
    bool is_sent = Serial.WriteSerialPort(to_send);

    if (is_sent) {
        return true;
    }
    else
    {
        return false;
    }
}

int getPoz(int error, int poz) {
    int newpoz = 0;
    if (error > 50) {
        newpoz = poz - 3;
    }
    else if (error < -50) {
        newpoz = poz + 3;
    }
    else {
        newpoz = poz;
    }

    if (newpoz > 179) newpoz = 180;
    if (newpoz < 0) newpoz = 0;

    return newpoz;
}

void procces(void) {

    char com_port[] = "\\\\.\\COM4";
    DWORD COM_BAUD_RATE = CBR_9600;
    SimpleSerial Serial(com_port, COM_BAUD_RATE);

    if (Serial.connected_) {
        std::cout << "Serial port open" << std::endl;
    }
    else {
        std::cout << "Can't open port" << std::endl;
    }

    int pozX = 90;
    int pozY = 110;
    int movePozXp = 0;
     
    //sendData(Serial, "90");
    while (true)
    {
       // const std::lock_guard<std::mutex> lock(g_i_mutex);
        pozX = getPoz(errorXp, pozX);
        pozY = getPozY(errorYp, pozY);
        string spozX = to_string(pozX);
        string spozY = to_string(pozY);
        string poz = spozX;
        poz += 'x';
        poz += spozY;

        std::cout << "From thread: errorXp:"<<errorXp<<", pozX"<<pozX << std::endl;
        std::cout << "From thread: errorYp:" << errorYp <<", pozY" << pozY << std::endl;
        std::cout << "Poz: " << poz << std::endl;
        sendData(Serial, poz);  
        Sleep(1000);
    }     
     
    Serial.CloseSerialPort();

}

int  getError(cv::Mat frame, cv::Rect bRect) {
    int pozx = bRect.x + bRect.width / 2;
    int pozy = bRect.y + bRect.height / 2;
    int centerX = frame.cols / 2;
    int centerY = frame.rows / 2;
    int errorX = pozx - centerX;
    return errorX;
}

cv::Mat getMotion(cv::Mat firstFrame, cv::Mat frame) {
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    cv::cvtColor(firstFrame, firstFrame, cv::COLOR_BGR2GRAY);

    cv::GaussianBlur(frame, frame, cv::Size(21, 21), 0);
    cv::GaussianBlur(firstFrame, firstFrame, cv::Size(21, 21), 0);

    cv::Mat delta;

    cv::absdiff(firstFrame, frame, delta);
    cv::threshold(delta, delta, 80, 255, cv::THRESH_BINARY);
    cv::dilate(delta, delta, cv::Mat(), cv::Point(-1, -1), 2);
    
    return delta; 
}         

int getPozY(int error, int poz) {
    int newpoz = 0;
    if (error > 50) {
        newpoz = poz + 3;
    } 
    else if (error < -50) {
        newpoz = poz - 3;
    }   
    else {
        newpoz = poz;
    }  
              
    if (newpoz > 140) newpoz = 140;
    if (newpoz < 60) newpoz = 60;     
      
    return newpoz; 
}   