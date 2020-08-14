
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "GxIAPI.h"
#include "DxImageProc.h"
#include "cameras.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <thread>
#include <unistd.h>

#define FLIP 1

using namespace cv;
using namespace std;

FileStorage fs;
std::string picpath = "./calibrationpics/";

Mat frame1;
Mat frame2;

string calib_func = "";

int pausetime = 5;

int frame_ID = 0;

int framerate = 90;
int multiplier = 45;
int pics = 40;

int done = 0;



void show_msg_in_feed(Mat frame, string message, int frame_ID)
{
  flip(frame, frame, 1);
  putText(frame, to_string(int(frame_ID / framerate)), Point(50, 50), FONT_HERSHEY_PLAIN, 4, cv::Scalar(0, 255, 100), 2);
  putText(frame, message, Point(50, 100), FONT_HERSHEY_PLAIN, 4, cv::Scalar(0, 255, 100), 2);
  resize(frame, frame, Size(640, 512));
  imshow("yeet", frame);
  waitKey(1);
}

void show_msg_in_double_feed(VideoCapture cap1, VideoCapture cap2, string message)
{
  for (int i = 0; i < 80; i++)
  {

    Mat frame1;
    Mat frame2;
    Mat concatframe;

    cap1 >> frame1;
    cap2 >> frame2;

    hconcat(frame1, frame2, concatframe);
    flip(concatframe, concatframe, 1);
    resize(concatframe, concatframe, Size(1280, 512));
    putText(concatframe, message, Point(50, 100), FONT_HERSHEY_PLAIN, 4, cv::Scalar(0, 255, 100), 2);
    imshow("yeet", concatframe);
    waitKey(1);
  }
}

void extrinsics_pics(String location, Mat frame1, Mat frame2, FileStorage fs, int frame_ID)
{
  Mat concatframe;
  std::string picfilename;

  if (frame_ID % multiplier == 0)
  {
    std::string nr = to_string(frame_ID * 2 / multiplier);
    picfilename = "pic" + std::string(5 - nr.length(), '0') + nr + ".jpg";
    string fullpath = location + picfilename;
    printf("%s\n", fullpath.c_str());
    imwrite(fullpath, frame1);
    fs << (fullpath).c_str();

    nr = to_string(frame_ID * 2 / multiplier + 1);
    picfilename = "pic" + std::string(5 - nr.length(), '0') + nr + ".jpg";
    fullpath = location + picfilename;
    imwrite(fullpath, frame2);
    fs << (fullpath).c_str();
  }

  hconcat(frame1, frame2, concatframe);
  cv::flip(concatframe, concatframe, 1);
  resize(concatframe, concatframe, Size(1280, 512));
  putText(concatframe, to_string(frame_ID / multiplier), Point(50, 50), FONT_HERSHEY_PLAIN, 4, cv::Scalar(0, 255, 100), 2);
  imshow("yeet", concatframe);
  waitKey(1);
}

void cam_pics(String location, Mat frame, FileStorage fs, int frame_ID)
{
  string picfilename;

  if (frame_ID % multiplier == 0)
  {
    string nr = to_string(int(frame_ID / multiplier));
    picfilename = "pic" + string(5 - nr.length(), '0') + nr + ".jpg";
    string fullpath = location + picfilename;
    imwrite(fullpath, frame);
    fs << (fullpath).c_str();
  }

  cv::flip(frame, frame, 1);
  cv::putText(frame, to_string(frame_ID / multiplier), Point(50, 50), FONT_HERSHEY_PLAIN, 4, cv::Scalar(0, 255, 100), 2);
  resize(frame, frame, Size(640, 512));
  cv::imshow("image", frame);
  cv::waitKey(1);
}

static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM *pFrame)
{

  
  cv::Mat image;
  image.create(pFrame->nHeight, pFrame->nWidth, CV_8UC1);
  image.data = (uchar *)pFrame->pImgBuf;

  cv::cvtColor(image, image, cv::COLOR_BayerRG2RGB);
  //cv::resize(image, image, cv::Size(1664, 832));

  if (FLIP == 1)
  {
    cv::flip(image, image, 0);
    cv::flip(image, image, 1);
  }

  frame1 = image(cv::Rect(0, 0, image.cols/2, image.rows));
  frame2 = image(cv::Rect(image.cols/2, 0, image.cols/2, image.rows));

  if (frame_ID < framerate * pausetime)
  {

    if (0 == strcmp(calib_func.c_str(), "extr"))
    {
      show_msg_in_feed(image, "extrinsics", frame_ID);
    }
    else if (0 == strcmp(calib_func.c_str(), "cam1"))
    {
      show_msg_in_feed(frame1, "intrinsics cam1", frame_ID);
    }
    else if (0 == strcmp(calib_func.c_str(), "cam2"))
    {
      show_msg_in_feed(frame2, "intrinsics cam2", frame_ID);
    }
    else if (0 == strcmp(calib_func.c_str(), "ground"))
    {
      show_msg_in_feed(frame1, "ground", frame_ID);
    }
    
  }
  else if (frame_ID >= framerate * pausetime)
  {

    int new_frame_ID = frame_ID - framerate * pausetime;

    if (0 == strcmp(calib_func.c_str(), "extr"))
    {
      extrinsics_pics(picpath + "extr/", frame1, frame2, fs, new_frame_ID);
    }
    else if (0 == strcmp(calib_func.c_str(), "cam1"))
    {
      cam_pics(picpath + "cam1/", frame1, fs, new_frame_ID);
    }
    else if (0 == strcmp(calib_func.c_str(), "cam2"))
    {
      cam_pics(picpath + "cam2/", frame2, fs, new_frame_ID);
    }
    else if (0 == strcmp(calib_func.c_str(), "ground"))
    {
      imwrite("groundpattern.jpg", frame1);
      done = 1;
    }
    
  }


  //cout << frame_ID << endl;
  frame_ID++;

  if (frame_ID == framerate * pausetime + multiplier * pics)
  {
    done = 1;
  }


return;
}

int main(int argc, char **argv)
{
  
  calib_func = argv[1];

  if (0 == strcmp(argv[1], "extr"))
  {
    fs = FileStorage("pairlist.xml", FileStorage::WRITE);
    fs << "strings"
       << "[";
  }
  else if (0 == strcmp(argv[1], "cam1"))
  {
    fs = FileStorage("imglist1.xml", FileStorage::WRITE);
    fs << "strings"
       << "[";
  }
  else if (0 == strcmp(argv[1], "cam2"))
  {
    fs = FileStorage("imglist2.xml", FileStorage::WRITE);
    fs << "strings"
       << "[";
  }



  Stereocamera cam(1, 16, 1000, 1.5, 1, 1.5, 90, GX_TRIGGER_MODE_OFF);
  cam.setCallback(OnFrameCallbackFun);
  cam.startCamera();

  while(done==0){
    usleep(100);
  }
}