#include "3d_functions.hpp"
#include "cameras.hpp"

#include <queue>
#include <thread>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <unistd.h>

#define FLIP 1

int iLowH = 42;
int iHighH = 81;

int iLowS = 40;
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;

projector3D projector(Size(PATTERN_WIDTH, PATTERN_HEIGHT), PATTERN_SQUARE_SIZE);

void postprocess_contours(Mat &imgcontours)
{
  //morphological opening (remove small objects from the foreground)
  erode(imgcontours, imgcontours, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
  dilate(imgcontours, imgcontours, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));

  //morphological closing (fill small holes in the foreground)
  dilate(imgcontours, imgcontours, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
  erode(imgcontours, imgcontours, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
}

Point get_biggest_contour_center(vector<vector<Point>> objcontours)
{
  vector<Point> biggestcontour;
  int biggestcontour_size = 0;
  int biggestcontour_x = 0;
  int biggestcontour_y = 0;

  for (int cntr = 0; cntr < objcontours.size(); cntr++)
  {
    if (objcontours[cntr].size() > biggestcontour_size)
    {
      biggestcontour_size = objcontours[cntr].size();
      biggestcontour = objcontours[cntr];
    }
  }

  Moments M = moments(biggestcontour);
  int cX = M.m10 / M.m00;
  int cY = M.m01 / M.m00;

  return cv::Point(cX, cY);
}

static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM *pFrame)
{

  Mat image;
  image.create(pFrame->nHeight, pFrame->nWidth, CV_8UC1);
  image.data = (uchar *)pFrame->pImgBuf;

  cvtColor(image, image, cv::COLOR_BayerRG2RGB);

  if (FLIP == 1)
  {
    flip(image, image, 0);
    flip(image, image, 1);
  }

  Mat frame1 = image(cv::Rect(0, 0, image.size().width / 2, image.size().height));
  Mat frame2 = image(cv::Rect(image.size().width / 2, 0, image.size().width / 2, image.size().height));

  Mat frame1_b;
  Mat frame2_b;

  cvtColor(frame1, frame1_b, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  cvtColor(frame2, frame2_b, COLOR_BGR2HSV);

  inRange(frame1_b, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), frame1_b); //Threshold the image
  inRange(frame2_b, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), frame2_b);

  postprocess_contours(frame1_b);
  postprocess_contours(frame2_b);

  vector<Vec4i> hierarchy;


  vector<vector<Point>> contours1;
  vector<vector<Point>> contours2;
  findContours(frame1_b, contours1, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  findContours(frame2_b, contours2, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

  cv::Point p1 = get_biggest_contour_center(contours1);
  cv::Point p2 = get_biggest_contour_center(contours2);

  circle(image,p1, 16, Scalar(255, 255, 255), 5);
  circle(image, cv::Point(p2.x + frame1.cols, p2.y), 16, Scalar(255, 255, 255), 5);

  //HERE EXTRACT A POINT FROM BOTH IMAGES AND INSERT THEM INTO object_location()

  //own code goes here

  Point3f object3D_POS = projector.object_position(p1, p2, true);

  std::cout << object3D_POS << std::endl;

  cvtColor(frame1_b, frame1_b, COLOR_GRAY2BGR);
  cvtColor(frame2_b, frame2_b, COLOR_GRAY2BGR);
  hconcat(frame1_b, frame2_b, frame1_b);

  resize(frame1_b, frame1_b, cv::Size(image.cols / 3, image.rows / 3));
  resize(image, image, cv::Size(image.cols / 3, image.rows / 3));

  putText(frame1_b, "x: " + to_string(object3D_POS.x/100) + "m", Point(10,30), FONT_HERSHEY_PLAIN, 2, Scalar(255, 255,255), 1);
  putText(frame1_b, "y: " + to_string(object3D_POS.y/100) + "m", Point(10,60), FONT_HERSHEY_PLAIN, 2, Scalar(255, 255,255), 1);
  putText(frame1_b, "z: " + to_string(object3D_POS.z/100) + "m", Point(10,90), FONT_HERSHEY_PLAIN, 2, Scalar(255, 255,255), 1);

  vconcat(image, frame1_b, image);
  imshow("camera", image);
  //imshow("camera_filtered", imgThresholded);
  int key = waitKey(1);
  if (key == 27)
  {
    cv::destroyAllWindows();
    exit;
  }
}

int main(int argc, char *argv[])
{

  namedWindow("camera");

  //Create trackbars in "Control" window
  createTrackbar("LowH", "camera", &iLowH, 179); //Hue (0 - 179)
  createTrackbar("HighH", "camera", &iHighH, 179);

  createTrackbar("LowS", "camera", &iLowS, 255); //Saturation (0 - 255)
  createTrackbar("HighS", "camera", &iHighS, 255);

  createTrackbar("LowV", "camera", &iLowV, 255); //Value (0 - 255)
  createTrackbar("HighV", "camera", &iHighV, 255);

  Stereocamera cam(1, 16, 1000, 1.4, 1, 1.6, 30, GX_TRIGGER_MODE_OFF);
  cam.setCallback(OnFrameCallbackFun);
  cam.startCamera();

  while (1)
  {
    usleep(100);
  };

  return 0;
}