#include "3d_functions.hpp"
#include "cameras.hpp"

#include <queue>
#include <thread>


#define FLIP 0
#define BOARD_WIDTH 9
#define BOARD_HEIGHT 6
#define SQUARE_SIZE 3.78f



projector3D projector(Size(BOARD_WIDTH,BOARD_HEIGHT),SQUARE_SIZE);


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

  Mat frame1 = image(cv::Rect(0, 0, image.size().width/2, image.size().height));
  Mat frame2 = image(cv::Rect(image.size().width/2, 0, image.size().width/2, image.size().height));

    Point2i leftPoint;
    Point2i rightPoint;


    //HERE EXTRACT A POINT FROM BOTH IMAGES AND INSERT THEM INTO object_location()

    //own code goes here

    //projector.object_position(leftPoint, rightPoint, true);


    imshow("camera", image);
    int key = waitKey(1);
    if(key == 27)
    {
        cv::destroyAllWindows();
        exit;
    }


  
}

int main(int argc, char *argv[])
{
    
    Stereocamera cam(1, 16, 10000, 1.5, 1, 1.5, 30, GX_TRIGGER_MODE_OFF);
    cam.setCallback(OnFrameCallbackFun);
    cam.startCamera();

    return 0;
}