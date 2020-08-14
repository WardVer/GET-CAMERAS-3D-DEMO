#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/videoio/videoio.hpp>
#include "opencv2/features2d.hpp"


#include <chrono>



using namespace cv;
using namespace std;

#define PATTERN_WIDTH 9 
#define PATTERN_HEIGHT 6
#define PATTERN_SQUARE_SIZE 3.78 //in centimeters



class projector3D
{
public:
    projector3D(cv::Size board_size, float square_size)
    {
        /*  
        *read the different parameters of our cameras
        * extrinsics is a combination of the relative rotation and translation
        * of the 2 cameras.
        * intrinsics describe how a camera projects 3D points to the 2D image plane.
        * 
        * more info in their respective read functions
        */

        read_extrinsics();
        read_intrinsics();


        hconcat(rotation_relative, translation_relative, P2);
        P2 = K2 * P2;

        //projection matrix P1 describes the way a 3D point in real distance gets transformed into pixel coordinates.
        
        P1 = (cv::Mat_<double>(3, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
        P1 = K1 * P1;

        boardSize = board_size;
        squareSize = square_size;

        cam_pose_to_origin();
        
    }

    projector3D() {}

    Point3f object_position(Point2f point1, Point2f point2, bool ground_coords);

    
private:
    cv::Mat rotation_relative;
    cv::Mat translation_relative;

    cv::Mat K1;
    cv::Mat D1;
    cv::Mat K2;
    cv::Mat D2;

    cv::Mat P1;
    cv::Mat P2;

    cv::Mat rotation_ground;
    cv::Mat translation_ground;

    cv::Size boardSize;
    float squareSize;

    int read_extrinsics();

    int read_intrinsics();

    void cam_pose_to_origin();

 };