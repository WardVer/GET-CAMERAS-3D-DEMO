#include "3d_functions.hpp"


int projector3D::read_extrinsics()
{

    /*
        * read the extrinsic parameters of the cameras that we achieved from the calibration and saved in a file
        * This is a combination of the rotation and translation of the second camera compared to the first camera
        * 
        * This is needed to be able to locate a point in 3D with stereo vision
        * 
        * This also makes it possible to project a point from the 3D coordinate system of one camera to the 3D coordinate system of the other camera.
        */

    FileStorage fs;
    fs.open("extrinsics.xml", FileStorage::READ);

    if (!fs.isOpened())
    {
        cerr << "Failed to open extrinsics" << endl;
        return 1;
    }

    FileNode node = fs["R"];
    node >> rotation_relative;

    node = fs["T"];
    node >> translation_relative;

    fs.release();

    return 0;
}

int projector3D::read_intrinsics()
{

    /*
        * read the intrinsic parameters of the cameras that we achieved from the calibration and saved in a file
        * This is a combination of the camera matrix K and distortion matrix D of both cameras.
        * The combination of these matrices describes how the camera projects a 3D point to its 2D image plane
        * 
        * To calculate the location on the image of an object located in the 3D coordinate system of the camera, simply multiply
        * the 3D point vector with the camera matrix K of the camera that it was recorded on.
        */

    FileStorage fs;
    fs.open("intrinsics.xml", FileStorage::READ);

    if (!fs.isOpened())
    {
        cerr << "Failed to open extrinsics" << endl;
        return 1;
    }

    FileNode node = fs["K1"];
    node >> K1;

    node = fs["D1"];
    node >> D1;

    node = fs["K2"];
    node >> K2;

    node = fs["D2"];
    node >> D2;

    fs.release();

    return 0;
}

void projector3D::cam_pose_to_origin()

{

    /*
        * In this function we calculate the rotation and translation of the ground relative to the camera. 
        * The results are based in the 3D coordinate system of the camera.
        * after running this function the class parameters translation_ground and rotation_ground will be set.
        * While calculating the 3D project of a point you can then choose to pick the camera or ground coordinate system
        */


    //initialize imgPts and objPts.
    //imgPts will be the pixel coordinates of the ground pattern points on our image.
    //objPts will be the real life coordinates of the ground pattern with the origin the top right corner.
    vector<Point2f> imgPts;
    vector<Point3f> objPts;

    Mat rvec;
    Mat tvec;

    bool found = false;

    Mat groundpicture = imread("groundpattern.jpg");
    Mat gray;
    cvtColor(groundpicture, gray, COLOR_BGR2GRAY);

    //find the chessboard pattern points on our image
    found = cv::findChessboardCorners(gray, boardSize, imgPts,
                                      CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
    if (found)
    {
        cornerSubPix(gray, imgPts, Size(5, 5), Size(-1, -1),
                     TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.1));
    }
    else
    {
        std::cout << "ground pattern not found" << std::endl;
        return;
    }
    

    //fill in objPts with the real life chessboard pattern point coordinates
    for (int j = 0; j < boardSize.height; j++)
        for (int k = 0; k < boardSize.width; k++)
            objPts.push_back(Point3f(j * squareSize, k * squareSize, 0));

    //find the rotation and translation of the pattern in the 3D camera coordinate system
    solvePnP(objPts, imgPts, K1, D1, rvec, tvec);

    Mat newvec;
    Mat newvec_t;

    //convert rotation vector format to rotation matrix format, we can't use the vector anywhere but
    //the matrix is very useful for easy rotation calculations.
    Rodrigues(rvec, newvec);

    //copy the rotation vector into a temporary one to avoid copying already changed columns
    newvec.copyTo(newvec_t);

    /*the axi of the pattern are a bit messed up but here I switch the axes around to make
        *Z forward
        *X right
        *Y up
        */
    Mat column = (newvec_t.col(2));
    column.copyTo(newvec.col(1));

    column = (newvec_t.col(1));
    column.copyTo(newvec.col(0));

    column = (0 - newvec_t.col(0));
    column.copyTo(newvec.col(2));

    rotation_ground = newvec;
    translation_ground = tvec;
}



Point3f projector3D::object_position(Point2f point1, Point2f point2, bool ground_coords)
{
    //get the 3D position of the object from 2 pixel coordinates
    Mat points3D_t;

    vector<Point2d> Points2D_1;
    vector<Point2d> Points2D_2;

    Points2D_1.push_back(point1);
    Points2D_2.push_back(point2);


    triangulatePoints(P1, P2, Points2D_1, Points2D_2, points3D_t);

    //some transforms to get a usable 3D vector format
    Mat points3D_2;
    Mat points3D_r = points3D_t.t();
    points3D_r = points3D_r.reshape(4);
    cv::convertPointsFromHomogeneous(points3D_r, points3D_2);

    Mat positions[3];

    cv::split(points3D_2, positions);

    
    Mat new_3D_points = (Mat_<double>(3, 1) << positions[0].at<double>(0), positions[1].at<double>(0), positions[2].at<double>(0));
    
    if(ground_coords)
    { 
        new_3D_points = rotation_ground.inv() * ((new_3D_points) - translation_ground); 
    }
    
    Point3f final_position = Point3f(new_3D_points.at<double>(0,0), new_3D_points.at<double>(1,0), new_3D_points.at<double>(2,0));

    return final_position;
}
