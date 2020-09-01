# Image detection with stereo cameras

This is a very basic representation of the image detection for Formula Electric Belgium for driving our car autonomously. But most importantly it serves as an example on how to use stereo cameras (more specifically those provided my [Get-Cameras.com](https://www.get-cameras.com/Image-Processing-Products)).

Our original software uses complex neural networks and other techniques to get a more accurate result and to give distinction between all the different cones, but here we replaced that with a basic color filter to do object detection.

all you need is a bright object of just 1 color (a color that shouldn't be present anywhere else in the room) and hold it in front of the camera.


## used hardware 

* Get-Cameras (Daheng Imaging) Stereo camera [VEN-134-90U3C-D](https://www.get-cameras.com/USB3-Camera-1.3MP-Color-Python-1300-Dual-Head-VEN-134-90U3C-D)

## used software

* Ubuntu 18.04

* Opencv 4.2.0
 
* cmake

* Daheng imaging camera drivers



## Post-installation setup

After everything has been compiled and installed, you will have to edit the CmakeList.txt

* You have to make sure that the line "find_package( OpenCV REQUIRED )" finds the exact OpenCV version that you compiled before.
* you need to link the headers of the camera drivers (add their include folder) in the cmakelists.txt file, they are called DxImageProc.h and GxIAPI.h

## Usage

### installation

if everything is linked correctly in the cmakelists.txt file then simply run 'cmake .' and 'make' within the main folder

### Calibration

This step is done with OpenCV as they provide a relatively easy to use calibration package. First you need a checkerboard pattern like this one https://github.com/opencv/opencv/blob/master/doc/pattern.png

Print it out and make sure that you can't bend it and that it is as big as possible. The best solution would be to glue it to something like a wooden board. It is also very important that the pattern has an even amount of squares in 1 dimension and an uneven amount in the other, like 9 squares by 6 squares for example. Keep into account that OpenCV expects you to fill in the amound of square intersections, not the amount of squares. This means you need to substract 1 in each dimension. If your pattern is 11 x 8 squares then you need to fill in 10 x 7.

### MAKE SURE TO EDIT THE EXPECTED PROPERTIES OF THE PATTERN IN THE 3d_functions.hpp FILE.
these properties are 

#define PATTERN_WIDTH  
#define PATTERN_HEIGHT
#define PATTERN_SQUARE_SIZE


1. take pictures for getting the intrinsic parameters of the camera

 use the line "calibration_pics cam1" AND "calibration_pics cam2" to take pictures for both cameras seperately

 this will activate the cameras to take 40 pictures in total, make sure the pattern is always visible. Hold the pattern close and move around slowly.
 
 

2. take pictures for getting the extrinsic parameters of the camera

 use the line "./calibration_pics extr"

 this will activate the cameras to take another 40 pictures in total, make sure the pattern is always visible on BOTH cameras. Hold the pattern close and move around slowly.



3. take a picture to find the orientation of the cameras compared to the ground

 Put the pattern on the ground and aim the the pattern forward (the side that's usually up has to point forward).

 use the line "./calibration_pics ground"
 
 
4. run the calibration

 use the line "./calibrate"
 
 The result should be RMS < 0.5, higher could possibly work but will be less accurate
 
 ### Running
 
use the line "./demo"

after this you will see this image:

![](https://github.com/WardVer/GET-CAMERAS-3D-DEMO/blob/master/Screenshot%202020-08-14%2014:40:14.png)

Here you can adjust the sliders H (Hue), S (Saturation), and V (Value) until only your desired object is visible on the bottom images.

From here on out the program will calculate the 3D location from the images. THE 3D Location has the top left square as origin point and also uses the pattern as base for x, y and z axis. The pattern is not required anymore after the image within the calibration phase has been taken.





## explanation of the calibration

The first step is the calibration of the cameras. For the software to triangulate a 3D position from 2 2D coordinates on our images, the software first has to know where the cameras are located and how the cameras project a 3D world into 2D.

OpenCV is a very powerful tool for this. It provides many functions to handle all of this.

The first concept for camera calibration is based on the pin-hole model of a camera. The result of the calibration are 2 matrices that describe their location, rotation and how the camera views the world. These 2 matrices are also achieved by 2 different kinds of calibration. There is intrinsic and extrinsic calibration. In both forms of calibration the camera has to see a specific pattern or image to be able to calibrate. This can be any pattern that is easily detectable and we also need the exact dimensions of the pattern. Most commonly people use a checkerboard pattern of black and white squares, because it is easy to recognize and easy to measure.

The first matrix retrieved from the intrinsic calibration contains all the parameters that describe how the camera projects the 3D world to a 2D image. This is very useful because we can simply multiply the 3D coordinates of an object with the intrinsic matrix to achieve the 2D location or the pixel coordinate of the object on the image. Note that these 3D coordinates have the camera as the origin point, with the Z axis coming straight out of the front of the camera, the Y axis coming from the top of the camera and the Z axis on the right of the camera. This can be done for each camera separately. Just show each camera the pattern and run the intrinsic calibration twice. Once for each camera.

The extrinsic matrix is achieved with the extrinsic calibration. The goal of this calibration is to find the exact relative position and rotation of the 2 cameras. This requires the pattern to be visible for both cameras. Note that only the relative position of the second camera is found. This means that the first camera as considered to be the origin point. You can initiate this as (0,0,0) with no rotation but for example you can also initiate it as the position and rotation compared to the center of the car. The extrinsic matrix is not only useful for calibration... It is also possible to multiply a 3d coordinate from the origin camera system with the extrinsic matrix to convert it to a 3d coordinate relative to the second camera.

Finally there is the projection matrix, which is basically the combination of the intrinsic matrix and the extrinsic matrix. By multiplying a 3D coordinate of an object relative to a camera with a projection matrix, you can directly convert this to a 2D pixel coordinate of the object of the other camera.

To get more detailed information on how OpenCV does this and how to use it, refer to the the OpenCV documentation of calibration.



