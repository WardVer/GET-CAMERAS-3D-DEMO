#define GPU
#define OPENCV

#include "yolo_v2_class.hpp"
#include "3d_functions.hpp"
#include "cameras.hpp"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "image_detection/image_location.h"
#include "image_detection/Cone.h"
#include "image_detection/ConeArray.h"
//#include "pathplanner/path.h"

//#include "pathplanner/path.h"

#include <queue>
#include <thread>

#include "GxIAPI.h"
#include "DxImageProc.h"

#define FLIP 1

#define FRAMEHEIGHT 832
#define FRAMEWIDTH 832

#define EXPOSURE_CHANGE_RATE 0

#define DRAW 1

#define names_file "data/all_cones.names"
#define cfg_file "cfg/all_cones.cfg"
#define weights_file "weights/all_cones_final.weights"

#define YOLO_THRESHOLD 0.2f

#define FPS 10

#define MAX_CONE_DIST 6.f

#define REAR_AXLE_TO_LEFTCAMERA_FORWARD_DIST 0.5f
#define REAR_AXLE_TO_LEFTCAMERA_SIDE_DIST 0.2f

std::atomic<int> fps_cap_counter(0), fps_det_counter(0);
std::atomic<int> current_fps_cap(0), current_fps_det(0);
std::atomic<bool> exit_flag(false);
std::chrono::steady_clock::time_point steady_start, steady_end;
float exposure = 2000;
bool sendpause = false;

struct detection_data_t
{
    Mat cap_frame1;
    Mat cap_frame2;
    std::shared_ptr<image_t> det_image;
    std::vector<bbox_t> result_vec1;
    std::vector<bbox_t> result_vec2;
    Mat draw_frame;
    Mat rough_3d;
    std::vector<Point2f> offsets;
    std::vector<Point3f> points3D;
    bool new_detection;
    uint64_t timestamp;
    bool exit_flag;
    detection_data_t() : exit_flag(false), new_detection(false) {}
};

void draw_boxes(Mat mat_img, std::vector<bbox_t> result_vec)
{
    for (auto &i : result_vec)
    {
        Scalar color;
        if (i.obj_id == 0)
            color = Scalar(0, 255, 255);
        if (i.obj_id == 1)
            color = Scalar(255, 100, 0);
        if (i.obj_id == 2)
            color = Scalar(0, 100, 255);
        if (i.obj_id == 3)
            color = Scalar(0, 0, 255);

        if (i.x > 0 && i.y > 0 && i.w > 0 && i.h > 0 && i.x + i.w < mat_img.cols && i.y + i.h < mat_img.rows)
        {
            rectangle(mat_img, Rect(i.x, i.y, i.w, i.h), color, 2);
        }
    }
}

template <typename T>
class send_one_replaceable_object_t
{
    std::atomic<T *> a_ptr;

public:
    void send(T const &_obj)
    {
        T *new_ptr = new T;
        *new_ptr = _obj;
        std::unique_ptr<T> old_ptr(a_ptr.exchange(new_ptr));
    }

    T receive()
    {
        std::unique_ptr<T> ptr;
        do
        {
            while (!a_ptr.load())
                std::this_thread::sleep_for(std::chrono::microseconds(200));
            ptr.reset(a_ptr.exchange(NULL));
        } while (!ptr);
        T obj = *ptr;
        return obj;
    }

    bool is_object_present()
    {
        return (a_ptr.load() != NULL);
    }

    send_one_replaceable_object_t() : a_ptr(NULL)
    {
    }
};

send_one_replaceable_object_t<detection_data_t> cap2prepare, cap2draw, prepare2detect, draw2show, draw2write, draw2net, detect2threed, threed2draw;

static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM *pFrame)
{

    uint64_t timestamp = 0;
    detection_data_t detection_data;

    std::chrono::steady_clock::time_point capture_begin;
    std::chrono::steady_clock::time_point capture_end;

    Mat image;
    cuda::GpuMat frame;
    cuda::GpuMat gray;
    gray.convertTo(gray, CV_8UC1);

    Ptr<cuda::Filter> filter = cuda::createGaussianFilter(gray.type(), gray.type(), Size(21, 21), 4);

    if (pFrame->status == 0)
    {
        capture_begin = std::chrono::steady_clock::now();

        //read in frame to opencv
        image.create(pFrame->nHeight, pFrame->nWidth, CV_8UC1);
        image.data = (uchar *)pFrame->pImgBuf;
        timestamp = pFrame->nTimestamp;

        //load frame in gpu

        //set the correct image format and size
        cv::cvtColor(image, image, COLOR_BayerRG2RGB);
        

        if (FLIP == 1)
        {
            cv::flip(image, image, -1);
        }

        frame.upload(image);

        

        cuda::resize(frame, frame, Size(2 * FRAMEWIDTH, FRAMEHEIGHT));

        //make a grayscale image to check brightness value

        cuda::cvtColor(frame, gray, COLOR_RGB2GRAY);
        filter->apply(gray, gray);
        //Mat cpugray(gray);
        //imshow("gray", cpugray);

        double minval;
        double maxval;
        Point minloc;
        Point maxloc;

        Scalar mean;
        Scalar stdv;
        cuda::minMaxLoc(gray, &minval, &maxval, &minloc, &maxloc);
        Mat Mask = (Mat(gray) == 255);
        float whitesize = countNonZero(Mask);
        float pixelcount = FRAMEWIDTH * FRAMEHEIGHT;
        float whitepart = whitesize / pixelcount;
        float exposurerate = sqrt(sin(whitepart * M_PI / 2));
        //std::cout << whitepart << std::endl;
        //std::cout << exposurerate << std::endl
        //          << std::endl;

        if (whitepart >= 0.01)
        {
            exposure -= EXPOSURE_CHANGE_RATE * exposurerate;
        }
        else
        {
            exposure += EXPOSURE_CHANGE_RATE / 10;
        }

        std::vector<cuda::GpuMat> channels;

        //do histogram EQ
        cuda::cvtColor(frame, frame, COLOR_RGB2HSV);
        cuda::split(frame, channels);
        cuda::equalizeHist(channels[2], channels[2]);
        cuda::merge(channels, frame);
        cuda::cvtColor(frame, frame, COLOR_HSV2RGB);

        //load image from GPU and put in main structure
        Mat cpuframe(frame);
        detection_data.cap_frame1 = cpuframe(Rect(0, 0, FRAMEWIDTH, FRAMEHEIGHT));
        detection_data.cap_frame2 = cpuframe(Rect(FRAMEWIDTH, 0, FRAMEWIDTH, FRAMEHEIGHT));
        detection_data.timestamp = timestamp;


        /*imshow("yoink1", detection_data.cap_frame1);
        imshow("yoink2", detection_data.cap_frame2);
        waitKey(0);*/

        //send to other threads
        cap2prepare.send(detection_data);
        if (DRAW == 1)
            cap2draw.send(detection_data);

        fps_cap_counter++;

        capture_end = std::chrono::steady_clock::now();
        //std::cout << "capture time = " << std::chrono::duration_cast<std::chrono::microseconds>(capture_end - capture_begin).count() << "[µs]" << std::endl;
    }
    return;
}

int main(int argc, char *argv[])
{

    Detector detector(cfg_file, weights_file);
    std::thread t_cap, t_trigger, t_process, t_prepare, t_detect, t_3d, t_post, t_draw, t_write, t_network;

    Stereocamera cam(1, 16, 5000, 1.5, 1, 1.5, GX_TRIGGER_MODE_ON);
    cam.setCallback(OnFrameCallbackFun);
    cam.startCamera();

    while (ros::ok)
    {

        try
        {

            // pre-processing video frame (resize, convertion)
            t_prepare = std::thread([&]() {
                std::shared_ptr<image_t> det_image;
                detection_data_t detection_data;
                std::chrono::steady_clock::time_point begin;
                std::chrono::steady_clock::time_point end;
                do
                {
                    detection_data = cap2prepare.receive();
                    begin = std::chrono::steady_clock::now();
                    det_image = detector.mat_to_image_resize(detection_data.cap_frame1);
                    detection_data.det_image = det_image;
                    prepare2detect.send(detection_data); // detection
                    end = std::chrono::steady_clock::now();
                    //std::cout << "process time = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

                } while (!detection_data.exit_flag);
                //std::cout << " t_prepare exit \n";
            });

            t_trigger = std::thread([&]() {
                while (1)
                {
                    cam.trigger();
                    cam.setExposure(exposure);
                    std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000 / FPS)));
                }
            });

            // detection by Yolo
            if (t_detect.joinable())
                t_detect.join();
            t_detect = std::thread([&]() {
                std::chrono::steady_clock::time_point begin;
                std::chrono::steady_clock::time_point end;
                std::shared_ptr<image_t> det_image;
                detection_data_t detection_data;

                do
                {
                    detection_data = prepare2detect.receive();

                    begin = std::chrono::steady_clock::now();
                    det_image = detection_data.det_image;
                    std::vector<bbox_t> result_vec;

                    if (det_image)
                        result_vec = detector.detect_resized(*det_image, FRAMEWIDTH, FRAMEHEIGHT, YOLO_THRESHOLD, true); // true
                    fps_det_counter++;

                    detection_data.new_detection = true;
                    detection_data.result_vec1 = result_vec;
                    detect2threed.send(detection_data);
                    end = std::chrono::steady_clock::now();
                    //std::cout << "detect time = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

                } while (!detection_data.exit_flag);
                //std::cout << " t_detect exit \n";
            });

            // draw rectangles (and track objects)
            t_draw = std::thread([&]() {
                detection_data_t detection_data;
                std::chrono::steady_clock::time_point begin;
                std::chrono::steady_clock::time_point end;
                do
                {

                    // get new Detection result if present
                    if (!threed2draw.is_object_present())
                    { // use old captured frame
                        detection_data = threed2draw.receive();
                        begin = std::chrono::steady_clock::now();
                    }
                    // get new Captured frame
                    else
                    {
                        detection_data_t old_detection_data = detection_data;
                        detection_data = cap2draw.receive();
                        begin = std::chrono::steady_clock::now();
                        Mat draw_frame1 = detection_data.cap_frame1;
                        detection_data.result_vec1 = old_detection_data.result_vec1;
                    }

                    Mat draw_frame1 = detection_data.cap_frame1.clone();
                    std::vector<bbox_t> result_vec1 = detection_data.result_vec1;

                    draw_boxes(draw_frame1, result_vec1);

                    detection_data.draw_frame = draw_frame1;
                    draw2show.send(detection_data);
                    end = std::chrono::steady_clock::now();
                    //std::cout << "draw time = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

                } while (!detection_data.exit_flag);
                //std::cout << " t_draw exit \n";
            });

            //estimate 3D

            t_3d = std::thread([&]() {
                ros::init(argc, argv, "cone_detector");
                ros::NodeHandle n_rviz;
                ros::NodeHandle n_path;
                ros::NodeHandle n_slam;
                ros::Publisher marker_pub = n_rviz.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
                ros::Publisher path_pub = n_path.advertise<image_detection::image_location>("image_location", 1);
                ros::Publisher slam_pub = n_slam.advertise<image_detection::ConeArray>("ConeArray", 1);

                uint32_t shape = visualization_msgs::Marker::MESH_RESOURCE;

                detection_data_t detection_data;
                std::chrono::steady_clock::time_point begin;
                std::chrono::steady_clock::time_point end;

                projector3D projector(Size(9, 6), 3.78f);

                int h = 0;

                do
                {

                    detection_data = detect2threed.receive();

                    begin = std::chrono::steady_clock::now();

                    if (detection_data.result_vec1.size() == 0)
                        continue;
                    std::vector<box_3D> stereo_boxes;
                    std::vector<box_3D *> valid_boxes;
                    std::vector<box_3D *> valid_boxes_filtered;

                    for (bbox_t tempbox : detection_data.result_vec1)
                    {
                        box_3D stereo_box;
                        stereo_box.x1 = tempbox.x;
                        stereo_box.y1 = tempbox.y;
                        stereo_box.w1 = tempbox.w;
                        stereo_box.h1 = tempbox.h;
                        stereo_box.obj_id = tempbox.obj_id;
                        stereo_box.prob = tempbox.prob;
                        stereo_boxes.push_back(stereo_box);
                    }

                    valid_boxes = projector.stereo3D(detection_data.cap_frame1, detection_data.cap_frame2, &stereo_boxes);

                    if (sendpause == true)
                    {
                        valid_boxes.clear();
                    }

                    for(box_3D * boxref : valid_boxes)
                    {
                        //std::cout << norm(cv::Point2f(boxref->x_3d, boxref->z_3d)) << std::endl;
                        if(norm(cv::Point2f(boxref->x_3d, boxref->z_3d)) < MAX_CONE_DIST*100)
                        {
                            valid_boxes_filtered.push_back(boxref);
                        }
                    }

                    if (DRAW == 1)
                        threed2draw.send(detection_data);
                    end = std::chrono::steady_clock::now();
                    //std::cout << "3D time = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

                    image_detection::ConeArray conearr;

                    for (int a = 0; a < valid_boxes_filtered.size(); a++)
                    {

                        valid_boxes_filtered[a]->x_3d -= 100 * REAR_AXLE_TO_LEFTCAMERA_SIDE_DIST;
                        valid_boxes_filtered[a]->z_3d += 100 * REAR_AXLE_TO_LEFTCAMERA_FORWARD_DIST;

                        image_detection::Cone cone;
                        cone.x = valid_boxes_filtered[a]->x_3d * 10;
                        cone.y = valid_boxes_filtered[a]->z_3d * 10;
                        cone.id = valid_boxes_filtered[a]->obj_id;
                        conearr.cones.push_back(cone);
                    }

                    slam_pub.publish(conearr);
                    visualization_msgs::MarkerArray markers;
                    visualization_msgs::Marker marker;

                    marker.header.frame_id = "/my_frame";
                    marker.ns = "cones";
                    marker.action = visualization_msgs::Marker::DELETE;

                    for (int m = 0; m < h; m++)
                    {
                        marker.id = m;
                        markers.markers.push_back(marker);
                    }

                    marker_pub.publish(markers);

                    markers.markers.clear();

                    h = 0;

                    marker.id = h;
                    marker.type = shape;
                    marker.ns = "car";
                    marker.mesh_resource = "package://image_detection/car.stl";
                    marker.action = visualization_msgs::Marker::ADD;

                    marker.pose.position.x = 0;
                    marker.pose.position.y = 0;
                    marker.pose.position.z = 0;
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.707;
                    marker.pose.orientation.w = 0.707;

                    marker.scale.x = 1;
                    marker.scale.y = 1;
                    marker.scale.z = 1;

                    marker.color.r = 0.2f;
                    marker.color.g = 1.0f;
                    marker.color.b = 0.4f;
                    marker.color.a = 1.0;

                    //markers.markers.push_back(marker);

                    h++;
                    //marker.type = visualization_msgs::Marker::CYLINDER;
                    marker.header.stamp = ros::Time::now();
                    marker.mesh_resource = "package://image_detection/cone2.dae";

                    marker.pose.position.z = 0;
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;

                    marker.scale.x = 0.9;
                    marker.scale.y = 0.9;
                    marker.scale.z = 0.9;

                    marker.ns = "cones";
                    marker.header.frame_id = "/tftransform";

                    for (box_3D *showcone : valid_boxes_filtered)
                    {

                        marker.id = h;
                        marker.scale.z = 0.9;
                        marker.pose.position.x = showcone->z_3d / 100;
                        marker.pose.position.y = -showcone->x_3d / 100;

                        if (showcone->obj_id == 3)
                            marker.scale.z += 0.6;

                        if (showcone->obj_id == 0)
                        {
                            marker.color.r = 2.f;
                            marker.color.g = 2.f;
                            marker.color.b = 0.f;
                            marker.color.a = 1.0;
                        }
                        if (showcone->obj_id == 1)
                        {
                            marker.color.r = 0.2f;
                            marker.color.g = 0.6f;
                            marker.color.b = 2.f;
                            marker.color.a = 1.0;
                        }
                        if (showcone->obj_id == 2)
                        {
                            marker.color.r = 3.f;
                            marker.color.g = 1.f;
                            marker.color.b = 0.f;
                            marker.color.a = 1.0;
                        }
                        if (showcone->obj_id == 3)
                        {
                            marker.color.r = 3.f;
                            marker.color.g = 1.f;
                            marker.color.b = 0.f;
                            marker.color.a = 1.0;
                        }

                        markers.markers.push_back(marker);
                        h++;
                    }
                    marker_pub.publish(markers);

                } while (!detection_data.exit_flag);
                //std::cout << " t_draw exit \n";
            });

            // show detection

            startWindowThread();
            std::chrono::steady_clock::time_point begin;
            std::chrono::steady_clock::time_point end;

            detection_data_t detection_data;

            do
            {
                detection_data = draw2show.receive();

                begin = std::chrono::steady_clock::now();

                Mat draw_frame;
                resize(detection_data.draw_frame, draw_frame, Size(832, 832));

                begin = std::chrono::steady_clock::now();

                imshow("overview", draw_frame);
                int key = waitKey(30); // 3 or 16ms
                if (key == 'q')
                    while (true)
                        if (waitKey(100) == 'p')
                            break;
                //if (key == 'e') extrapolate_flag = !extrapolate_flag;
                if (key == 27)
                {
                    exit_flag = true;
                }
                if (key == 112)
                {
                    if (sendpause == false)
                    {
                        sendpause = true;
                    }
                    else
                    {
                        sendpause = false;
                    }
                }

                end = std::chrono::steady_clock::now();
                //std::cout << "show time = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

                //std::cout << " current_fps_det = " << current_fps_det << ", current_fps_cap = " << current_fps_cap << std::endl;
            } while (!detection_data.exit_flag);
            //std::cout << " show detection exit \n";

            destroyWindow("window name");
            // wait for all threads
            if (t_cap.joinable())
                t_cap.join();
            if (t_3d.joinable())
                t_3d.join();
            if (t_prepare.joinable())
                t_prepare.join();
            if (t_detect.joinable())
                t_detect.join();
            if (t_post.joinable())
                t_post.join();
            if (t_draw.joinable())
                t_draw.join();
            if (t_write.joinable())
                t_write.join();
            if (t_network.joinable())
                t_network.join();

            break;
        }

        catch (std::exception &e)
        {
            std::cerr << "exception: " << e.what() << "\n";
            getchar();
        }
        catch (...)
        {
            std::cerr << "unknown exception \n";
            getchar();
        }
    }

    return 0;
}