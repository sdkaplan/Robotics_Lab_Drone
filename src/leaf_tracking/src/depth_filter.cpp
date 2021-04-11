#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/tracking.hpp>
#include <math.h>
#include <iostream>
#include <ctime>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sys/stat.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include </home/nvidia/Robotics_Lab_experiment/src/leaf_tracking/src/cv-helpers.hpp>
#include </home/nvidia/Robotics_Lab_experiment/src/leaf_tracking/src/rs-processing.hpp>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

/*
void create_mask(int x, int y, int flags, void* userdata )
{
    Mat img = *((Mat *)userdata);

    Mat mask(img.size(),CV_8UC1);
    mask = 0;
    //contours.push_back(pts);
    //drawContours(mask,contours,0,Scalar(255),-1);
    Mat masked(img.size(),CV_8UC3,Scalar(255,255,255));
    src.copyTo(masked,mask);
    src.copyTo(cloneimg);
    imshow( "masked", masked );

    imshow( "Mask", img );
}
*/

int main(int argc, char **argv) try
{
    double scale = 0.5;
    // SET UP REALSENSE
    rs2::colorizer color_map; // Declare depth colorizer for pretty visualization of depth
    rs2::align align_to(RS2_STREAM_COLOR);
    rs2::pipeline pipe; // Declare RealSense pipeline, encapsulating the device and sensors
    // Start streaming with default recommended configuration. The default video configuration contains Depth and Color streams, Gyro and Accelerometer are enabled by default
    pipe.start();

    rs2::decimation_filter dec_filter;
    rs2::temporal_filter temp_filter;

    rs2::spatial_filter spat_filter;
    spat_filter.set_option(RS2_OPTION_HOLES_FILL, 2);

    rs2::spatial_filter spat_filter_alpha;  
    spat_filter_alpha.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 1);

    rs2::threshold_filter thres_filter;
    thres_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.15);
    thres_filter.set_option(RS2_OPTION_MAX_DISTANCE, .5);
  
    rs2::frameset data = pipe.wait_for_frames();    // Wait for next set of frames from the camera
    rs2::frameset aligned_set = align_to.process(data); //create a set of frames that is aligned
    rs2::frame depth = aligned_set.get_depth_frame(); //get the depth frame from the aligned set
    rs2::frame color = aligned_set.get_color_frame(); //get the color frame from the aligned set

    // Query frame size (width and height)
    const int w = depth.as<rs2::video_frame>().get_width();
    const int h = depth.as<rs2::video_frame>().get_height();

    Mat mask;
    Mat masked_color(Size(640, 360), CV_8UC3);

    while (waitKey(1) < 0){
        rs2::frameset data = pipe.wait_for_frames();// Wait for next set of frames
        rs2::frameset aligned_set = align_to.process(data);

        rs2::frame depth_filtered = aligned_set.get_depth_frame();
        rs2::frame depth_raw = aligned_set.get_depth_frame().apply_filter(color_map);
        rs2::frame color_raw = aligned_set.get_color_frame();

        depth_filtered = spat_filter_alpha.process(depth_filtered);
        depth_filtered = spat_filter.process(depth_filtered);    
        depth_filtered = thres_filter.process(depth_filtered);
        depth_filtered = depth_filtered.apply_filter(color_map);

        Mat frame_depth_raw(Size(w, h), CV_8UC3, (void*)depth_raw.get_data(), Mat::AUTO_STEP);
        Mat frame_depth_filtered(Size(w, h), CV_8UC3, (void*)depth_filtered.get_data(), Mat::AUTO_STEP);
        Mat frame_color(Size(w, h), CV_8UC3, (void*)color_raw.get_data(), Mat::AUTO_STEP);

	    // Depth Filtered Video Stream
        resize(frame_depth_filtered, frame_depth_filtered, cvSize(0,0), scale, scale);
        imshow("depth filtered", frame_depth_filtered);

        // Depth Raw Video Stream
        resize(frame_depth_raw, frame_depth_raw, cvSize(0,0), scale, scale);
        imshow("depth raw", frame_depth_raw);
        
        // Color Raw Video Stream
        resize(frame_color, frame_color, cvSize(0,0), scale, scale);
        imshow("color raw", frame_color);

        //create a mask of the color black
        inRange(frame_depth_filtered, Scalar(0, 0, 0), Scalar(0, 0, 0), mask);
        imshow("Mask", mask);

        //apply mask to color image
        masked_color.copyTo(frame_color, mask);
        imshow("masked color", frame_color);

        // Exit if ESC pressed.
        int k = waitKey(1);
        if(k == 27){
            break;
        }

    }

}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

