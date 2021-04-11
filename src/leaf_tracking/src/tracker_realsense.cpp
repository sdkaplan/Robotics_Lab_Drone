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

// CONSTANTS
// Colors for text
const Scalar darkgreen = Scalar(34, 139, 35);
const Scalar red = Scalar(0, 0, 255);
const Scalar white = Scalar(255, 255, 255);
const Scalar blue = Scalar(255, 0, 0);
const Scalar black = Scalar(0, 0, 0);


//GLOBAL VAR DECLARATION
Mat frame_raw;
int non_zero_avg;
int non_zero_sum = 0;
int non_zero_counter = 0;
Rect2d bbox;
bool ok = true;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

/* 
Calculates the position of the tracked object relative to the center of the camera
Returns a geometry_msgs::Point that can be published as a message with ROS
geometry_msgs::Point is a message type embedded in ROS
 */
geometry_msgs::Point calc_pos(Point p_drone, Point p_leaf){
    int x = p_drone.x - p_leaf.x;
    int y = p_drone.y - p_leaf.y;
    geometry_msgs::Point dist;
    dist.x = x;
    dist.y = y;
    return dist;
}

/* 
Adds a highlight behind the text to make it easier to read when displayed on the video
Puts the highlighted text on the frame
 */ 
void highlightText(Mat frame, string text, const int x,const int y, Scalar font_color, Scalar highlight_color,int font, double font_scale, int thickness){

    cv::Size textSize = cv::getTextSize(text, font, font_scale, thickness, 0);
    cv::Point p1(x - 5, y - textSize.height - 5);  // upper left
    cv::Point p2(x + textSize.width + 5, y + 5); // lower right
    cv::Rect rRect(p1, p2);
    rectangle(frame, rRect, highlight_color, -1, 1 );
    putText(frame, text, Point(x, y), font, font_scale, font_color, thickness, 8, false );
}

/* 
Calculates the center point between two given points
Returns type Point which is part of OpenCV
 */
Point calc_center(Point p0, Point p1){
    int center_x = (p0.x + p1.x) /2;
    int center_y = (p0.y + p1.y) /2;
    Point p_center = Point (center_x, center_y);
    return p_center;
}

/* 
Checks the size of the mask, updates the mask average, compares the current mask to the average
Returns true if it is below the target threshold and false if it is above the target threshold
 */
bool check_mask(Mat mask){
    int non_zero = countNonZero(mask);
    non_zero_sum = non_zero_sum + non_zero;
    non_zero_counter = non_zero_counter + 1;
    non_zero_avg = non_zero_sum / non_zero_counter;
    int diff = abs(non_zero_avg - non_zero);
    //cout << SSTR(non_zero_avg) << endl;
    if (diff < (non_zero_avg * 0.5)){
        //cout << " below thres " << SSTR(diff) << endl;
        return true;
    }else{
        cout << " above thres " << SSTR(diff) <<endl;
        return false;    
    }
}

/*
Saves the image to the folder for testing if things work
*/
bool save_image(Mat frame, int counter, string name){
    string path = "/home/nvidia/Documents/Sydney/tracked_image/tracked_image-" + name + SSTR(counter) + ".png";
    imwrite(path, frame);
    return true;
}

/*
Calculates the average distance of a bounding box on a given frame

float avg_dist(Mat frame, Rect2d bbox){
    for (int i = bbox.x; i < bbox.x + bbox.width; i++){
        for (int j = bbox.y; j < bbox.y + bbox.height; j++){
            rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 ); // bbox around tracked object
            //float distance = depth.get_distance(i, j);
            distance = 10.0; 
        }
    }  
    float dist_mm = distance * 1000;
    float width = 5.125*2 * dist_mm / 7.08;
    float mm_per_pix_at_dist = width / frame.size().width;
    return mm_per_pix_at_dist;
}
*/

/*
Checks if the color of a bounding box is entirely black
*/
bool is_not_black(Mat mask, Rect2d bbox){
    if (bbox.x < 0){bbox.x = 0;}
    if (bbox.y < 0){bbox.y = 0;}
    if (bbox.x + bbox.width > mask.cols){bbox.width = mask.cols - bbox.x;}
    if (bbox.y + bbox.height > mask.rows){bbox.height = mask.rows - bbox.y;}
    Mat mask_crop  = mask(bbox);
    //imshow("Cropped",mask_crop);
    int non_zero = countNonZero(mask_crop);
    int size = mask_crop.rows * mask_crop.cols;
    if ((size * 0.9) < non_zero){
        //cout << "BLACK FRAME: size: " << SSTR(size) << "    non-Zero: " << SSTR(non_zero) << endl;
        cout << "TRACKING NOTHING" << endl;
        return false;
    }
    else{
        //cout << "size: " << SSTR(size) << "    non-Zero: " << SSTR(non_zero) << endl;
        return true;
    }
}



int main(int argc, char **argv) try
{
    // SET UP ROS NODE FOR LOCATION PUBLISHING
    ros::init(argc, argv, "publish_location");
    ros::NodeHandle n;
    ros::Publisher pos_pub = n.advertise<geometry_msgs::Point>("location", 1000);

    // SET UP ROS NODE FOR IMAGE PUBLISHING
    ros::init(argc, argv, "camera");
    ros::NodeHandle n1;
    image_transport::ImageTransport it_(n1);
    image_transport::Publisher image_pub_ = it_.advertise("/camera/image_tracked", 1);
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    // SET UP ROS NODE FOR IMAGE PUBLISHING
    ros::init(argc, argv, "camera_raw1");
    ros::NodeHandle n2;
    image_transport::ImageTransport it1_(n2);
    image_transport::Publisher image_pub1_ = it1_.advertise("/camera/raw1", 1);
    cv_bridge::CvImagePtr cv_ptr1(new cv_bridge::CvImage);
    
    // Define variables used
    double scale = 0.35;
    int total_fps = 0;
    int counter = 0;

    // Text Features
    int font = FONT_HERSHEY_SIMPLEX;
    double font_scale = .7;
    int thickness = 2;
    Size textSize = cv::getTextSize("CSRT Tracker", font, font_scale, thickness, 0);
    int text1_x = 40; 
    int text1_y = textSize.height + 10;
    int spacing = textSize.height + 10; //spacing for text on the screen in the y direction

    // Colors for text
    //Scalar darkgreen = Scalar(34, 139, 35);
    //Scalar red = Scalar(0, 0, 255);
    //Scalar white = Scalar(255, 255, 255);
    //Scalar blue = Scalar(255, 0, 0);
    //Scalar black = Scalar(0, 0, 0);

    // Create a tracker
    string trackerType = "CSRT";
    Ptr<Tracker> tracker;
    tracker = TrackerCSRT::create();

    // SET UP REALSENSE
    rs2::colorizer color_map; // Declare depth colorizer for pretty visualization of depth
    rs2::align align_to(RS2_STREAM_COLOR);
    rs2::pipeline pipe; // Declare RealSense pipeline, encapsulating the device and sensors
    // Start streaming with default recommended configuration. The default video configuration contains Depth and Color streams, Gyro and Accelerometer are enabled by default
    pipe.start();

    // SET UP FILTERS
    rs2::decimation_filter dec_filter;
    rs2::temporal_filter temp_filter;
    rs2::spatial_filter spat_filter;
    rs2::threshold_filter thres_filter;

    spat_filter.set_option(RS2_OPTION_HOLES_FILL, 2);
    rs2::spatial_filter spat_filter_alpha;  
    spat_filter_alpha.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 1);

    rs2::frameset data = pipe.wait_for_frames();                // Wait for next set of frames from the camera
    rs2::frameset aligned_set = align_to.process(data);         //create a set of frames that is aligned
    rs2::frame depth_filtered = aligned_set.get_depth_frame();  //get the depth frame from the aligned set
    rs2::frame color_raw = aligned_set.get_color_frame();       //get the color frame from the aligned set

    // Query frame size (width and height)
    const int w = depth_filtered.as<rs2::video_frame>().get_width();
    const int h = depth_filtered.as<rs2::video_frame>().get_height();


    // Read first X frames of video
    int end = 50;
    for (int i=0; i < end; i++){
        rs2::frameset data = pipe.wait_for_frames();            // Wait for next set of frames
        rs2::frameset aligned_set = align_to.process(data);     //align the frames
        rs2::frame color_raw = aligned_set.get_color_frame();   //get aligned color frame

        Mat frame_rawBGR(Size(w, h), CV_8UC3, (void*)color_raw.get_data(), Mat::AUTO_STEP);
        cv::cvtColor(frame_rawBGR, frame_raw, cv::COLOR_BGR2RGB);
        cout << "looping... " << SSTR(i) << endl;               // prints Output sentence on screen
    }
    
    /*
    Render the ROI smaller. When doing field tests with ssh it can take a long time for the frame to render on the screen
    the following code resizes the roi so it can renders faster. The user selects the roi on the smaller frame and then
    the bbox must be rescaled by the same value. 
    */
    Mat frame_raw_smaller;   
    int roi_scale = 2;
    resize(frame_raw, frame_raw_smaller, Size(frame_raw.cols/roi_scale, frame_raw.rows/roi_scale));
    Rect2d bbox = selectROI(frame_raw_smaller, false);
    bbox.x = bbox.x * roi_scale;            // resize the x value of the bbox
    bbox.y = bbox.y * roi_scale;            // resize the y value of the bbox
    bbox.width = bbox.width * roi_scale;    // resize the width value of the bbox
    bbox.height = bbox.height * roi_scale;  // resize the hegiht value of the bbox

    // Display bounding box with a dot.
    rectangle(frame_raw, bbox, blue, 2, 1 );
    Point point0 = Point( bbox.x, bbox.y);                           // top left corner of bbox
    Point point1 = Point(bbox.x + bbox.width, bbox.y + bbox.height); //bottom left corner of bbox
    Point centerBBOX = calc_center(point0, point1);                  //center of bbox

    tracker->init(frame_raw, bbox);     // start the tracker

    float min_dist = 0.05;
    float max_dist = 3.5;
    thres_filter.set_option(RS2_OPTION_MIN_DISTANCE, min_dist);
    thres_filter.set_option(RS2_OPTION_MAX_DISTANCE, max_dist);

    //apply filters and colorize the depth data
    depth_filtered = spat_filter_alpha.process(depth_filtered);
    depth_filtered = spat_filter.process(depth_filtered);    
    depth_filtered = depth_filtered.apply_filter(color_map);
    depth_filtered = thres_filter.process(depth_filtered);
    Mat frame_depth_filtered(Size(w, h), CV_8UC3, (void*)depth_filtered.get_data(), Mat::AUTO_STEP);

    //empty mats for the masks 
    Mat mask;
    Mat masked_color(Size(w, h), CV_8UC3);

    // Define points on the screen that are used later on
    Point p0(0,0);						                            //origin
    Point p1(frame_raw.size().width, frame_raw.size().height);      //bottom right corner
    Point p3(frame_raw.size().width/2, 0);    			            //top center
    Point p4(frame_raw.size().width, frame_raw.size().height/2);    //left center
    Point p5(frame_raw.size().width/2, frame_raw.size().height);    //bottom center
    Point p6(0, frame_raw.size().height/2);                         //right center
    textSize = cv::getTextSize("+Y", font, font_scale, thickness, 0);
    Point p7(20, frame_raw.size().height/2 - 20);                                   //+X
    Point p8(frame_raw.size().width/2 - textSize.width - 20, textSize.height + 20); //+Y
    Point const centerFrame = calc_center(p0, p1);


    while (waitKey(1) < 0){
        //get frames from realsense and set them up
        rs2::frameset data = pipe.wait_for_frames();                // Wait for next set of frames
        rs2::frameset aligned_set = align_to.process(data);         //align the frames
        rs2::frame depth_filtered = aligned_set.get_depth_frame();  //get aligned depth frame
        rs2::frame depth1         = aligned_set.get_depth_frame();  //get aligned depth frame
        rs2::depth_frame depth    = aligned_set.get_depth_frame();  //get depth frame
        rs2::frame color_raw      = aligned_set.get_color_frame();  //get aligned color frame
        
        depth1 = spat_filter_alpha.process(depth1);
        depth1 = spat_filter.process(depth1); 
        depth1 = depth1.apply_filter(color_map);
        Mat frame_depth(Size(w, h), CV_8UC3, (void*)depth1.get_data(), Mat::AUTO_STEP);
        //save_image(frame_depth, counter, "depth");

        // Apply filters and colorize the depth image
        depth_filtered = spat_filter_alpha.process(depth_filtered);
        depth_filtered = spat_filter.process(depth_filtered);    
        depth_filtered = thres_filter.process(depth_filtered);
        depth_filtered = depth_filtered.apply_filter(color_map);

        //convert to opencv Mat
        Mat frame_depth_filtered(Size(w, h), CV_8UC3, (void*)depth_filtered.get_data(), Mat::AUTO_STEP);
    
        Mat frameBGR(Size(w, h), CV_8UC3, (void*)color_raw.get_data(), Mat::AUTO_STEP);    
        Mat frame;
        cv::cvtColor(frameBGR, frame, cv::COLOR_BGR2RGB);
    
        //create a mask on everything that is black, check size of mask
        inRange(frame_depth_filtered, Scalar(0, 0, 0), Scalar(0, 0, 0), mask);
       
        //bool good_frame = check_mask(mask);
        bool good_frame = is_not_black(mask, bbox);
        masked_color.copyTo(frame, mask); //apply the mask to the frame

        //grab the frame and color it RGB
        Mat frame_rawBGR(Size(w, h), CV_8UC3, (void*)color_raw.get_data(), Mat::AUTO_STEP);
        cv::cvtColor(frame_rawBGR, frame_raw, cv::COLOR_BGR2RGB);

        //publish the raw frame 
        cv_ptr1->image = frame;
        cv_ptr1->encoding = "bgr8";
        image_pub1_.publish(cv_ptr1->toImageMsg());

        // Start timer
        double timer = (double)getTickCount();
        
        if(good_frame){
            // Update the tracking result
            bool ok = tracker->update(frame, bbox);
        }

        // Draw coordinate system
        line(frame, p3, p5, white, 5, 1, 0);
        line(frame, p4, p6, white, 5, 1, 0);
        highlightText(frame,"+X",p7.x, p7.y, white, black, font, font_scale, thickness);
        highlightText(frame,"+Y",p8.x, p8.y, white,black, font, font_scale, thickness);

        if (ok && good_frame){    // Tracking success
            //defining points on the bbox
            Point point0 = Point(bbox.x, bbox.y);                               // top left corner of bbox
            Point point1 = Point(bbox.x + bbox.width, bbox.y + bbox.height);    //bottom right corner of bbox
            Point centerBBOX = calc_center(point0, point1);                     //center of bbox
            Point centerBBOX_2(-(centerBBOX.x - frame.size().width/2), -(centerBBOX.y - frame.size().height/2)); //center of bbox adjusted with Origin at center of frame

            // pix to mm at given depth (dist)
            rectangle(frame_depth_filtered, bbox, Scalar( 255, 0, 0 ), 2, 1 ); // bbox around tracked object
            float distance = depth.get_distance(centerBBOX.x, centerBBOX.y);
            float dist_mm = distance * 1000;
            float width = 5.125*2 * dist_mm / 7.08;
            float mm_per_pix_at_dist = width / frame.size().width;
            Point centerBBOX_2_mm(-(centerBBOX_2.x * mm_per_pix_at_dist), (centerBBOX_2.y * mm_per_pix_at_dist)); //center of bbox adjusted with Origin at center of frame

            //draw tracked object
            rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );     // bbox around tracked object
            line(frame, centerBBOX, centerFrame, red, 5, 1, 0);     // line from center frame to center bbox
            circle(frame, centerFrame, 10, red, -1, 1, 0);          // red dot at center of frame
            circle(frame, centerBBOX, 10, blue, -1, 1, 0);          // blue dot in center of bbox
            highlightText(frame, "X: " + SSTR(centerBBOX_2.y * mm_per_pix_at_dist) + "mm", centerBBOX.x + 20, centerBBOX.y - 90, white, blue, font, font_scale*.5, thickness*0.5);
            highlightText(frame, "Y: " + SSTR(centerBBOX_2.x * mm_per_pix_at_dist) + "mm", centerBBOX.x + 20, centerBBOX.y - 65, white, blue, font, font_scale*.5, thickness*0.5);
            highlightText(frame, "Z: " + SSTR(dist_mm) + "mm", centerBBOX.x + 20, centerBBOX.y - 40, white, blue, font, font_scale*.5, thickness*0.5);

            //ROS: read the position change needed and publish it
            geometry_msgs::Point msg = calc_pos(centerFrame, centerBBOX);
            pos_pub.publish(msg);
            cout << "Tracking" << endl;


        } else if (!good_frame){
            rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );     // bbox around tracked object
            highlightText(frame,"Tracker Skipping Frame", text1_x, text1_y + spacing, white, red, font, font_scale, thickness);
            ROS_ERROR("WARNING: Tracking Nothing!!");
        } else {
            // Tracking failure detected.
            highlightText(frame,"Tracking failure detected", text1_x, text1_y + spacing, white, red, font, font_scale, thickness);

	        // ROS: Publish unrealistic number
            Point fail_box(100000, 100000);
	        geometry_msgs::Point msg = calc_pos(centerFrame, fail_box);
            pos_pub.publish(msg);
            ROS_ERROR("WARNING: Tracking Failure!!");
        }
        

        // Display trackerEXIT_FAILURE type on frame
        highlightText(frame,trackerType + " Tracker",text1_x, text1_y, white, darkgreen, font, font_scale, thickness);
        
        // time for saving video with time in the name
       // time_t now = time(0);
        //tm *ltm = localtime(&now);
        //string timer1 = SSTR(1 + ltm->tm_min) + SSTR(1 + ltm->tm_sec);

	    // ROS: Publish tracked frame for saving to video
        imshow("tracking", frame);
        cv_ptr->image = frame;
        cv_ptr->encoding = "bgr8";
        image_pub_.publish(cv_ptr->toImageMsg());

        // Exit if ESC pressed.
        int k = waitKey(1);
        if(k == 27){
            break;
        }
        counter = counter + 1;

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

