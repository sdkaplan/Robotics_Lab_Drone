#include <ros/ros.h>
#include "std_msgs/String.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv2/opencv.hpp"
#include <iostream>
#include <typeinfo>
#include <sys/stat.h>
#include <ctime>
//#include <boost/filesystem.hpp>

using namespace std;
using namespace cv;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

int counter {0};
string folder;
cv::VideoWriter video_saver;
//boost::filesystem::path path_target;
string timer;

static const std::string OPENCV_WINDOW = "Tracking2";

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // convert subscribed frame ROS image to opencv Mat
    Mat frame = cv_ptr->image;

    // write subscribed frame to a video
    //imshow(OPENCV_WINDOW, frame);

    if (counter == 0){
      // time for saving video with time in name the
      time_t now = time(0);
      tm *ltm = localtime(&now);
      string timer = SSTR(1 + ltm->tm_mon) + "-" + SSTR(ltm->tm_mday) + "-" + SSTR(1900 + ltm->tm_year) + "-" + SSTR(ltm->tm_hour) + ":" + SSTR(1 + ltm->tm_min);

      string folder = "/home/nvidia/Documents/Sydney/tracked_image/" + timer;
      cout << folder.c_str() << endl;
      int dir = mkdir(folder.c_str(), 0777);
      
      int fcc = VideoWriter::fourcc('X','V','I','D');
      video_saver.open("/home/nvidia/Documents/Sydney/tracker-" + timer + ".MP4",fcc,8, Size(frame.size().width,frame.size().height));
    }

    video_saver.write(frame);
    time_t now = time(0);
    tm *ltm = localtime(&now);
    string timer = SSTR(1 + ltm->tm_mon) + "-" + SSTR(ltm->tm_mday) + "-" + SSTR(1900 + ltm->tm_year) + "-" + SSTR(ltm->tm_hour) + ":" + SSTR(1 + ltm->tm_min);
    string path = "/home/nvidia/Documents/Sydney/tracked_image/tracked_image"+ SSTR(1 + ltm->tm_mon) + "-" + SSTR(ltm->tm_mday) + "-" + SSTR(1900 + ltm->tm_year) + "-" + SSTR(ltm->tm_hour) + ":" + SSTR(1 + ltm->tm_min)+ "_" + SSTR(counter) + ".png";
    imwrite(path, frame);
    counter = counter + 1;
    waitKey(1);

}

int main(int argc, char **argv){
  ros::init(argc, argv, "reading_image");
  ros::NodeHandle n;
  image_transport::Subscriber image_sub_;
  image_transport::ImageTransport it_(n);
  image_sub_ = it_.subscribe("/camera/image_tracked", 1, imageCallback);
  ros::spin();

}
