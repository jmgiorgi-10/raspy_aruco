#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <vector>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

  
  // Subsicriber callback function.
  void imageCb(const sensor_msgs::ImageConstPtr& msg, ros::Publisher& pub) {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // ARUCO IMPLEMENTATION //

    // Defining camera values
    float data0[12] = {476.7030836014194, 0.0, 400.5, -0.0, 0.0, 476.7030836014194, 400.5, 0.0, 0.0, 0.0, 1.0, 0.0};
    float data1[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    Mat cameraMatrix(1, 12, CV_32F, data0);
    Mat distCoeffs(1, 5, CV_32F, data1);

    Mat image = cv_ptr->image;
    vector<int> markerIds;
    vector< vector<Point2f> > markerCorners;
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
    aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
    vector<Vec3d> rvecs, tvecs;
    aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

    std_msgs::Float32MultiArray array;
    if (tvecs.size() >= 1) {
      for (int i = 0; i < 3; i++) {
        array.data.push_back(tvecs.at(0)[i]);
      }
    }
    pub.publish(array);
  }

int main(int argc, char** argv)
{
  ros::Publisher t_pub;

  ros::init(argc, argv, "image_converter");
  ros::NodeHandle n1;
  image_transport::ImageTransport it(n1);
  ros::NodeHandle n2;
  t_pub = n2.advertise<std_msgs::Float32MultiArray>("array", 100);
  //pub_ptr = &t_pub;
  // Subscribe to input video feed and publish output video feed
  image_transport::Subscriber image_sub_ = it.subscribe("iris/camera_red_iris/image_raw", 1, boost::bind(imageCb, _1, boost::ref(ros::Publisher)));

  //ImageConverter ic(n);
  //t_pub.publish(array);
  ros::spin();
  return 0;
}