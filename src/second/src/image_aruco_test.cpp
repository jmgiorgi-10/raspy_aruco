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

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub;
  ros::NodeHandle n2;
  ros::Publisher pub;
  Mat cameraMatrix;
  Mat distCoeffs;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &ImageConverter::imageCb, this);
    pub = n2.advertise<std_msgs::Float32MultiArray>("array", 1);

    image_pub = it_.advertise("/aruco_camera", 1);

    cameraMatrix = (Mat1d(3,3) << 1.1131964048572934e+03, 0, 6.4918146753565634e+02, 0, 1.1234296429352053e+03, 2.6055413683654331e+02, 0, 0, 1);
    distCoeffs = (Mat1d(1,5) <<  2.4936964447069973e-01, -7.0650882086700950e-01, -4.6592829034340401e-02, 3.6756516954705444e-04,  9.5260003500440427e-01);


  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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

    Mat image = cv_ptr->image;
    vector<int> markerIds;
    vector< vector<Point2f> > markerCorners;
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
    aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
    vector<Vec3d> rvecs, tvecs;
    aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

    aruco::drawDetectedMarkers(cv_ptr->image, markerCorners, markerIds);
    image_pub.publish(cv_ptr->toImageMsg());

    std_msgs::Float32MultiArray array;

    if (tvecs.size() >= 1) {
      for (int i = 0; i < 3; i++) {
        //array.data.push_back(tvecs.at(0)[i]);
        array.data.push_back((tvecs.at(0))[i]);
        ///array.data.push_back(1);
      }
    }
    pub.publish(array);
    
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::Rate r(20); // 1 Hertz for easy testing.
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  //ros::spin();
  return 0;
}
