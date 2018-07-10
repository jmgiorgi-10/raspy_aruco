// Version with Calibration Data for actual WebCam.

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <vector>

using namespace cv;
using namespace std;
static const std::string OPENCV_WINDOW = "Image window";

// B-Frame == Base Frame (Quad).
// G-Frame == Ground Frame.

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
  ros::Time last_request;
  Mat rot_mat;
  Mat rot_mat_yaw;
  Mat T;
  Mat RT;
  Mat X;
  Mat u;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    pub = n2.advertise<std_msgs::Float32MultiArray>("array", 1);

    image_pub = it_.advertise("/aruco_camera", 1);

    cameraMatrix = (Mat1f(3,3) << 1.4986230310335097e+03, 0., 5.9621656961862107e+02, 0.,1.5003680421420222e+03, 3.8539255070784276e+02, 0., 0., 1.);
    distCoeffs = (Mat1f(5,1) << -2.0389153175224519e-02, 1.6311037251535379e+00, 1.1359858738580822e-02, -2.3943006177546120e-03, -5.8474614091374315e+00);

    last_request = ros::Time::now();

    // Initialize matrices
    X = (Mat1f(3,1) << .2/2, .2/2 ,0);  // Position of marker in G-Frame.
    T = (Mat1f(3,1) << 0, 0, 0);  // Translation matrix; 3D B-Frame WRT GFrame (marker center)
    RT = Mat(3, 4, CV_32F, float(0)); // Rotation and Translation matrix for 3D
    rot_mat_yaw = (Mat1f(3,3) << 0, 0, 0);
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
    //////////////////////////
    // ARUCO IMPLEMENTATION //
    //////////////////////////
    Mat image = cv_ptr->image;
    vector<int> markerIds;
    vector< vector<Point2f> > markerCorners;
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
    aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
    vector<Vec3d> rvecs, tvecs;
    aruco::estimatePoseSingleMarkers(markerCorners, .2, cameraMatrix, distCoeffs, rvecs, tvecs);

    aruco::drawDetectedMarkers(cv_ptr->image, markerCorners, markerIds);

    image_pub.publish(cv_ptr->toImageMsg());

    std_msgs::Float32MultiArray array;

    //////////////////////
    // Pose Calculation //
    //////////////////////
    if (tvecs.size() > 0) {
      Vec3d rvec = rvecs.at(0);  // Only one marker being detected so far.

      //cout << rvec[2] << '\n';

      Vec3d tvec = tvecs.at(0);

      Point2f markerCorner = (markerCorners[0])[0];  // Assuming only one marker, take top left corner.
      u = (Mat1f(3,1) << markerCorner.x, markerCorner.y, float(1));  // Get pixel coordinates of top left corner.
      Rodrigues(rvec, rot_mat);
      rot_mat.convertTo(rot_mat, CV_32F);  // Get rotation matrix in 32-bit float.

      // Define all matrix variables & other//
      float Z_pr = tvec[2];
      float Z = 0; // Marker center position in G-Frame.

      // Populate RT Matrix:
      for (int i = 0; i < 3; i++) { // iterate rows.
        RT.at<float>(i,3) = tvec[i];  // Including translation vector
        for (int j = 0; j < 3; j++) { // iterate columns.
          RT.at<float>(i,j) = rot_mat.at<float>(i,j);  // Including rotation matrix
        }
      }

      float theta = -asin(rot_mat.at<float>(2,0));
      float psi = atan2(rot_mat.at<float>(2,1), rot_mat.at<float>(2,2));
      float yaw = atan2(rot_mat.at<float>(1,0) / cos(theta), rot_mat.at<float>(0,0) / cos(theta));

      // Populate Z-rotation matrix //
      rot_mat_yaw.at<float>(0,0) = cos(yaw);
      rot_mat_yaw.at<float>(1,0) = sin(yaw);
      rot_mat_yaw.at<float>(0,1) = -sin(yaw);
      rot_mat_yaw.at<float>(1,1) = cos(yaw);
      rot_mat_yaw.at<float>(2,2) = float(1);

      T = -rot_mat_yaw * X + cameraMatrix.inv() * (Z - Z_pr) * u;
 
      if (ros::Time::now() - last_request > ros::Duration(0.5)) {  // Print value of T every half a second.
        cout << T << '\n';
        last_request = ros::Time::now();
      }
    }

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::Rate r(20); // Repeat loop 20 times/sec.
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
