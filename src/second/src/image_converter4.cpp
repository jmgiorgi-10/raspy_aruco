#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <vector>

using namespace cv;
using namespace std;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub;
  ros::NodeHandle n2;
  ros::Publisher pub;
  ros::Publisher yaw_pub;
  Mat cameraMatrix;
  Mat distCoeffs;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("iris/camera_red_iris/image_raw", 1,
      &ImageConverter::imageCb, this);
    pub = n2.advertise<std_msgs::Float32MultiArray>("array", 5);
   // yaw_pub = n2.advertise<std_msgs::Float32>("yaw_array", 1);

    image_pub = it_.advertise("/aruco_camera", 1);

    // Defining camera values
    cameraMatrix = (Mat1f(3,3) << 476.7030836014194, 0.0, 400.5, 0.0, 476.7030836014194, 400.5, 0.0, 0.0, 1.0);
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
    aruco::estimatePoseSingleMarkers(markerCorners, 0.3, cameraMatrix, distCoeffs, rvecs, tvecs);

    aruco::drawDetectedMarkers(cv_ptr->image, markerCorners, markerIds);

    float length = 3.0;

    if (rvecs.size() > 0) {
      aruco::drawAxis(cv_ptr->image, cameraMatrix, distCoeffs, rvecs.at(0), tvecs.at(0), length);
    }

    image_pub.publish(cv_ptr->toImageMsg());

    std_msgs::Float32MultiArray array;

    //////////////////////
    // Pose Calculation //
    //////////////////////
    if (tvecs.size() > 0) {
      Vec3d rvec = rvecs.at(0);  // Only one marker being detected so far.
      Vec3d tvec = tvecs.at(0);

      Point2f markerCorner = (markerCorners[0])[0];  // Assuming only one marker, take top left corner.
      Mat u = (Mat1f(3,1) << markerCorner.x, markerCorner.y, float(1));  // Get pixel coordinates of top left corner.

      Mat rot_mat;
      Mat t_mat = (Mat1f(3,1) << 0.0, 0.0, 0.0);
      Mat xx = (Mat1f(3,1) << 0.0, 0.0, 0.0);
      Mat xx_prime = (Mat1f(3,1) << 0.0,0.0,0.0);  // Center of aruco marker, in aruco frame.
      for (int i = 0; i < 3; i++) {
       t_mat.at<float>(i,0) = tvec[i];
      }
      Rodrigues(rvec, rot_mat);
      rot_mat.convertTo(rot_mat, CV_32F);  // Convert rotation matrix output to 32-bit float.
      xx = rot_mat.t()*xx_prime - rot_mat.t()*t_mat;

      for (int i = 0; i < 3; i++) {
        //array.data.push_back(xx.at<float>(i,0));
      }

      // Define all matrix variables & other//
      float Z_pr = tvec[2];
      float Z = 0; // Marker center position in G-Frame.
      Mat X = (Mat1f(3,1) << .3/2,.3/2, 0);  // Position of marker in G-Frame.

      Mat X_h = (Mat1f(4,1) << .3/2, .3/2 , 0 ,1); // (Homogeneous coords. position of marker in G-Frame).
      Mat T = (Mat1f(3,1) << 0,0,0);  // Translation matrix; 3D B-Frame WRT GFrame (marker center)

      Mat RT = Mat(3, 4, CV_32F, float(0)); // Rotation and Translation matrix for 3D homogeneous coordinates. G-Frame --> B-Frame (camera).
      // Populate RT Matrix:
      for (int i = 0; i < 3; i++) { // iterate rows.
        RT.at<float>(i,3) = tvec[i];  // Including translation vector
        for (int j = 0; j < 3; j++) { // iterate columns.
          RT.at<float>(i,j) = rot_mat.at<float>(i,j);  // Including rotation matrix
        }
      }
      //Construct yaw rotation matrix.
      // Mat rot_mat_yaw;
      // Vec3d rvec_yaw;
      // rvec_yaw[0] = float(0);
      // rvec_yaw[1] = float(0);
      // rvec_yaw[2] = rvec[2];
      // Rodrigues(rvec_yaw, rot_mat_yaw);
      // rot_mat_yaw.convertTo(rot_mat_yaw, CV_32F);  // Convert elements to 32-bit Float.

      // Convert rotation matrix to Euler angles.

      float theta = -asin(rot_mat.at<float>(2,0));
      float psi = atan2(rot_mat.at<float>(2,1), rot_mat.at<float>(2,2));
      float yaw = atan2(rot_mat.at<float>(1,0) / cos(theta), rot_mat.at<float>(0,0) / cos(theta));

      // Populate Z-rotation matrix //
      Mat rot_mat_yaw = (Mat1f(3,3) << 0, 0, 0);
      rot_mat_yaw.at<float>(0,0) = cos(yaw);
      rot_mat_yaw.at<float>(1,0) = sin(yaw);
      rot_mat_yaw.at<float>(0,1) = -sin(yaw);
      rot_mat_yaw.at<float>(1,1) = cos(yaw);
      rot_mat_yaw.at<float>(2,2) = float(1);

      T = -rot_mat_yaw * X + cameraMatrix.inv() * (Z - Z_pr) * u;
 
      for (int i = 0; i < 2; i++) {
        array.data.push_back(T.at<float>(i, 0));  // Tx, Ty from VPHEA.
      }
      array.data.push_back(tvec[2]);  // Add Tz from VPBEA.






      yaw = yaw * 180/3.14;

      array.data.push_back(yaw);  // Push back yaw error in degrees.

      pub.publish(array);

    } else {
      std_msgs::Float32MultiArray array;
      pub.publish(array);  // Publish empty array if no marker detected.
    }
    
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");

  ImageConverter ic;
  ros::Rate r(40);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
