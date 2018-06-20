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

class ImageResults {

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub;
public:
	ImageResults() 
		: it_(nh_)
	{
		pub = n2.advertise<std_msgs::Float32MultiArray>("array", 100);
		image_sub = it_.subscribe("iris/camera_red_iris/image_raw", 1, &ImageResults::image_callback, this);
	}

	void image_callback(const sensor_msgs::ImageConstPtr& msg) {
		cv_bridge::CvImagePtr cv_ptr;  // Will be ptr to img and properties in cv format.
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    

    
	}

private:
	//ros::NodeHandle na;
	//ros::NodeHandle n2;
	ros::Publisher pub;
	
	

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "ImageResults");
	ImageResults ir;
	ros::spin();
	return 0;
}