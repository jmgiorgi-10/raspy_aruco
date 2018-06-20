// Computer Vision Node for Marker detection / Pose estimation //

#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <vector>

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
	//float data[10] = { 1,2,3,4,5,6,7,8,9,10 };
	//Mat M(1, 10, CV_32F, data);
	//cout << M << "/n";

	ros::init(argc, argv, "cv_node3");
	ros::NodeHandle n;

	// Testing ros message with Vec3d data type.
	vector<Vec3d> test; 
	test = {{0,0,0}, {1,1,1}, {2,2,2}};
	//cout << test.at(1)[1] << '\n'; 

	ros::Publisher test_pub = n.advertise<std_msgs::Float32MultiArray>("array", 100);

	while (ros::ok()) {
		std_msgs::Float32MultiArray array;
		for (int i = 0; i < 3; i++) {
			array.data.push_back(test.at(2)[i]);
		}
		test_pub.publish(array);
	}


	return(0);
}