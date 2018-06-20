#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

// Global cringe alert

geometry_msgs::TwistStamped msg_vel;

void t_callback(const std_msgs::Float32MultiArray::ConstPtr& arr_msg) {
		//if (arr_msg->data.size() == 0) {
		//		return;  // Exit if no markers found
		//	} 		
		//int x_err = 5; // Just for initial testing purpose.
		//if (x_err > 0) {
          msg_vel.twist.linear.x = 1;
          msg_vel.twist.linear.y = 0;
          msg_vel.twist.linear.z = 0;
      // else {
      //	msg_vel.twist.linear.x = 0;
      //	msg_vel.twist.linear.y = 0;
      //	msg_vel.twist.linear.z = 0;
     // }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_control");

	ros::NodeHandle nh;
    ros::Subscriber t_sub; 
    ros::Publisher vel_pub;

    ros::Rate r(20); // 20 Hertz.

    t_sub = nh.subscribe<std_msgs::Float32MultiArray>("array", 1, t_callback);
	vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel",100);

	while(ros::ok()) {
		vel_pub.publish(msg_vel);
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}