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

class Controller {

    ros::NodeHandle nh;
    ros::Subscriber t_sub; 
    ros::Publisher vel_pub;

	public:
		Controller() {
			t_sub = nh.subscribe<std_msgs::Float32MultiArray>("array", 1, &Controller::t_callback, this);
			vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel",100);
		}

		void t_callback(const std_msgs::Float32MultiArray::ConstPtr& arr_msg) {
			geometry_msgs::TwistStamped msg_vel;
			if (arr_msg->data.size() == 0) {
					msg_vel.twist.linear.x = 1;
          msg_vel.twist.linear.y = 0;
          msg_vel.twist.linear.z = 0;
          vel_pub.publish(msg_vel);
				return;  // Exit if no markers found
			} 
		  //float x_err = (arr_msg->data)[0];
		  int x_err = 5;
      if (x_err > 0) {
          msg_vel.twist.linear.x = 1;
          msg_vel.twist.linear.y = 0;
          msg_vel.twist.linear.z = 0;
          vel_pub.publish(msg_vel);
      }
      return;
		}

};

int main(int argc, char** argv) {

  ros::init(argc, argv, "controller");
  Controller c0;
  ros::spin();
  return 0;

}