
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

using namespace std;

geometry_msgs::TwistStamped vel_msg;
geometry_msgs::TwistStamped vel_msg2;
mavros_msgs::Altitude alt_msg;
mavros_msgs::State current_state;
geometry_msgs::Quaternion quaternion;
geometry_msgs::PoseStamped attitude;
mavros_msgs::AttitudeTarget set_pos{};

bool init;

bool linear;  // Switch between setting linear & angular velocities.

void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

void vel_CB(const geometry_msgs::TwistStamped::ConstPtr& vel) {
  vel_msg.twist.linear.x = vel->twist.linear.x;
  vel_msg.twist.linear.y = vel->twist.linear.y;
  vel_msg.twist.linear.z = -vel->twist.linear.z;  // Flip sign to change frame.
  //vel_msg.twist.angular.z = vel->twist.angular.z; 
  
  vel_msg2.twist.linear.x = 0;
  vel_msg2.twist.linear.y = 0;
  vel_msg2.twist.linear.z = 0;
  vel_msg2.twist.angular.x = 0;
  vel_msg2.twist.angular.y = 0;
  vel_msg2.twist.angular.z = vel->twist.angular.z;  // Message w/ yaw.
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
          ("mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
          ("mavros/setpoint_position/local", 10);
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
          ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
          ("mavros/set_mode");

  ros::Publisher att_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 10);

  ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("vel", 5, vel_CB);

  //the setpoint publishing rate MUST be faster than 2Hz to not drop out of OFFBOARD mode.
  ros::Rate rate(20.0);

  vel_msg.twist.linear.x = 0;  // Set all initial velocities to zero.
  vel_msg.twist.linear.y = 0;
  vel_msg.twist.linear.z = 0;
  vel_msg.twist.angular.z = 0;

  vel_msg2.twist.linear.x = 0;
  vel_msg2.twist.linear.y = 0;
  vel_msg2.twist.linear.z = 0;
  vel_msg2.twist.angular.z = 0;

  linear = true;

  init = true;

  // wait for FCU connection
  while(ros::ok() && !current_state.connected){
      ros::spinOnce();
      rate.sleep();
  }

  geometry_msgs::PoseStamped pose;

  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 2.3;

  //send a few setpoints before starting
  for(int i = 100; ros::ok() && i > 0; --i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  // LOOP 1 -> SET OFFBOARD
  while(ros::ok()) {
    if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
        if( set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
            break;  // Once offboard is enabled, start sending velocity commands in next loop
        }
        last_request = ros::Time::now();
      } else {
        if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
          if( arming_client.call(arm_cmd) &&
            arm_cmd.response.success){
            ROS_INFO("Vehicle armed");
          }
            last_request = ros::Time::now();
          }
        }

    local_pos_pub.publish(pose);

    ros::spinOnce();
    rate.sleep();
  }

  // LOOP 2 -> VELOCITY COMMANDS
  float yaw = 0;

  while(ros::ok()) {
 
    vel_pub.publish(vel_msg);

    ros::spinOnce();
    rate.sleep();
   }

  return 0;
}
