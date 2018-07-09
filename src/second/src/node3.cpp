
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
bool out_of_sight = false;
mavros_msgs::State current_state;
geometry_msgs::Quaternion quaternion;
geometry_msgs::PoseStamped attitude;
mavros_msgs::AttitudeTarget set_pos{};

void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
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


  //the setpoint publishing rate MUST be faster than 2Hz to not drop out of OFFBOARD mode.
  ros::Rate rate(20.0);

  vel_msg.twist.linear.x = .7;  // Set all initial velocities to zero.
  vel_msg.twist.linear.y = 0;
  vel_msg.twist.linear.z = 0;
  vel_msg.twist.angular.z = 0;

  vel_msg2.twist.linear.x = 0;
  vel_msg2.twist.linear.y = 0;
  vel_msg2.twist.linear.z = 0;
  vel_msg2.twist.angular.z = 1;

  // wait for FCU connection
  while(ros::ok() && !current_state.connected){
      ros::spinOnce();
      rate.sleep();
  }

  geometry_msgs::PoseStamped pose;

  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 1.5;


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

  while(ros::ok()){
    if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
        if( set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
             ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
      } else {
        if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
          if( arming_client.call(arm_cmd) &&
            arm_cmd.response.success){
            ROS_INFO("Vehicle armed");
              
            last_request = ros::Time::now();
          }
          }
        }
      if ((ros::Time::now() - last_request < ros::Duration(7))) {
        local_pos_pub.publish(pose);
        cout << ros::Time::now() - last_request << "\n";
      } else {
        last_request = ros::Time::now();
        break;
      }
      ros::spinOnce();
      rate.sleep();
  }

  // while(ros::ok()) {
  //   if(ros::Time::now() - last_request < ros::Duration(3)) {
  //     vel_msg.twist.linear.x = 1; // Move in x-direction
  //     vel_pub.publish(vel_msg);
  //     cout << "linear" << '\n';
  //   } else if (ros::Time::now() - last_request < ros::Duration(6)) {
  //     vel_msg.twist.linear.x = 0;
  //     vel_pub.publish(vel_msg);
  //     cout << "angular" << '\n';  // Halt before rotating.
  //   } else if (ros::Time::now() - last_request < ros::Duration(9)) {
  //     vel_msg2.twist.angular.z = 1.5;
  //     vel_pub.publish(vel_msg2);  // Rotate.
  //   } else if (ros::Time::now() - last_request < ros::Duration(12)) {
  //     vel_pub.publish(vel_msg);  // Halt before moving again.
  //   }
  //   else {
  //     last_request = ros::Time::now();
  //   }
  // }

  pose.pose.orientation = tf::createQuaternionMsgFromYaw(5);

  while(ros::ok()) {
    //geometry_msgs::PoseStamped pose2;

    local_pos_pub.publish(pose);
  }


  return 0;
}