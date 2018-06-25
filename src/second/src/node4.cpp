#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

using namespace std;

geometry_msgs::TwistStamped vel_msg;

mavros_msgs::Altitude alt_msg;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

/*//////////////////////////
    ARUCO MARKER CALLBACK
*///////////////////////////


// Callback to tvec array from ARUCO marker
void ar_callback(const std_msgs::Float32MultiArray::ConstPtr& array) {
    if (array->data.size() != 0) {  // Check if any marker detected.
        if ((array->data)[0] > 0) {
            vel_msg.twist.linear.x = .3;  // Rudimentary x,y control system.
        } else if (array->data[0] < 0) {
            vel_msg.twist.linear.x = -.3;
        }
        if ((array->data)[1] > 0) {
            vel_msg.twist.linear.y = .3;
        } else if (array->data[1] < 0) {
            vel_msg.twist.linear.y = -.3;
        }
    }
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
    ros::Subscriber arr_sub = nh.subscribe<std_msgs::Float32MultiArray>("array", 1, ar_callback);
    ros::Publisher alt_pub = nh.advertise<std_msgs::Float32>("mavros/altitude", 10);

    //the setpoint publishing rate MUST be faster than 2Hz to not drop out of OFFBOARD mode.
    ros::Rate rate(20.0);

    vel_msg.twist.linear.x = 0;  // Set all initial velocities to zero.
    vel_msg.twist.linear.y = 0;
    vel_msg.twist.linear.z = 0;

    alt_msg.local = 1.8;  // Set local altitude message value.

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.8;

    std_msgs::Float32 alt_msg;
    alt_msg.data = 1.8;

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
            //continue;
        } else {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                
                last_request = ros::Time::now();
            }
            //continue;
            }
          }
        //if (current_state.armed && ros::Time::now() - last_request < ros::Duration(15.0)) {
        if ((ros::Time::now() - last_request < ros::Duration(10))) {
            local_pos_pub.publish(pose);
            cout << ros::Time::now() - last_request << "\n";
        } else {
            vel_pub.publish(vel_msg);
            // Reset vel_msg to null.
            vel_msg.twist.linear.x = 0;
            vel_msg.twist.linear.y = 0;
            vel_msg.twist.linear.z = 0;

            alt_pub.publish(alt_msg);
        }
        //    local_pos_pub.publish(pose);
        
        
        //} else {
          //  vel_pub.publish(vel_msg);  // After 5 seconds, switch to publish velocity message.
        //}
       // local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}