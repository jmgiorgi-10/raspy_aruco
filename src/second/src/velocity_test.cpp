/*
Test sending velocity setpoint commands w/ MAVROS
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

class Vel { 
	ros::NodeHandle n;
	ros::Publisher vel_pub;
	geometry_msgs::TwistStamped vel_msg;
public:
	Vel() {
		vel_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel",100);
		vel_msg.twist.linear.x = 2;
		vel_msg.twist.linear.y = 2;
		vel_msg.twist.linear.z = 2;
	}

	void update() {
		vel_pub.publish(vel_msg);
		return;
	}

};

int main (int argc, char **argv) {
	ros::init(argc, argv, "vel_test");
	ros::NodeHandle n;
	ros::Publisher vel_pub;

  ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);

	ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

   mavros_msgs::SetMode offb_set_mode;
   offb_set_mode.request.custom_mode = "OFFBOARD";

	//Vel v;
	ros::Rate r(20);  // Set the publishing rate in Hertz.
	vel_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel",100);
	geometry_msgs::TwistStamped vel_msg;
	vel_msg.twist.linear.x = 2;
	vel_msg.twist.linear.y = 2;
	vel_msg.twist.linear.z = 2;
	vel_msg.twist.angular.z = 1;

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 2;

  for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    r.sleep();
    }

	ros::Time last_request = ros::Time::now();

	while(ros::ok()) {
		//v.update();
		//ros::spinOnce();

   if( current_state.mode != "OFFBOARD" &&
    (ros::Time::now() - last_request > ros::Duration(5.0))){
    if( set_mode_client.call(offb_set_mode) &&
        offb_set_mode.response.mode_sent){
        ROS_INFO("Offboard enabled");
    }
    last_request = ros::Time::now();
	 } 

	 last_request = ros::Time::now();

		vel_pub.publish(vel_msg);
		r.sleep();
	}

	return(0);
}