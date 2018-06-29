#!/usr/bin/env python

######################################################################################
# Controller node subscribers to array, and outputs three PID output velocity values #
######################################################################################

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import geometry_msgs.msg
from PID import PID

class Controller:
	def __init__(self):
		self.value = 0;
		self.Tx = 0;
		self.Ty = 0;
		self.Tz = 0;

		sub = rospy.Subscriber("array", Float32MultiArray, callback);
		pub_vel_x = rospy.Publisher("vel_x", Float32, queue_size=10);
		pub_vel_y = rospy.Publisher("vel_y", Float32, queue_size=10);

		pub_vel = rospy.Publisher("vel", geometry_msgs.msg.TwistStamped, queue_size=10);
		vel_cmd = geometry_msgs.msg.TwistStamped();

		pid_x = PID(.05, 0, 0, 0.5);  # PIDs for x, y velocities.
		pid_y = PID(.05, 0, 0, 0.5);

	def callback(array):
		print("got to callback")
		if array.data.size() > 0:
			Tx = array.data[1];
			Ty = array.data[0];
			Tz = array.data[2];

			print(Tx);
			print(Ty);

			self.update(); # Update PID outputs at each array callback.

		else:

			print("got to else")

			vel_cmd.linear.x = 0;
			vel_cmd.linear.y = 0;

			pub_vel.publish(vel_cmd);

	def update(self):
		pid_x.value = Tx;  # Update Translation vector with current detections.
		pid_y.valiue = Ty;
 		
 		pid_x.update();
 		pid_y.update();

 		vel_cmd.linear.x = pid_x.output;
 		vel_cmd.linear.y = pid_y.output;

 		pub_vel.publish(vel_cmd);

 		pub_vel_x.publish(pid_x.output);
 		pub_vel_y.publish(pid_y.output);

	### Add main, and instantiate Controller class.
if __name__ == "__main__":
	rospy.init_node('controller');
	c0 = Controller();
	rospy.spin();









