#!/usr/bin/env python

######################################################################################
# Controller node subscribers to array, and outputs three PID output velocity values #
######################################################################################

import rospy
from std_msg.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
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

	rospy.spin();

	def callback(array):
		Tx = array.data[0];
		Ty = array.data[1];
		Tz = array.data[2];

		self.pid_x = PID(2, 0, 0, 0.5);
		self.pid_y = PID(2, 0, 0, 0.5);

		self.update(); # Update PID outputs at each array callback.

	def update(self):
		pid_x.value = Tx;  # Update Translation vector with current detections.
		pid_y.valiue = Ty;
 		
 		pid_x.update();
 		pid_y.update();

 		pub_vel_x.publish(pid_x.output);
 		pub_vel_y.publish(pid_y.output);

	### Add main, and instantiate Controller class.
if __name__ == "__main__":
	rospy.init_node('controller');
	c0 = Controller();









