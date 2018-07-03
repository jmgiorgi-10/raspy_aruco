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
		self.Tx = 0;
		self.Ty = 0;
		self.Tz = 0;

		sub = rospy.Subscriber("array", Float32MultiArray, self.callback);

		self.pub_vel = rospy.Publisher("vel", geometry_msgs.msg.TwistStamped, queue_size=10);
		self.pub_yaw = rospy.Publisher("yaw", geometry_msgs.msg.TwistStamped, queue_size=10);

		self.first_yaw = True;

		self.vel_cmd = geometry_msgs.msg.TwistStamped();
		self.yaw_cmd = geometry_msgs.msg.Quaternion();

		self.pid_x = PID(1.5, 0, .4, .8, 0);
		self.pid_y = PID(1.5, 0, .4, .8, 0);
		self.pid_z = PID(.8, 0, .2, .4, 1.5);

	def callback(self, array):
		if len(array.data) > 0:
			self.Tx = array.data[1];  # Body frame: z point ups; Marker frame: z points down (both right handed frames).
			self.Ty = array.data[0];
			self.Tz = array.data[2];
			self.yaw = array.data[3];  # Access current yaw in degrees.

		  ## Choose shortest setpoint, based on current yaw translation ##
			if self.first_yaw:
				if self.yaw > 0:
					self.pid_yaw = PID(1.5, 0, .4, .8, 0);
				else:
					self.pid_yaw = PID(-1.5, 0, -.4, .8, 0);
				self.first_yaw = False;

			self.update(); # Update PID outputs at each array callback.

		else:
			self.vel_cmd.twist.linear.x = 0;
			self.vel_cmd.twist.linear.y = 0;
			self.vel_cmd.twist.linear.z = 0;
			self.vel_cmd.twist.angular.z = 0;
			self.pub_vel.publish(self.vel_cmd);

	def update(self):
		self.pid_x.value = self.Tx;  # Update Translation vector with current detections.
		self.pid_y.value = self.Ty;
		self.pid_z.value = self.Tz;
		self.pid_yaw.value = self.yaw * 3.14/180;  # Convert back to radians/sec.
 		
 		self.pid_x.update();
 		self.pid_y.update();
 		self.pid_z.update();
 		self.pid_yaw.update();

 		#print(self.pid_x.output);

 		self.vel_cmd.twist.linear.x = self.pid_x.output;
 		self.vel_cmd.twist.linear.y = self.pid_y.output;
 		self.vel_cmd.twist.linear.z = self.pid_z.output;
 		self.vel_cmd.twist.angular.z = self.pid_yaw.output;

 		self.pub_vel.publish(self.vel_cmd);

	### Add main, and instantiate Controller class.
if __name__ == "__main__":
	rospy.init_node('controller');
	c0 = Controller();
	rospy.spin();









