#!/usr/bin/env python
import rospy

class PID:
	def __init__(self, Kp, Ki, Kd, max_amplitude, setpoint):
		self.Kp = Kp;
		self.Ki = Ki;
		self.Kd = Kd;
		self.setpoint = setpoint;
		self.value = 0;
		self.error_int = 0;
		self.last_error = 0;
		self.output = 0;
		self.max_amplitude = max_amplitude;  # Max output amplitude.
		self.last_time = rospy.get_time();

	def update(self):
		error = self.value - self.setpoint;
		if error != 0:
			#print(error)
			p_term = self.Kp * error;
			dt = rospy.get_time() - self.last_time;
			self.last_time = rospy.get_time();
			self.error_int = (self.error_int + error) * dt;
			i_term = self.Ki * self.error_int;

			# Prevent integral windup
			if i_term > self.max_amplitude:
			  i_term = self.max_amplitude;
			if i_term < -self.max_amplitude:
				i_term = -self.max_amplitude; 

			error_diff = error - self.last_error;
			d_term = self.Kd * error_diff;

			self.output = p_term + i_term + d_term;  # PID Baby

			if self.output > self.max_amplitude:  # Output bound.
				self.output = self.max_amplitude; 
			elif self.output < -self.max_amplitude:
				self.output = -self.max_amplitude;

			#print(output)

			self.last_error = error;