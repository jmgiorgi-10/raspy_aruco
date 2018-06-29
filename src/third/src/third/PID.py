#!/usr/bin/env python
import rospy

class PID:
	def __init__(self, Kp, Ki, Kd, max_amplitude):
		self.Kp = Kp;
		self.Ki = Ki;
		self.Kd = Kd;
		self.value = 0;
		self.setpoint = 0; 
		self.max_amplitude = max_amplitude;  # Max output amplitude.

	void update(self):
		error = value - setpoint;
		p_term = Kp * error;
	  dt = rospy.get_time();
	  error_int = (error_int + self.error) * dt;
	  i_term = Ki * error_int;

	  if i_term > max_amplitude:  # Prevent integral windup.
	  	i_term = max_amplitude;
	  if i_term < -max_amplitude:
	  	i_term = -max_amplitude; 

	  error_diff = self.error - self.last_error;
	  d_term = Kd * error_diff;
		self.output = p_term + i_term + d_term;  # PID Baby

		if self.output > max_amplitude:  # Output bound.
			self.output = max_amplitude; 
		elif self.output < -max_amplitude:
			self.output = -max_amplitude;

		last_error = error;