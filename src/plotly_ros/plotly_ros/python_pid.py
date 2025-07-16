#!/usr/bin/env python3
'''
For the project the PID class variables:
current_value = 'carla/hero/speedometer'
output is published to carla/hero/vehicle_control_cmd_manual
'''


class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self.integral = 0
        self.previous_error = 0


    def compute(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error *dt
        derivative = (error - self.previous_error)/dt if dt>0 else 0
        
        output = (
            self.Kp * error + 
            self.Ki * self.integral +
            self.Kd * derivative
            )
        
        return output