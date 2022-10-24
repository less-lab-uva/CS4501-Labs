#!/usr/bin/env python

class PID:
  # TODO FOR CHECKPOINTS 1-3
  # On node initialization
  def __init__(self, p=0.0, i=0.0, d=0.0):
    self.Kp = p
    self.Ki = i
    self.Kd = d
    self.lasterr = 0
    self.integral = 0

   # TODO FOR CHECKPOINTS 1-3
  def pid_loop(self, error, dt):
    self.integral += error * dt
    derivative = error - self.lasterr
    output = (self.Kp * error) + (self.Kd * derivative) + (self.Ki * self.integral)
    self.lasterr = error
    return output
