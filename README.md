# Odometry

  Odometry is the use of data from motion sensors to estimate change in position over time.

This ROS node provides position estimates based on the following sources:

* [x] wheel velocities (dead reckoning)
* [ ] IMU sensor readings
* [ ] control inputs and robot dynamics


## Error

### Dead Reckoning

The position estimate will accumulate a small error over time, depending on how out of sync the message streams from the two motor controllers are.

In the most extreme case, when the two controllers are 90 degrees out of phase and the velocity varies a lot, the error can approach 25%. In the regular case it should however be less than 0.1%.

The accuracy can thus be increased by either

- decreasing the variation in velocity by increasing the sampling rate, or
- decrease the phase shift between the two motor controllers.