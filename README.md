# Odometry

ROS node providing position estimates based on the wheel velocities of the robot.

## Error

The position estimate will accumulate a small error over time, depending on how out of sync the message streams from the two motor controllers are.

In the most extreme case, when the two controllers are 90 degrees out of phase and the velocity varies a lot, the error can approach 25%. In the regular case it should however be less than 0.1%.

The accuracy can thus be increased by either

- decreasing the variation in velocity by increasing the sampling rate, or
- decrease the phase shift between the two motor controllers.