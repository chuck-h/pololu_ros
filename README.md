# pololu_ros
This is a ROS driver for the Pololu Simple Motor Controller. It uses the Pololu
Protocol to communicate over serial to control a motor.

For a project I am working on, we are using the daisy chain configuration. If
you want standard behavior, you will have to rewrite the Daisy chain so that
multiple objects don't share a port. I am planning on doing that later because
it would be easy. If you are interested in this, raise an issue and I can work
on that. Or you can submit a pull request.


## pololu_node
### Subscribed Topics

`/peekay/arm/vel` (std_msgs/Float32)<br>
Command to control the arm linear actuators.  Expects -1.0 to 1.0

`/peekay/bucket/vel` (std_msgs/Float32)<br>
Command to control the bucket linear actuators. Expects -1.0 to 1.0

`/peekay/pinion/vel` (std_msgs/Float32)<br>
Command to control the pinion for the rack and pinion. Expects -1.0 to 1.0


## Package overview

The low level driver for this package is in the `src` directory.  It is setup
for a Daisy chain configuration, so all motors share the same serial connection
and all commands are sent with the device number and motors ignore commands
not directed to them.

The ros specific code for this package is in the `nodes` directory. This
starts a ros node and subscribes to topics to control the motors using the
Daisy chain driver.
