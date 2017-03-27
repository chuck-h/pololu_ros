# pololu_ros

This is a ROS driver for the Pololu Simple Motor Controller.  It uses the
Pololu Protocol to communicate over serial to control a motor.

For a project I am working on, we are using the daisy chain configuration.
If you want standard behavior, you will have to rewrite the Daisy chain so that
multiple objects don't share a port.  I am planning on doing that later because
it would be easy. If you are interested in this, raise an issue and I can work
on that.  Or you can submit a pull request.
