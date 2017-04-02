#!/usr/bin/python
from __future__ import division
from pololu_driver import Daisy # serial controller for controller in Daisy chain

import rospy
from std_msgs.msg import Float32

# TODO: if necessary, add more try and excepts for error catching


class Node(object):
    def __init__(self):
        """Init ros node"""
        rospy.init_node("pololu_node", log_level=rospy.DEBUG) #TODO: remove 2nd param when done debugging
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to pololu daisy chain")

        self.port = rospy.get_param("/emcee/daisy/port", "/dev/ttyUSB0") # param, default
        # Subscribers
        self.arm_sub = rospy.Subscriber("/emcee/arm/vel", Float32, self.arm_vel_callback, queue_size=5)
        self.bucket_sub = rospy.Subscriber("/emcee/bucket/vel", Float32, self.bucket_vel_callback, queue_size=5)
        self.pinion_sub = rospy.Subscriber("/emcee/pinion/vel", Float32, self.pinion_vel_callback, queue_size=5)

        self.TIMEOUT = 2  # time between hearing commands before we shut off the motors

        # rospy.sleep(1)
        rospy.logdebug("Daisy chain port %s", self.port)
        rospy.logdebug("DAISY NODE TIMEOUT %s", self.TIMEOUT)

        # get device numbers from ros parameter server (see config/daisy.yamlss)
        self.arm_left_devnum = rospy.get_param("/emcee/daisy/linear_actuators/arm_left", "0")
        self.arm_right_devnum = rospy.get_param("/emcee/daisy/linear_actuators/arm_right", "1")
        self.bucket_left_devnum = rospy.get_param("/emcee/daisy/linear_actuators/bucket_left", "2")
        self.bucket_right_devnum = rospy.get_param("/emcee/daisy/linear_actuators/bucket_right", "3")
        self.pinion_devnum = rospy.get_param("/emcee/daisy/pinion", "4")

        # initalize Daisy chain serial controllers
        self.arm_left = Daisy(self.arm_left_devnum, port=self.port) # first one initialized must set the port
        self.arm_right = Daisy(self.arm_right_devnum)
        self.bucket_left = Daisy(self.bucket_left_devnum)
        self.bucket_right = Daisy(self.bucket_right_devnum)
        self.pinion = Daisy(self.pinion_devnum)

    def _stop_all_motors(self):
        """Send stop command to all daisy chained motors"""
        self.arm_left.stop()
        self.arm_right.stop()
        self.bucket_left.stop()
        self.bucket_right.stop()
        self.pinion.stop()

    def run(self):
	"""Run the main ros loop"""
        rospy.loginfo("Starting daisy node ros loop")
        r_time = rospy.Rate(10) #10 Hz looping

        while not rospy.is_shutdown():
            # TODO: consider making individual ones for each of the controllers
            if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > self.TIMEOUT:
                self._stop_all_motors()

            r_time.sleep()

    def arm_vel_callback(self, command):
	"""Command the arm linear actuators based on the incoming command"""
        self.last_set_speed_time = rospy.get_rostime()

        rospy.logdebug("Velocity command to arm linear actuators: %.4f", command.data)


        motor_command = max(-1.0, min(command.data, 1.0)) # put bounds on the incoming command
        motor_command = int(command.data * 3200) # scale to that expected by drivers

        rospy.logdebug("Arm linear actuator serial command = %d", motor_command)

        # drive both linear actuators with the same command
        self.arm_left.drive(motor_command)
        self.arm_right.drive(motor_command)


    def bucket_vel_callback(self, command):
	"""Command the bucket slinear actuators based on the incoming command"""
        self.last_set_speed_time = rospy.get_rostime()

        rospy.logdebug("Velocity command to bucket linear actuators: %.4f", command.data)

        motor_command = max(-1.0, min(command.data, 1.0)) # put bounds on the incoming command
        motor_command = int(command.data * 3200) # scale to that expected by drivers

        rospy.logdebug("Bucket linear actuator serial command = %d", motor_command)

        # drive both linear actuators with the same command
        self.bucket_left.drive(motor_command)
        self.bucket_left.drive(motor_command)


    def pinion_vel_callback(self, command):
	"""Command the rack and pinion based on the incoming command"""
        self.last_set_speed_time = rospy.get_rostime()

        rospy.logdebug("Velocity command to pinion motor: %.4f", command.data)

        motor_command = max(-1.0, min(command.data, 1.0)) # put bounds on the incoming command
        motor_command = int(command.data * 3200) # scale to that expected by drivers

        rospy.logdebug("Pinion motor serial command = %d", motor_command)

        # drive both linear actuators with the same command
        self.bucket_left.drive(motor_command)
        self.bucket_left.drive(motor_command)



    def shutdown(self):
	"""Handle shutting down the node"""
        rospy.loginfo("Shutting down daisy node")

        if hasattr(self, "sub"):
            # so these don't get called while the node is shutting down
            self.arm_sub.unregister()
            self.bucket_sub.unregister()
            self.pinion_sub.unregister()

        self._stop_all_motors()
        # quit()

if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting daisy node")
