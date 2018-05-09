#!/usr/bin/python
from __future__ import division
from pololu_driver import Single # serial controller for controller

import rospy
from std_msgs.msg import Float32

# TODO: if necessary, add more try and excepts for error catching


class Node(object):
    def __init__(self):
        """Init ros node"""
        rospy.init_node("pololu_node", log_level=rospy.DEBUG) #TODO: remove 2nd param when done debugging
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to pololu")
        self.last_set_speed_time = rospy.get_rostime()
        self.port = rospy.get_param("~port", "/dev/ttyACM0") # param, default

        self.TIMEOUT = 2  # time between hearing commands before we shut off the motors

        rospy.loginfo("pololu port: %s", self.port)
        rospy.loginfo("POLOLU NODE TIMEOUT = %s", self.TIMEOUT)

        # get device numbers from ros parameter server (see config/daisy.yamlss)
        self.devnum = 0

        # initalize Daisy chain serial controller
        self.controller = Single(self.devnum, port=self.port)
        # Subscribers
        self.vel_sub = rospy.Subscriber("~vel", Float32, self.vel_callback, queue_size=1)
        self._has_showed_message = False  # flag to indicate we have showed the motor shutdown message

        #rospy.sleep(1)

    def _stop_all_motors(self):
        """Send stop command to all daisy chained motors"""
        self.controller.stop()

    def run(self):
	"""Run the main ros loop"""
        rospy.loginfo("Starting Daisy node loop")
        r_time = rospy.Rate(10) #10 Hz looping

        while not rospy.is_shutdown():
            # TODO: consider making individual ones for each of the controllers
            if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > self.TIMEOUT:
                self._stop_all_motors()
                if (not self._has_showed_message):
                     rospy.loginfo("No commands received in the last %d seconds. SHUTTING DOWN MOTORS",self.TIMEOUT)
                self._has_showed_message = True
            else:
                self._has_showed_message = False

            r_time.sleep()

    def vel_callback(self, command):
	"""Command the motor based on the incoming command"""
        self.last_set_speed_time = rospy.get_rostime()

        rospy.logdebug("Velocity command to arm linear actuators: %.4f", command.data)


        motor_command = max(-1.0, min(command.data, 1.0)) # put bounds on the incoming command
        motor_command = int(command.data * 3200) # scale to that expected by drivers

        rospy.logdebug("Arm linear actuator serial command = %d", motor_command)

        # drive motor with the command
        self.controller.drive(motor_command)

    def shutdown(self):
	"""Handle shutting down the node"""
        rospy.loginfo("Shutting down pololu node")

        if hasattr(self, "sub"):
            # so these don't get called while the node is shutting down
            self.vel_sub.unregister()

        self._stop_all_motors()
        # quit()

if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting pololu node")
