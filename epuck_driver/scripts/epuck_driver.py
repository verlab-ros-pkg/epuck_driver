#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

from ePuck import ePuck


class EPuckDriver(object):
    def __init__(self, epuck_address):
        self._bridge = ePuck(epuck_address, False)

    def disconnect(self):
        self._bridge.close()

    def connect(self):
        self._bridge.connect()

    def run(self):
        # Connect with the ePuck
        self.connect()
        # Disconnect when rospy is going to down
        rospy.on_shutdown(self.disconnect)

        self._bridge.step()

        # Subscribe to Commando Velocity Topic
        rospy.Subscriber("/mobile_base/cmd_vel", Twist, self.handler_velocity)

        # Spin almost forever
        rospy.spin()

    def handler_velocity(self, data):
        linear = data.linear.x
        angular = data.angular.z

        left_vel = linear * 1500 + (angular * -1 * 500)
        right_vel = linear * 1500 + (angular * 1 * 500)

        self._bridge.set_motors_speed(left_vel, right_vel)
        self._bridge.step()

        print left_vel
        print right_vel
        print "--------------"


def run():
    rospy.init_node("epuck_drive", anonymous=True)

    epuck_address = rospy.get_param("~epuck_address")

    EPuckDriver(epuck_address).run()


if __name__ == "__main__":
    run()
