#!/usr/bin/env python


import rospy
import numpy as np
from cv_bridge.core import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from epuck.ePuck import ePuck

## Camera parameters
IMAGE_FORMAT = 'RGB_365'
CAMERA_ZOOM = 8

## Epuck dimensions
# Wheel Radio (cm)
WHEEL_RADIO = 4
# Separation between wheels (cm)
WHEEL_SEPARATION = 5.3


# available sensors
sensors = ['accelerometer', 'proximity', 'motor_position', 'light',
           'floor', 'camera']


class EPuckDriver(object):
    """

    :param epuck_name:
    :param epuck_address:
    """

    def __init__(self, epuck_name, epuck_address):
        self._bridge = ePuck(epuck_address, False)
        self._name = epuck_name

        self.enabled_sensors = {s: None for s in sensors}

    def greeting(self):
        """
        Hello by robot.
        """
        self._bridge.set_body_led(1)
        self._bridge.set_front_led(1)
        rospy.sleep(0.5)
        self._bridge.set_body_led(0)
        self._bridge.set_front_led(0)

    def disconnect(self):
        """
        Close bluetooth connection
        """
        self._bridge.close()

    def setup_sensors(self):
        """
        Enable epuck sensors based on the parameters.
        By default, all sensors are false.

        """
        # get parameters to enable sensors
        for sensor in sensors:
            self.enabled_sensors[sensor] = rospy.get_param('~' + sensor, False)

        # Only enabled sensors
        enable = [s for s, en in self.enabled_sensors.items() if en]

        # Enable the right sensors
        self._bridge.enable(**enable)

        if self.enabled_sensors['camera']:
            self._bridge.set_camera_parameters(IMAGE_FORMAT, 40, 40, CAMERA_ZOOM)

    def run(self):
        # Connect to the ePuck
        self._bridge.connect()

        # Setup the necessary sensors.
        self.setup_sensors()

        # Disconnect when rospy is going to down
        rospy.on_shutdown(self.disconnect)

        self.greeting()

        self._bridge.step()

        # Subscribe to Commando Velocity Topic
        rospy.Subscriber("mobile_base/cmd_vel", Twist, self.handler_velocity)

        # Sensor Publishers
        # rospy.Publisher("/%s/mobile_base/" % self._name, )

        if self.enabled_sensors['camera']:
            self.image_publisher = rospy.Publisher("camera", Image)

        # Spin almost forever
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._bridge.step()
            self.update_sensors()

            rate.sleep()

    def update_sensors(self):
        # print "accelerometer:", self._bridge.get_accelerometer()
        # print "proximity:", self._bridge.get_proximity()
        # print "light:", self._bridge.get_light_sensor()
        # print "motor_position:", self._bridge.get_motor_position()
        # print "floor:", self._bridge.get_floor_sensors()
        # print "image:", self._bridge.get_image()



        ## If image from camera
        if self.enabled_sensors['camera']:
            # Get Image
            image = self._bridge.get_image()
            print image
            if image is not None:
                nimage = np.asarray(image)
                image_msg = CvBridge().cv2_to_imgmsg(nimage, "rgb8")
                self.image_publisher.publish(image_msg)


    def handler_velocity(self, data):
        """
        Controls the velocity of each wheel based on linear and angular velocities.
        :param data:
        """
        linear = data.linear.x
        angular = data.angular.z

        # Kinematic model for differential robot.
        wl = (linear - (WHEEL_SEPARATION / 2.) * angular) / WHEEL_RADIO
        wr = (linear + (WHEEL_SEPARATION / 2.) * angular) / WHEEL_RADIO

        # At input 1000, angular velocity is 1 cycle / s or  2*pi/s.
        left_vel = wl * 1000.
        right_vel = wr * 1000.
        self._bridge.set_motors_speed(left_vel, right_vel)


def run():
    rospy.init_node("epuck_drive", anonymous=True)

    epuck_address = rospy.get_param("~epuck_address")
    epuck_name = rospy.get_param("~epuck_name", "epuck")

    EPuckDriver(epuck_name, epuck_address).run()


if __name__ == "__main__":
    run()
