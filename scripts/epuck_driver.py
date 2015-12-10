#!/usr/bin/env python

import rospy
import numpy as np
from cv_bridge.core import CvBridge
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32MultiArray
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
SENSORS = [ 'accelerometer',
            'proximity',
            'motor_position',
            'light',
            'floor', 'camera']

class EPuckDriver(object):
    """
    :param epuck_name:    e-puck name
    :param epuck_address: e-puck bluetooth MAC address
    """
    def __init__(self, epuck_name, epuck_address):
        self._bridge = ePuck(epuck_address, False)
        self._name = epuck_name

        self.enabled_sensors = {sensor_name: None for sensor_name in SENSORS}

    def greeting(self):
        """
        Hello by robot.
        """

        self._bridge.set_body_led(1)
        self._bridge.set_front_led(1)
        self._bridge.step()

        rospy.sleep(0.5)

        self._bridge.set_body_led(0)
        self._bridge.set_front_led(0)
        self._bridge.step()

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
        for sensor in SENSORS:
            self.enabled_sensors[sensor] = rospy.get_param('~%s' % sensor, False)

        # Only enabled sensors
        enable = [sensor for sensor, enabled in self.enabled_sensors.items() if enabled]

        # Enable the right sensors
        self._bridge.enable(*enable)

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
        rospy.Subscriber("mobile_base/commands/velocity", Twist, self.handler_velocity)

        # Sensor Publishers

        if self.enabled_sensors['camera']:
            self.image_publisher = rospy.Publisher("camera", Image, queue_size = 10)

        if self.enabled_sensors['accelerometer']:
            self.accelerometer_publisher = rospy.Publisher("accelerometer", Vector3, queue_size = 10)

        if self.enabled_sensors['motor_position']:
            self.motor_position_publisher = rospy.Publisher("motor_position", UInt32MultiArray, queue_size = 10)

        if self.enabled_sensors["proximity"]:
            self.proximity_publisher = rospy.Publisher("proximity", UInt32MultiArray, queue_size = 10)

        # Spin almost forever
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._bridge.step()
            self.update_sensors()

            rate.sleep()

    def update_sensors(self):
        # print "light:", self._bridge.get_light_sensor()
        # print "floor:", self._bridge.get_floor_sensors()

        ## Send image
        if self.enabled_sensors['camera']:
            # Get Image
            image = self._bridge.get_image()

            if image is not None:
                nimage = np.asarray(image)
                image_msg = CvBridge().cv2_to_imgmsg(nimage, "rgb8")
                self.image_publisher.publish(image_msg)

        # Send accelerometer sensor values
        if self.enabled_sensors['accelerometer']:
            accel = self._bridge.get_accelerometer()

            self.accelerometer_publisher.publish( Vector3( *accel ) )

        # Send the motor positions
        if self.enabled_sensors["motor_position"]:
            motor_position = self._bridge.get_motor_position()

            self.motor_position_publisher.publish( UInt32MultiArray( data = list(motor_position) ) )

        # Send the proximity sensor values
        if self.enabled_sensors["proximity"]:
            proximity = self._bridge.get_proximity()

            self.proximity_publisher.publish( UInt32MultiArray( data = list(proximity) ) )

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
    rospy.init_node("epuck_driver", anonymous=True)

    epuck_address = rospy.get_param("~epuck_address")
    epuck_name = "epuck_%i" % rospy.get_param("~epuck_id", 0)

    EPuckDriver(epuck_name, epuck_address).run()

if __name__ == "__main__":
    run()
