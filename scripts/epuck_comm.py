#!/usr/bin/env python

import rospy
import yaml
import rospkg

import subprocess
from system_utils import launcher_lib

connection_info = {
  0: {
    'pin': '1593',
    'addr': '10:00:E8:6C:D7:C8'
  },
  4: {
    'pin': '1554',
    'addr': '10:00:E8:6C:D7:BD'
  },
  3: {
    'pin': '1589',
    'addr': '10:00:E8:6C:D7:E6'
  },
  10: {
    'pin': '1553',
    'addr': '10:00:E8:6C:EC:A7'
  }
}

def run():
  rospy.init_node("epuck_comm", anonymous=True)

  robot_id = rospy.get_param("~robot_id", 0)

  pin = connection_info[robot_id]["pin"]
  addr = connection_info[robot_id]["addr"]

  rospy.loginfo("Connecting %s :: %s" % (pin, addr))

  args = launcher_lib.create_args(
    epuck_id = robot_id,
    epuck_address = addr
  )

  pc = subprocess.Popen(['bluetooth-agent', pin])
  launcher_lib.launch("epuck_driver", "epuck_controller.launch", args)

  rospy.sleep(6)
  pc.kill()

  # for pin in connection_info:
    # addr = connection_info[pin]

    # rospy.loginfo("Connecting %s :: %s" % (pin, addr))

    # args = launcher_lib.create_args(
    #   epuck_id = pin,
    #   epuck_address = addr
    # )

    # launcher_lib.launch("epuck_driver", "epuck_controller.launch", args)

    # pc = subprocess.Popen(['bluetooth-agent', pin])

    # rospy.sleep(6)
    # pc.kill()

  rospy.spin()

if __name__ == "__main__":
  run()
