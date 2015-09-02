ROS Driver for E-Puck Robot
============================

ROS Driver for [e-puck robot](http://www.e-puck.org/).
It is based on the [pyePuck library](https://github.com/mmartinortiz/pyePuck).

This driver is maintained by the [Verlab](http://verlab.dcc.ufmg.br/) Laboratory at Universidade Federal de Minas Gerais.

Requirements
------------
Libraries

* Python Bluetooth or Pybluez
* Python Image Library (PIL)

E-puck robots must run the [webots](www.cyberbotics.com) firmware (resources/firmware/webots-firmware-1.4.4.hex).


How to use the epuck driver
---------------------------

Run the epuck driver with the e-puck's  MAC address (eg. 10:00:E8:6C:D7:E8)

    $ roslaunch epuck_driver epuck_controller.launch epuck_address:='MAC'

Test teleoperating with the keyboard (the package ros-indigo-turtlebot-teleop must be installed)

    $ roslaunch epuck_driver epuck_teleop.launch





Some useful bluetooth commands
--------------------

To find the nearby bluetooth devices and get their MAC address:

	$ hcitool scan

To automatically set the bluetooth password when you start the epuck_controller.launch (the XXXX is the password, eg. 1515)

    $ bluetooth-agent XXX

