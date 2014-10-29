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


How to use
-----------

Run the epuck_controller.launch with the e-puck's  MAC address (eg. 10:00:E8:6C:D7:E8)

    $ roslaunch epuck_driver epuck_controller.launch epuck_address:='MAC'
