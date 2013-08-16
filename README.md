butler
======

Dependencies
------------

```bash

apt-get install ros-groovy-joystick-drivers
apt-get install ros-groovy-openni-launch
apt-get install jpnevulator
apt-get install festival

svn checkout http://wu-robotics.googlecode.com/svn/trunk/rwi/b21_teleop
svn checkout http://wu-robotics.googlecode.com/svn/trunk/rwi/rflex
svn checkout http://wu-robotics.googlecode.com/svn/trunk/wu_ptu/ #(ptu46/ptu_action_server.py must be made executable)
git clone https://github.com/hawesie/laser_drivers.git
git clone https://github.com/barcuk/common.git

```


Initialisation
--------------

In separate terminals, run:

'''bash

rosrun butler 1.sh

'''

'''bash

rosrun butler 2.sh

'''

Then press ctrl+c on both.

Unplug and replug the Xtion's USB cable else it will not start.
  

Launching
---------

The main launch file including drivers and core programs:

'''bash

roslaunch butler butler.launch n:=x

'''
Where x is the number of QR markers


Navigation in the lab and area outside:

'''bash

roslaunch butler lg.launch

'''


Queueing and crowd navigation:

'''bash

roslaunch butler queue.launch

'''

This will only accept goals sent throught the QR system, as detailed below.


Queueing
--------

Goals can be added to the queue either by scanning the relevant QR code (not currently working due to a server change) or by running:

'''bash

rostopic pub /qr\_markers/goal std\_msgs/Int32 x

'''

Where x is the marker number.


Troubleshooting
---------------

* The Rflex system doesn't power up
    + Kill power to the robot and start it up again, making sure that the screen powers up **before** you turn the computers on.

* The laser doesn't work
    + Unplug the laser power connector and reconnect it, then relaunch butler.launch
    + This can happen if the laser driver has previously not closed properly, but also occasionally happens randomly.

* The PTU doesn't work
    + Turn the switch on the PTU control box off and back on, then relaunch butler.launch
    + This appears to happen randomly

* The costmaps don't clear properly
    + Relaunch lg.launch
    + This often happens on the first time that move_base is run on a new roscore

* queue.launch fails to find an initial plan when one should be available
    + The costmaps likely aren't clearing properly. Relaunch queue.launch

*  The computers turn off but Rflex stays on
    + The batteries need charging

*  The robot loses all power
    + One of the batteries is probably loose, and the others need charging. Check that all four batteries are fully pushed in.

* The robot has been on charge but quickly loses all power
    + One of the batteries is probably loose, and so it failed to charge.


