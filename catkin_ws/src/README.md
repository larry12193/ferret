# Ferret Borehole Robot

## Requirements

+ Ubuntu Mate 16.04 for Odroid xu4 (https://odroid.in/ubuntu_16.04lts)
+ ROS Kinetic full-desktop install
+ Catkin Tools
+ ROS serial package (http://wiki.ros.org/rosserial)
+ ROS laser_proc package (http://wiki.ros.org/laser_proc)

## Configuration

### Networking
ODROID:    192.168.1.59
SICK TIM:  192.168.4.2
PointGrey: 192.168.5.2

### Dependencies
```
sudo apt-get install ros-kinetic-serial
sudo apt-get install ros-kinetic-laser-proc
```

### Network Socket
Configure network socket receieve buffer size to accomidate the data load coming from the Point Grey Blackfly camera (see http://www.ptgrey.com/KB/10016)
```
sudo sysctl -w net.core.rmem_max=2048576 net.core.rmem_default=2048576
```

Add the following two lines to the end of /etc/sysctl.conf for this change to persist through reboot
```
net.core.rmem_max=2048576
net.core.rmem_default=2048576
```

## Installation

### Build repository
```
cd ~
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone https://github.com/larry12193/ferret.git
cd ..
catkin_make
```

### Install udev rules 
```\
cd ~/catkin_ws/udev_rules
sudo chmod +x install-udev.sh
sudo ./install-udev.sh
```

Please reboot device for udev rules to take effect.

## To Run
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch radferret radferret_init.launch
rosrun radferret radferret_scan_once
```

## Roboteq MicroBasic Scripting

The MicroBasic scripting launguage reference for this motor controller can be found at 
+ https://www.roboteq.com/index.php/docman/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v17/file

To interface directly with Roboteq SDC2130 motor controller, install the Roborun+ PC utility from (note this is windows only)
+ http://www.roboteq.com/index.php/support/downloads


