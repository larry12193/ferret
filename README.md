# Ferret Borehole Robot

## Requirements

+ Ubuntu Mate 16.04 for Odroid xu4 (https://odroid.in/ubuntu_16.04lts)
+ ROS Kinetic full-desktop install
+ Catkin Tools

## Configuration

### Networking
ODROID:    10.42.0.10
SICK TIM:  192.168.4.2
PointGrey: 192.168.5.2

### Dependencies
```
sudo apt-get install tmux
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

### Setup environment
Clone the repository to the desired directory and add the enviornment variable that points to the main ferret folder. Change the '/path/to/ferret' as needed for your specific setup. Then add in the location of where data should be saved to by replacing '/path/to/data/storage'. The setup.bash source and other internal programmatic operations depend on these enviornment variables being set properly.
```
git clone https://github.com/larry12193/ferret.git
echo "export FERRET_PATH=/path/to/ferret" >> ~/.bashrc
echo "export FERRET_LOG_DIR=/path/to/data/storage" >> ~/.bashrc
echo "source $FERRET_PATH/catkin_ws/devel/setup.bash"
echo ROS_IP=10.42.0.10
```

### Install SDK dependencies
For the camera to work without relying on an internet connection to fetch the camera SDK, enter the ferret/dependencies folder and run the install script for the ARM based FlyCapture SDK. Make sure the script is executable and run it as sudo. It will prompt you for some inputs.
```
cd dependencies
sudo chmod +x install-flycap.sh
sudo ./install-flycap.sh
```

### Install udev rules
```
cd catkin_ws/udev_rules
sudo chmod +x install-udev.sh
sudo ./install-udev.sh
```

Please reboot for udev rules to take effect.

## To Run
Launch the top level launch file to initialize all subsystems
```
roslaunch $FERRET_PATH/launch/ferret.launch
```
The executive accepts commands published to /executive/command to run HDR image sequences and LIDAR scans with video capture. 
STOP  = 0  (Stops all motion and returns to idle)
LIDAR = 1  (Runs single 360 degree scan)
HDR   = 2  (Runs single 360 degree HDR image sequence)

## Roboteq MicroBasic Scripting

The MicroBasic scripting launguage reference for this motor controller can be found at
+ https://www.roboteq.com/index.php/docman/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v17/file

To interface directly with Roboteq SDC2130 motor controller, install the Roborun+ PC utility from (note this is windows only)
+ http://www.roboteq.com/index.php/support/downloads

The script onboard performs a homing sequence on boot and then starts writing counter and endstop state data to the serial port that the higher level software uses to make decisions with. It will also accept a command to re-home the device if the executive is restarted without restarting the entire robot via a power cycle. The script can be found at catkin_ws/ferret_script.mbs, see for more details.

## Known Issues
1) The arduino that is used to control the LED lights and the IMU cannot currently be connected to the robot at the same time. There were issues observed with having the two serial devices connected, one on USB2 and the other on USB3. When one was written to or read from, the other would disconnect. Because of this, a seperate USB2 hub was used and solved the issue, however it got fried when I removed the plasic enclosure and heatshrunk it to fit inside the body. 

2) The rotation of the end is reversed in the TF tree within ROS, this causes the point clouds to be inverted. I believe this is the reason, but there could be something deeper going on underneath. First thing to be addressed after hardware is connected.
