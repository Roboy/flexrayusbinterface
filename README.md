## Description ##
The FlexRayUsbInterface is the lowest communication layer, i.e. hardware interface.
It works through driver library [ftd2xx](http://www.ftdichip.com/Drivers/D2XX.htm).

# Dependencies #

In order to build this library you will need:
- cmake 
- GNU Make
- Boost version >= 1.58
- [Catkin](https://github.com/ros/catkin) (get the newest available) or `catkin_make` which comes with ROS jade or later
- Other library dependencies which need to be installed by executing the script:
```bash
./install_deps.sh
```

# Before running your program #
__NOTE: We recommend copying the udev rules file to /etc/udev/rules.d/, otherwise the communication with the ftdi device will require running your program with `sudo`__

```bash
sudo cp <path/to/flexrayusbinterface>/udev/30-ftdi.rules /etc/udev/rules.d/
```
_or_
```bash
sudo <path/to/flexrayusbinterface>/install_udev_rules.sh
```
