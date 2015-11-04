### Description ###
The FlexRayUsbInterface is the lowest communication layer, i.e. hardware interface.
It works through driver library ftd2xx.
TODO: Filter necessary functions
TODO: create commandline interface for motor control
TODO: test with real hardware
TODO: pack into ros node (yet not clear which format, first compare with ros_control)

### Dependencies ###
install libftd2xx_1.1.12_amd64.deb (on ubuntu) via:

```
#!bash
cd path/to/flexrayusbinterface
sudo dpkg -i lib/libftd2xx_1.1.12_amd64.deb
```
### Build steps ###

```
#!bash
cd path/to/flexrayusbinterface
cmake .
make
```