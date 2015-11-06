### Description ###
The FlexRayUsbInterface is the lowest communication layer, i.e. hardware interface.
It works through driver library [ftd2xx](http://www.ftdichip.com/Drivers/D2XX.htm).

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