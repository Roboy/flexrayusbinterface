### Description ###
The FlexRayUsbInterface is the lowest communication layer, i.e. hardware interface.
It works through driver library [ftd2xx](http://www.ftdichip.com/Drivers/D2XX.htm).

### Dependencies ###
install libftd2xx v1.1.12 (on ubuntu) via:

```
#!bash
cd path/to/flexrayusbinterface
sudo dpkg -i lib/libftd2xx_1.1.12_amd64.deb
```
install libftd2xx v1.1.12 (on fedora 21 and below) via:

```
#!bash
cd path/to/flexrayusbinterface
sudo yum localinstall lib/libftd2xx-1.1.12-2.x86_64.rpm
```
install libftd2xx v1.1.12 (on fedora 22 and above) via:

```
#!bash
cd path/to/flexrayusbinterface
sudo dnf install lib/libftd2xx-1.1.12-2.x86_64.rpm
```
### Build steps ###

```
#!bash
cd path/to/flexrayusbinterface
cmake .
make
```