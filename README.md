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
install libftd2xx v1.3.6 (on fedora) via:
```
#!bash
cd path/to/flexrayusbinterface/lib
tar -xvzf libftd2xx-x86_64-1.3.6.tgz 
cd release/build
sudo -s
cp libftd2xx.* /usr/local/lib
chmod 0755 /usr/local/lib/libftd2xx.so.1.3.6
ln -sf /usr/local/lib/libftd2xx.so.1.3.6 /usr/local/lib/libftd2xx.so
cd ..
cp ftd2xx.h WinTypes.h /usr/local/include/
ldconfig -v|grep ftd2xx
exit
```

Ros dependency through logging functionality
```
#!bash
sudo apt-get install ros-indigo-desktop-full
```
ncurses
```
#!bash
sudo apt-get install libncurses5-dev 
```
I recommend copying the udev rules file to /etc, otherwise the commandline tool can only be run with root privileges
```
#!bash
sudo cp path/to/flexrayusbinterface/udev/30-ftdi.rules /etc/udev/rules.d/
```

### Build steps ###

```
#!bash
cd path/to/flexrayusbinterface
cmake .
make
```

### Run it ###

```
#!bash
cd path/to/flexrayusbinterface
./flexrayusbinterface

```

### Documentation ###

```
#!bash
cd path/to/flexrayusbinterface
doxygen Doxyfile
```