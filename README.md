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