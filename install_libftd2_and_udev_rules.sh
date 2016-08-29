#!/bin/bash
dpkg -i ${BASH_SOURCE%/*}/lib/libftd2xx_1.1.12_amd64.deb
cp path/to/flexrayusbinterface/udev/30-ftdi.rules /etc/udev/rules.d/