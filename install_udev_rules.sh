#!/bin/bash
cp $(readlink -f ${BASH_SOURCE%/*})/udev/30-ftdi.rules /etc/udev/rules.d/
