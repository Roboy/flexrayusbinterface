#!/usr/bin/env bash

prefix="/usr/local"

while [[ $# -gt 1 ]]
do
key="$1"

case $key in
    -p|--prefix)
    prefix="$2"
    shift # past argument
    ;;
    -h|--help)
    echo "usage: ./install_ftd2xx.sh [-h | --help] [-p <path> | --prefix <path>]"
    echo "\twhere <path> is the installation prefix (default: /usr/local)"
    shift # past argument
    ;;
    *)
            # unknown option
    ;;
esac
shift # past argument or value
done

wget http://www.ftdichip.com/Drivers/D2XX/Linux/libftd2xx-x86_64-1.3.6.tgz
tar xzf libftd2xx-x86_64-1.3.6.tgz
rm libftd2xx-x86_64-1.3.6.tgz
echo Installing headers
mkdir -p "${prefix}/include"
cp release/ftd2xx.h release/WinTypes.h "${prefix}/include/"
echo "Installing library"
mkdir -p "${prefix}/lib"
cp release/build/libftd2xx.so.1.3.6 "${prefix}/lib/"
rm -r release 
chmod 0755 "${prefix}/lib/libftd2xx.so.1.3.6"
ln -sf $(readlink -f ${prefix}/lib/libftd2xx.so.1.3.6) $(readlink -f ${prefix}/lib/libftd2xx.so)
