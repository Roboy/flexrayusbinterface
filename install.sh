#!/bin/bash
default_prefix="$(readlink -f ${BASH_SOURCE%/*})/third_party"
prefix="${default_prefix}"

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
    echo "\twhere <path> is the installation prefix (default: $default_prefix)"
    exit
    shift # past argument
    ;;
    *)
            # unknown option
    ;;
esac
shift # past argument or value
done

set -e
mkdir -p "${prefix}/include"
mkdir -p "${prefix}/lib"
touch "${prefix}/include"
touch "${prefix}/lib"
prefix=$(readlink -f ${prefix})
set +e

# ftd2xx
wget http://www.ftdichip.com/Drivers/D2XX/Linux/libftd2xx-x86_64-1.3.6.tgz
tar xzf libftd2xx-x86_64-1.3.6.tgz
rm libftd2xx-x86_64-1.3.6.tgz
cp release/ftd2xx.h release/WinTypes.h "${prefix}/include/"
cp release/build/libftd2xx.so.1.3.6 "${prefix}/lib/"
rm -r release 
chmod 0755 "${prefix}/lib/libftd2xx.so.1.3.6"
ln -sf libftd2xx.so.1.3.6 ${prefix}/lib/libftd2xx.so

# variant
wget https://github.com/mapbox/variant/archive/v1.1.5.tar.gz
tar xf v1.1.5.tar.gz
rm v1.1.5.tar.gz
cp -r variant-1.1.5/include/* ${prefix}/include
rm -rf variant-1.1.5/

# units
wget https://github.com/nholthaus/units/archive/v2.1.3.tar.gz
tar xf v2.1.3.tar.gz
rm v2.1.3.tar.gz
cp -r units-2.1.3/include/* ${prefix}/include
rm -rf units-2.1.3/

# yaml-cpp
wget https://github.com/jbeder/yaml-cpp/archive/bedb28f.tar.gz -O yaml-cpp.tar.gz
tar xf yaml-cpp.tar.gz 
rm yaml-cpp.tar.gz
mkdir -p yaml-cpp-bedb28fdb4fd52d97e02f6cb946cae631037089e/build
pushd yaml-cpp-bedb28fdb4fd52d97e02f6cb946cae631037089e/build
cmake .. -DCMAKE_INSTALL_PREFIX=${prefix} -DBUILD_SHARED_LIBS=ON
cmake --build . --target yaml-cpp/fast
cmake --build . --target install/fast
popd
rm -rf yaml-cpp-bedb28fdb4fd52d97e02f6cb946cae631037089e/
