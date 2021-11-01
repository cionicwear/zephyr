#!/bin/bash

#might need to install python and pip3 install --upgrade pip

sudo apt-get -y install python3
apt-get install wget
wget https://apt.kitware.com/kitware-archive.sh
sudo bash kitware-archive.sh
ln -snf /usr/share/zoneinfo/$CONTAINER_TIMEZONE /etc/localtime && echo $CONTAINER_TIMEZONE > /etc/timezone
sudo apt install -y --no-install-recommends git cmake ninja-build gperf \
  ccache wget \
  python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
  make gcc gcc-multilib g++-multilib libsdl2-dev
python3 -m pip install --upgrade pip
pip3 install --user -U west
pip3 install --user -U imgtool
# from docker environment:
export PATH="/github/home/.local/bin:$PATH"
ZEPHYR_BASE=$(pwd)
#in the cloned repo
echo "Initializing west"
west init -l
west update
west zephyr-export
pip3 install --user -r scripts/requirements.txt
export ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb
export GNUARMEMB_TOOLCHAIN_PATH=/usr
export LC_ALL=C.UTF-8
export LANG=C.UTF-8#
echo $PATH
pip3 show -f west
ls -la
cd samples/bluetooth/hci_uart/
west build --board=nrf52833dk_nrf52820

# firmware version MAJOR.MINOR.PATCH (patch at most 5 characters)
echo "Get git version"
HASH=`git rev-parse --short HEAD`
echo "Get git describe"
NEWTVER=`git describe --tags --abbrev=0`

DVER=${NEWTVER//v}


echo "Signing image $DVER"
west sign -t imgtool -- -v "$DVER"