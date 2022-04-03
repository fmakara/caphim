#!/bin/bash
ROOT=$(pwd)
# build the cross-compiler
cd micropython/
make -C mpy-cross/

# initialize the submodules
git submodule update --init -- lib/pico-sdk
git submodule update --init -- lib/tinyusb
cd lib/pico-sdk
git submodule update --init

cd $ROOT
#TODO: unpack diffs

