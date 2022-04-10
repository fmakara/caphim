#!/bin/bash
ROOT=$(pwd)
cd micropython/ports/rp2
if [ "$1" == "clean" ]; then
    make clean
else
    make -j4 && cp -v build-PICO/firmware.uf2 $ROOT/micropython.uf2
fi