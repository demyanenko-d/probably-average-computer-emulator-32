#!/bin/bash

SEABIOS_VERSION=seabios-1.17.0

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

wget -nc https://www.seabios.org/downloads/$SEABIOS_VERSION.tar.gz
tar --skip-old-files -xaf $SEABIOS_VERSION.tar.gz
cd $SEABIOS_VERSION

# copy config
cp $SCRIPT_DIR/.config .

make

cp out/bios.bin $SCRIPT_DIR/..
cp out/vgabios.bin $SCRIPT_DIR/..