#!/bin/sh

# this assumes an existing installation of yosys and nextpnr (for ice40).

# nextpnr-ice40 --up5k --package sg48 --pcf ice40.pcf --json ice40.json --asc ice40.pnr
# icepack ice40.pnr ice40.bin

rm -f ice40.json
rm -f ice40.pnr
rm -f ice40.bin
rm -f bin2c

echo "" > ice40.cpp

yosys -q -p 'synth_ice40' -b json -o ice40.json ice40.ys

if [ $? -ne 0 ]; then
  exit 1
fi

nextpnr-ice40 --pre-pack ice40_pnr.py --up5k --package sg48 --pcf ice40.pcf --json ice40.json --asc ice40.pnr

if [ $? -ne 0 ]; then
  exit 1
fi

c++ -std=c++14 -O2 -o bin2c bin2c.cpp && icepack ice40.pnr | ./bin2c ice40_bitstream > ice40.cpp

if [ $? -ne 0 ]; then
  exit 1
fi

rm -f ice40.json
rm -f ice40.pnr
rm -f bin2c

