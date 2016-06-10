# UB-FIRE Repo

## Onboard Firmware
The hardware is split into two processing units (lowlevel and highlevel). The lowlevel (Atmega328) reads several sensors and sends them to the highlevel (Atmel SAM3X8E) 
processor that evaluates those values and takes proper action. The live data feed is handled and sent over RS-422.
The code for the ADC-Board (MCU) is using the Arduino framework and is (mostly) written in C.

## FireTamer
This is the ground station software for the UB-FIRE REXUS 20 flight computer
The groundstation software is written using the QT-framework version 5.7. 
Due to the fact that QT can compile for different development environements, the software can be compiled for Windows, Mac or Unix.

## Telecommands
The telecommands that can be sent- or received by the software and flight computer are specified in the telecommands excel file.

For further updates visit : https://github.com/tsaG1337/UBFIRE_REXUS20
For more information visit: http://rexusbexus.net/


UB-FIRE REXUS 20
2015-2016
by Patrick Bihn 
