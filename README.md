# eval_kit_ble_client_demo
By: Colton Ottley @ Bend Labs
Date: May 5th, 2020

## Introduction

Arduino Sketch for configuring NRF52 Adafruit Feather boards to connect to and stream wireless data from Bend Labs One and Two Axis Eval Kits. With this module you can connect to [One](https://www.bendlabs.com/products/1-axis-evaluation-kit/) and [Two Axis Eval Kit](https://www.bendlabs.com/products/2-axis-evaluation-kit/) from Bend Labs

This sketch configures an NRF52 based Arduino development board to operate as the client role to communicate with a Bend Labs One and Two Axis Eval Kit. Implements most of the functionality found in the Bend Labs Sensor Demo Android App.
 
### Tested Board: Adafruit Feather NRF52840

*Alternatives*

_Should work with: Adafruit Feather NRF52832 or similar NRF52 Based Bluefruit Board_
 
#### Tested With Sparkfun Pro nRF52840 Mini

It sort of works but you may have to change lines 99 & 100 in variant.h to:
*  #define PIN_SERIAL1_RX       (15)
*  #define PIN_SERIAL1_TX       (17)
 
Sketch connects to first Eval Kit found and then determines if the sensor is a one or two axis sensor. Enables stretch measurements if the sensor type is a one axis. Automatically  enables notifications on the angle measurement service and then prints out new data received. 

Uses received characters from the COM/Serial Port to trigger calibration steps. 
