# eval_kit_ble_client_demo
Arduino Sketch for configuring NRF52 Adafruit Feather boards to connect to and 
stream wireless data from Bend Labs One and Two Axis Eval Kits

 *  Connect to One and Two Axis Eval Kit from Bend Labs
 *  By: Colton Ottley @ Bend Labs
 *  Date: May 5th, 2020
 *  
 *  This sketch configures an NRF52 based Arduino development board
 *  to operate as the client role to communicate with a Bend Labs
 *  One and Two Axis Eval Kit. Implements most of the functionality
 *  found in the Bend Labs Sensor Demo Android App.
 *  
 *  Tested Board: Adafruit Feather NRF52840
 *  Should work with: Adafruit Feather NRF52832 or similar 
 *  NRF52 Based Bluefruit Board
 *  
 *  Tested With Sparkfun Pro nRF52840 Mini it sort of works but you may have to 
 *  change lines 99 & 100 in variant.h to:
 *  #define PIN_SERIAL1_RX       (15)
 *  #define PIN_SERIAL1_TX       (17)
 *  
 *  Sketch connects to first Eval Kit found and then determines
 *  if the sensor is a one or two axis sensor. Enables stretch 
 *  measurements if the sensor type is a one axis. Automatically 
 *  enables notifications on the angle measurement service and then 
 *  prints out new data received. 
 *  
 *  Uses received characters from the COM/Serial Port to trigger 
 *  calibration steps. 
