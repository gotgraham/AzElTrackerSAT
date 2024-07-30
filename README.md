# AzElTrackerSAT
Arduino firmware to control https://www.thingiverse.com/thing:4664558, designed by jbyrns

Based off of AzElTracker, designed by https://github.com/ajohns1288/AzElTracker

This extends the AzElTracker by adding Wifi capability, specifically emulating the part of the PstRotator UDP interface that the CSN Technologies S.A.T. Controller uses. 

# To configure within the SAT controller:

1. Set the rotator type to "PstRotator"
2. Set the IP address to match that of this controller (you can issue an 'I' command over the USB serial connection to get the IP address)

# In the controller software:

1. Set your network SSID and password in FinalRotatorPort.ino
   const char * networkName = "your_wifi_ssid";
   const char * networkPswd = "your_wifi_password";
2. Set the IP address of your SAT controller in FinalRotatorPort.ino
   const char * sendUdpAddress = "sat_controller_ip";

## Items Needed For This Controller
1x Arduino Nano ESP32 or equivalent - https://www.amazon.com/Arduino-ABX00083-Bluetooth-MicroPython-Compatible/

***** 2x A4988 Stepper Boards. 
***** In testing, I'm seeing a lot of RFI on the 2m band. It sounds like the TMC2208 stepper board (drop in replacement) is substantially quieter. 
***** I'll update this with my findings once I have the boards. 
***** https://amazon.com/BIQU-Printer-Stepstick-TMC2208-Heatsink/

1x Stepper Breakout board for Nano - https://www.amazon.com/dp/B0834KK49M

2x Stepper Motors - https://www.amazon.com/dp/B08KVT823H

These items are optional - I disabled the homing for now

1x Small Magnet

1x Hall effect switch


## How it works
1: At startup, the azimuth is assumed to be due north (0deg) and the elevation runs a homing routine

2: Once homed, serial port is polled for commands from external program.

3: Rotors can be moved indepentently as well as together. Each can rotate continously or to a set position.

See section below for implemented commands

## Hardware adaptation

Since different people might want to use the same logic but for different hardware, the hardware specific values are in the rotorParams.h file. This way, you can tailor the code to your specific application without having to change mulitple variables all over the place

## Commands
If you choose not to use the wifi interface, the GS232 protocol is still fully supported: 

Command enumerations based on GS232 Protocol; works with 'rotctld -m 606 -r /dev/xxx' on RaspPi

CMD_R 'R' - Move right continously

CMD_L 'L' - Move left continously

CMD_U 'U' - Move up continously

CMD_D 'D' - Move down continously

CMD_STOP 'S' - Stop all axes

CMD_STOP_AZ 'A' - Stop azimuth only

CMD_STOP_EL 'E' - Stop elevation only

CMD_SPEED 'X' - Xs - Set speed to x (Azimuth only, 4: fast 1: slow)

CMD_MOVETO_AZ 'M' - Mxxx  - Moves to azimuth xxx

CMD_MOVETO 'W' - Wxxx yyy - Moves to azimuth xxx, elevation yyy

CMD_GET_AZ 'C' - Display azimuth position only (NOTE: C2 will display both axes)

CMD_GET_EL 'B' - Display elevation position only

//Non-Standard GS232 Commands

CMD_DEBUG 'Q' - Displays current positions/status/modes of rotor

CMD_SET_HOME 'F' - Unimplmented, may be used to set current position as home in future release

CMD_I 'I' - Print the controller IP address


## Drawbacks
Stepper motors do not have feedback. If the motor stalls or skips, your position will be off

## Future Work
Implement compass/accelerometer as position sensor input.
