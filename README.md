# Capstone-Design-Software

The code included in this repository was the software developed for the Senior Design project I developed for our team.
The project was implementing a "smart CPR mannequin" by installing sensors into a preexisting CPR mannequin.
IR distance sensors, the ADXL345 accelerometer, a load cell with the HX711 sensor, a Raspberry Pi with speakers, and the Adafruit Bluefruit module were all installed to reach that goal.
In order to install the sensors and supporting systems, 3D printed fixtures were installed in place of previous compartments within the provided CPR mannequin.
Regardless, the software contained within this repository is the sensing side of software prior to full integration with another group member's work with the Bluefruit and Android tablet.

The system flow first starts at the Arduino level where it gathers sensor data from the ADXL345 for accelerometer data, HX711 for force load cell, and two ADCs for the IR distance sensor data.
The data is then encoded in a bit pattern detailed in the Arduino project file to be sent via the Bluefruit to the Android tablet.
Lung and heart sounds could be chosen in the Android app and are then relayed as two data bytes back to the Arduino.
The two audio data bytes are echoed to the Raspberry Pi over USB UART to be processed.

The Arduino "CapstoneFunc" project has all the sensor code integrated into a file with the loop preformatted to collect the data, send and receive data to and from the tablet, and send the data bytes to the Raspberry Pi.
The only dependencies are bogde's HX711 library which is included in the Arduino project file.
