FusionIMUahrs
=============

AHRS Generic Sensor Arduino Library

FusionIMUahrs Library is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

For more detailed technical information about DCM IMU please refer to this
document: http://gentlenav.googlecode.com/files/DCMDraft2.pdf

This generic library, works with any sensor that forms part of a IMU
(Gyroscope, Accelerometer and Magnetometre - 6DOF/9DOF).

To use the library, functions for accessing the sensor data must be made
available in sketch project. These functions are passed to the library as
an argument. For more information, see the examples directory for some
example files to try it on.

To use this library, copy the FusionIMUahrs directory into the Arduino
libraries directory.

This has been tested with the following sparkfun boards:

- IMU 9 Degrees of Freedom - Sensor Stick (SEN-10724)
- IMU Digital Combo Board - 6 Degrees of Freedom ITG3200/ADXL345 (SEN-10121)
- Combination of two boards to make a 9DOF IMU
  IMU Digital Combo Board - 6 Degrees of Freedom ITG3200/ADXL345 (SEN-10121)
  Triple Axis Magnetometer Breakout - HMC5883L (SEN-10530)

Third party Arduino libraries:

- ITG-3200 gyroscope: http://code.google.com/p/itg-3200driver/
- ADXL345 accelerometer: http://code.google.com/p/adxl345-arduino/
- HMC5883L compass: http://code.bildr.org/download/976.zip

This is work in progress, and any corrections and additions are most welcome.
