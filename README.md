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
