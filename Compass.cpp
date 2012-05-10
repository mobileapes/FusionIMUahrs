#include "Arduino.h"
#include "Math.h"

#define _X_	0
#define _Y_	1
#define _Z_	2

float CompassHeading(int *magnetom, float roll, float pitch) {
	float cos_roll = cos(roll);
	float sin_roll = sin(roll);
	float sin_pitch = sin(pitch);

	// Tilt compensated Magnetic filed X:
	float MAG_X = (magnetom[_X_] * cos(pitch)) + (magnetom[_Y_] * sin_roll * sin_pitch) + (magnetom[_Z_] * cos_roll * sin_pitch);
	// Tilt compensated Magnetic filed Y:
	float MAG_Y = (magnetom[1] * cos_roll) - (magnetom[_Z_] * sin_roll);

	// Magnetic Heading
	return(fastAtan2(-MAG_Y, MAG_X));
}

/*
// Once you have your heading, you must then add your 'Declination Angle',
// which is the 'Error' of the magnetic field in your location.
// Find yours here: http://www.magnetic-declination.com/
// Mine is: 3° 11' WEST, which is 3.1833 Degrees, or (which we need) 0.0555590661 radians,
// I will use 0.0556
// If you cannot find your Declination, comment out these two lines, your
// compass will be slightly off.

#define DECLINATION_ANGLE	0.0556;

float CompassHeading(int *magnetom, float roll, float pitch) {
	float cos_roll = cos(roll);
	float sin_roll = sin(roll);
	float sin_pitch = sin(pitch);

	// Tilt compensated Magnetic filed X:
	float MAG_X = (magnetom[_X_] * cos(pitch)) + (magnetom[_Y_] * sin_roll * sin_pitch) + (magnetom[_Z_] * cos_roll * sin_pitch);
	// Tilt compensated Magnetic filed Y:
	float MAG_Y = (magnetom[1] * cos_roll) - (magnetom[_Z_] * sin_roll);

	// Magnetic Heading
	float heading = fastAtan2(-MAG_Y, MAG_X) += DECLINATION_ANGLE;

	if(heading < 0)			// Correct for when signs are reversed.
		heading += 2*PI;
	if(heading > 2*PI)		// Check for wrap due to addition of declination.
		heading -= 2*PI;

	return(heading);
}
*/
