/*
 * FusionIMUahrs Library version 0.1
 * AHRS Generic Sensor Library (Arduino 1.0 compatible)
 * Copyright (C) 2012-2012 Henrique Dias. All rights reserved.
 * 
 * FusionIMUahrs Library is based on sf9domahrs by Doug Weibel and Jose Julio:
 * http://code.google.com/p/sf9domahrs/
 * 
 * sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
 * Julio and Doug Weibel: http://code.google.com/p/ardu-imu/
 * 
 * FusionIMUahrs Library is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 * 
 * FusionIMUahrs Library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
 * more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License along
 * with FusionIMUahrs Library. If not, see <http://www.gnu.org/licenses/>.
 *
 * References:
 * http://code.google.com/p/sf9domahrs/
 * http://code.google.com/p/ardu-imu/source/browse/#svn%2Ftrunk
 * http://gentlenav.googlecode.com/files/DCMDraft2.pdf
 * 
 * Axis definition:
 *		X axis pointing forward (to the I2C connector)
 *		Y axis pointing to the right
 *		and Z axis pointing down (o). If up (+)
 * Positive pitch : nose up
 * Positive roll : right wing down
 * Positive yaw : clockwise
 * 
 */

#ifndef FusionIMUahrs_h
#define FusionIMUahrs_h

/*
 * Only for debugging purposes
 * OUTPUTMODE = 1 : will print the corrected data.
 * OUTPUTMODE = 0 : will print uncorrected data of the gyros (with drift)
 * 
 */
#define OUTPUTMODE 1

//#define PRINT_DCM 0		// Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0		// Will print the analog raw data
#define PRINT_EULER 1		// Will print the Euler angles Roll, Pitch and Yaw

#define IS_9DOF			1	// Set 0 to 6DOF.

#define GYRO			0
#define ACCEL			1
#define COMPASS			2
#define _X_				0
#define _Y_				1
#define _Z_				2

#define _P_				0
#define _I_				1

#define ROLLPITCH		0
#define YAW				1

#define _ROLL_				0
#define _PITCH_				1
#define _YAW_				2

#define ToRad(x) ((x)*0.017453293f)		//	ToRad(x) ((x)*PI/180)
#define ToDeg(x) ((x)*57.295779513f)	//	ToDeg(x) ((x)*180/PI)

typedef struct {
	int sample_time;
	int gravity;
	float gyro_gain[3];
	double kp[2];			// {Kp_RollPitch, Kp_Yaw}
	double ki[2];			// {Ki_RollPitch, Ki_Yaw}
#if IS_9DOF == 1
	int signal[3][3];		// Orientation - Correct directions x,y,z - gyros, accels, magnetormeter
#else
	int signal[2][3];		// Orientation - Correct directions x,y,z - gyros, accels, magnetormeter
#endif
} Configurations;

typedef struct {
	int analog_raw[3];		// array that stores the raw data of the sensors
	int corrected_value[3];	// array that stores the corrected data of the sensors
	int analog_offset[3];	// Array that stores the Offset of the sensors
} Sensors;

/*
 * Tuning Parameters (double)
 * Proportional plus integral (PI) feedback controller to produce a
 * rotation rate adjustment for the gyros.
 */
typedef struct {
	float p[3];
	float i[3];
} Corrections;

typedef struct {
	float dcm[3][3];
	float update[3][3];
	float temporary[3][3];
} Matrices;

class FusionIMUahrs {
    public:
		FusionIMUahrs(void);
    	void init(void);
    	void addReadGyroFunction(void (*f)(int *));
    	void addReadAccelFunction(void (*f)(int *));
#if IS_9DOF == 1
     	void addReadCompassFunction(void (*f)(int *));
#endif
    	bool update(void);
    	void getRollPitchYaw(float *rpy);
    	float getRoll(void);
    	float getPitch(void);
    	float getYaw(void);
    	
		Configurations config;

    private:
		void initializeValues(void);
		void (*function_read_gyro)(int *);
		void readGyro(void);
		void (*function_read_accel)(int *);
		void readAccel(void);
		void MatrixUpdate(void);
		void Normalize(void);
		void DriftCorrection(void);
		void createDCM(float matrix[3][3], float vector[3]);
		void EulerAngles(void);
//		void debug(char *str, int *a);
		void PrintData(void);

		float gyro_dt;
		unsigned long timer;
		float accel_vector[3];
		Corrections omega;

		Matrices matrix;
		float angles[3];
#if IS_9DOF == 1
		void (*function_read_compass)(int *);
		void readCompass(void);

		Sensors sensor[3];
		float mag_heading;
		unsigned int counter;
#else
		Sensors sensor[2];
#endif
};

#endif
