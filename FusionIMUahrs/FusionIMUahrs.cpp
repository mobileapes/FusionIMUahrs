/*
 * FusionIMUahrs Library version YYYYMMDD
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
 */

#include "Arduino.h"
#include "FusionIMUahrs.h"
#include "Vector.h"
#include "Matrix.h"
#include "Math.h"
#if IS_9DOF == 1
#include "Compass.h"
#endif

FusionIMUahrs::FusionIMUahrs(void) {
	initializeValues();
}

void FusionIMUahrs::init(void) {
	float tmp_offsets[2][3] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};

	for(int n = 0; n < config.total_samples; n++) {
		function_read_gyro(sensor[GYRO].analog_raw);
		function_read_accel(sensor[ACCEL].analog_raw);

		for(int s = GYRO; s <= ACCEL; s++)
			for(int i = _X_; i <= _Z_; i++)
				tmp_offsets[s][i] += sensor[s].analog_raw[i];

		delay(config.sample_delay);
	}
	
	for(int s = GYRO; s <= ACCEL; s++)
		for(int i = _X_; i <= _Z_; i++)
			sensor[s].analog_offset[i] = (tmp_offsets[s][i] / config.total_samples) + 0.5;

	sensor[ACCEL].analog_offset[_Z_] -= (config.gravity * config.signal[ACCEL][_Z_]);

	timer = millis();
	delay(20);
}

void FusionIMUahrs::initializeValues(void) {
	gyro_dt = 0.02f;	// Integration time (DCM algorithm) We will run the integration loop at 50Hz if possible
	timer = 0;
#if IS_9DOF == 1
	counter = 0;
	mag_heading = 0.0f;
#endif

	config.total_samples = 32;
	config.sample_delay = 20;
	
	config.gravity = 0;
	for(int i = _X_; i <= _Z_; i++)
		config.gyro_gain[i] = 0.0f;

	config.kp[ROLLPITCH] = 0.02;
	config.kp[YAW] = 1.2;
	config.ki[ROLLPITCH] = 0.00002;
	config.ki[YAW] = 0.00002;

#if IS_9DOF == 1	
	for(int s = GYRO; s <= COMPASS; s++)
		for(int i = _X_; i <= _Z_; i++)
			config.signal[s][i] = 1;

	memcpy(sensor, (int [3][3][3]) {
		{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}},	// Gyro
		{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}},	// Accel
		{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}	// Compass
	}, 3*sizeof(Sensors));
#else
	for(int s = GYRO; s <= ACCEL; s++)
		for(int i = _X_; i <= _Z_; i++)
			config.signal[s][i] = 1;

	memcpy(sensor, (int [3][3][3]) {
		{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}},	// Gyro
		{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}	// Accel
	}, 2*sizeof(Sensors));
#endif

	for(int i = 0; i < 3; i++) {
		accel_vector[i] = 0.0f;
		omega.p[i] = 0.0f;					// Omega Proportional correction
		omega.i[i] = 0.0f;					// Omega Integrator
		angles[i] = 0.0f;
	}
	memcpy(matrix.dcm, (float[3][3]) {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, 3*3*sizeof(float));
	memcpy(matrix.update, (float[3][3]) {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}}, 3*3*sizeof(float));
	memcpy(matrix.temporary, (float[3][3]) {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, 3*3*sizeof(float));
}

void FusionIMUahrs::addReadGyroFunction(void (*f)(int *)) {
	function_read_gyro = f;
}

void FusionIMUahrs::addReadAccelFunction(void (*f)(int *)) {
	function_read_accel = f;
}

#if IS_9DOF == 1
void FusionIMUahrs::addReadCompassFunction(void (*f)(int *)) {
	function_read_compass = f;
}
#endif

void FusionIMUahrs::readData() {
	function_read_gyro(sensor[GYRO].analog_raw);
	function_read_accel(sensor[ACCEL].analog_raw);

	for(int s = GYRO; s <= ACCEL; s++)
		for(int i = _X_; i <= _Z_; i++) // roll, pitch, yaw
			sensor[s].corrected_value[i] = config.signal[s][i] * (sensor[s].analog_raw[i] - sensor[s].analog_offset[i]);

#if IS_9DOF == 1
	if(++counter > 5) {
		function_read_compass(sensor[COMPASS].analog_raw);	
		for(int i = _X_; i <= _Z_; i++)
			sensor[COMPASS].analog_raw[i] *= config.signal[COMPASS][i];

		mag_heading = CompassHeading(sensor[COMPASS].analog_raw, angles[_ROLL_], angles[_PITCH_]);
		counter = 0;
	}
#endif
}

bool FusionIMUahrs::update() {
	if((millis() - timer) >= config.sample_delay) {
		unsigned long timer_old = timer;
		timer = millis();
		gyro_dt = (timer > timer_old) ? (timer - timer_old)/1000.0f : 0.0f;

		readData();

		MatrixUpdate();
		Normalize();
		DriftCorrection();
		EulerAngles();
		
		//PrintData();
		return true;
	}
	return false;
}

void FusionIMUahrs::MatrixUpdate(void) {
	float gyro_vector[3] = {0.0f, 0.0f, 0.0f};
	float omega_vector[3] = {0.0f, 0.0f, 0.0f};

	for(int i = _X_; i <= _Z_; i++) {
		gyro_vector[i] = sensor[GYRO].corrected_value[i] * ToRad(config.gyro_gain[i]);
		accel_vector[i] = sensor[ACCEL].corrected_value[i];
		omega_vector[i] = gyro_vector[i] + omega.i[i] + omega.p[i];
	}

#if OUTPUTMODE == 1
	InitDCMmatrix(matrix.update, omega_vector);
#else
	InitDCMmatrix(matrix.update, gyro_vector);
#endif

	MatrixMultiply(matrix.dcm, matrix.update, matrix.temporary);
	MatrixAdition(matrix.dcm, matrix.temporary);
}

void FusionIMUahrs::InitDCMmatrix(float matrix[3][3], float vector[3]) {
	matrix[0][0] =  0;
	matrix[1][1] =  0;
	matrix[2][2] =  0;
	matrix[2][1] =  gyro_dt * vector[0];	//  x
	matrix[0][2] =  gyro_dt * vector[1];	//  y
	matrix[1][0] =  gyro_dt * vector[2];	//  z
	matrix[0][1] = -matrix[1][0];			// -z
	matrix[1][2] = -matrix[2][1];			// -x
	matrix[2][0] = -matrix[0][2];			// -y
}

void FusionIMUahrs::Normalize(void) {
	static float temporary[3][3] = {
		{0.0f, 0.0f, 0.0f},
		{0.0f, 0.0f, 0.0f},
		{0.0f, 0.0f, 0.0f}
	};
	static float renormalize = 0.0f; // static ok
	
	// static work here - very fast
	static float error = -(
		matrix.dcm[0][0] * matrix.dcm[1][0] +
		matrix.dcm[0][1] * matrix.dcm[1][1] +
		matrix.dcm[0][2] * matrix.dcm[1][2])/2.0f;

	for(int i = 0; i < 3; i++) {
		temporary[0][i] = (matrix.dcm[1][i] * error) + matrix.dcm[0][i];
		temporary[1][i] = (matrix.dcm[0][i] * error) + matrix.dcm[1][i];
	}
	VectorCrossProduct(temporary[2], temporary[0], temporary[1]);

	for(int i = 0; i < 3; i++) {
		renormalize = (3 - (
			temporary[i][0] * temporary[i][0] +
			temporary[i][1] * temporary[i][1] +
			temporary[i][2] * temporary[i][2]))/2.0f;
		for(int j = 0; j < 3; j++)
			matrix.dcm[i][j] = temporary[i][j] * renormalize;
	}
}

void FusionIMUahrs::DriftCorrection(void) {
	static float error_roll_pitch[3] = {0.0f, 0.0f, 0.0f};	// static ok
	float k_accel_weight[2] = {0.0f, 0.0f};
	
	static float accel_magnitude = sqrt(
		accel_vector[0] * accel_vector[0] +
		accel_vector[1] * accel_vector[1] +
		accel_vector[2] * accel_vector[2])/config.gravity;	// static ok

	static float accel_weight = constrain(1 - 2 * abs(1 - accel_magnitude), 0.0f, 1.0f);	// static ok
	k_accel_weight[_P_] = config.kp[ROLLPITCH] * accel_weight;
	k_accel_weight[_I_] = config.ki[ROLLPITCH] * accel_weight;

	VectorCrossProduct(error_roll_pitch, accel_vector, matrix.dcm[2]);

	for(int i = 0; i < 3; i++) {
		omega.p[i] = error_roll_pitch[i] * k_accel_weight[_P_];
		omega.i[i] += error_roll_pitch[i] * k_accel_weight[_I_];
	}

#if IS_9DOF == 1
	float error_course = matrix.dcm[0][0] * sin(mag_heading) - matrix.dcm[1][0] * cos(mag_heading);	// float here ok

	for(int i = 0; i < 3; i++) {
		float error_yaw = matrix.dcm[2][i] * error_course;
		omega.p[i] += error_yaw * config.kp[YAW];
		omega.i[i] += error_yaw * config.ki[YAW];
	}
//#else	
	// alternative yaw drift correction like a GPS or something more interesting like a camera or even a sound sensor.
	// I don't know, but the possibilities are numerous.
#endif
}

void FusionIMUahrs::EulerAngles(void) {
	angles[_PITCH_] = -asin(matrix.dcm[2][0]);
	angles[_ROLL_] = fastAtan2(matrix.dcm[2][1], matrix.dcm[2][2]);
	angles[_YAW_] = fastAtan2(matrix.dcm[1][0], matrix.dcm[0][0]);
}

void FusionIMUahrs::getRollPitchYaw(float *rpy) {
	rpy[_ROLL_] = ToDeg(angles[_ROLL_]);
	rpy[_PITCH_] = ToDeg(angles[_PITCH_]);
	rpy[_YAW_] = ToDeg(angles[_YAW_]);
}

float FusionIMUahrs::getRoll(void) {
	return ToDeg(angles[_ROLL_]);
}

float FusionIMUahrs::getPitch(void) {
	return ToDeg(angles[_PITCH_]);
}

float FusionIMUahrs::getYaw(void) {
	return ToDeg(angles[_YAW_]);
}

void FusionIMUahrs::PrintData(void) {
#if DEBUG_MODE == 1
    char str[512];
    Serial.print("!");

#if PRINT_EULER == 1
    sprintf(str, "ANG:%.2f,%.2f,%.2f", ToDeg(angles[_ROLL_]), ToDeg(angles[_PITCH_]), ToDeg(angles[_YAW_]));
    Serial.print(str);
#endif
#if PRINT_ANALOGS==1
	sprintf(str, ",AN:%d,%d,%d,%d,%d,%d",
		sensor[GYRO].analog_raw[_X_],
		sensor[GYRO].analog_raw[_Y_],
		sensor[GYRO].analog_raw[_Z_],
		sensor[ACCEL].analog_raw[_X_],
		sensor[ACCEL].analog_raw[_Y_],
		sensor[ACCEL].analog_raw[_Z_]
	);
    Serial.print(str);
#endif
    Serial.println("");
#endif
}
