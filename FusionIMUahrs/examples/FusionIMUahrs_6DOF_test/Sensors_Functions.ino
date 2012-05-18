/*
 * Data adquisition
 */

#include <ITG3200.h>        // Download from: http://code.google.com/p/itg-3200driver/
#include <ADXL345.h>        // Download from: http://code.google.com/p/adxl345driver/

#define TWI_FREQ 400000L     // 100kHz normal mode, 400kHz fast mode

ITG3200 gyro = ITG3200();
ADXL345 accel = ADXL345();

void configIMU() {

  // How many samples do you need to estimate the offset value for each sensor... (int>0)
  imu.config.total_samples = 1024;  // default: 32
  // delay between samples or how often, in milliseconds, the calculations will be done. (int>0) 
  imu.config.sample_delay = 2;  // default: 20 milliseconds

  // Tuning Parameters (double)
  // Proportional plus integral (PI) feedback controller to produce
  // a rotation rate adjustment for the gyros.
  imu.config.kp[ROLLPITCH] = 0.02;
  imu.config.kp[YAW] = 1.2;
  imu.config.ki[ROLLPITCH] = 0.00002;
  imu.config.ki[YAW] = 0.00002;

  // ADXL345 Sensitivity(from datasheet) => 4mg/LSB   1G => 1000mg/4mg = 256 steps
  // Tested value : 248
  imu.config.gravity = 248; // this equivalent to 1G in the raw data coming from the accelerometer

  // http://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf
  // ITG3200 gyro: 2000 degrees per second (dps) full scale
  // 70 mdps/digit; 1 dps = 0.07
  // ITG-3200 sensitivity is 14.375 LSB/(degrees/sec).
  imu.config.gyro_gain[_X_] = 0.06957;  // 1/14.375
  imu.config.gyro_gain[_Y_] = 0.06957;  // 1/14.375
  imu.config.gyro_gain[_Z_] = 0.06957;  // 1/14.375

  // Orientation
  // Check your board
  // IMU Digital Combo Board - 6 Degrees of Freedom ITG3200/ADXL345
  // Axis definition:
  //    X axis pointing forward (to the I2C connector)
  //    Y axis pointing to the right
  //    and Z axis pointing down (o). If up (+)
  // Positive pitch : nose up
  // Positive roll : right wing down
  // Positive yaw : clockwise
  imu.config.signal[GYRO][_X_] = -1;
  imu.config.signal[GYRO][_Y_] =  1;
  imu.config.signal[GYRO][_Z_] = -1;  
  imu.config.signal[ACCEL][_X_] = 1;
  imu.config.signal[ACCEL][_Y_] = 1;
  imu.config.signal[ACCEL][_Z_] = 1;
}

void i2cInit() {
  // Speed up i2c access
  // http://arduino.cc/playground/Code/ATMELTWI
  TWSR = 0;        // no prescaler => prescaler = 1
  TWBR = ((16000000L / TWI_FREQ) - 16) / 2; // change the I2C clock rate
  TWCR = 1<<TWEN;  // enable twi module, no interrupt
}

void initGyro() {
  gyro.reset();
  gyro.init(ITG3200_ADDR_AD0_LOW);
  delay(2);
}

void initAccel() {
  accel.powerOn();
  delay(2);
}

// function to pass as an argument to read gyroscope data.
void read_gyro(int *axes) {
  gyro.readGyroRaw(&axes[0], &axes[1], &axes[2]);

  return;
}

// function to pass as an argument to read accelerometer data.
void read_accel(int *axes) {
  accel.readAccel(&axes[0], &axes[1], &axes[2]);

  return;
}

