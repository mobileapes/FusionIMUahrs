/*
 * Edit the file FusionIMUahrs.h and set the defined constant "#define IS_9DOF" to 1.
 */

#include <Wire.h>
#include <FusionIMUahrs.h>

float angles[3];  // roll, pitch, yaw

FusionIMUahrs imu = FusionIMUahrs();

void setup() {
  Serial.begin(115200);
  Wire.begin();

  delay(5);

  initGyro();
  initAccel();  
  initCompass();
  i2cInit();

  configIMU();

  // Pass functions as argument.
  imu.addReadGyroFunction(read_gyro);
  imu.addReadAccelFunction(read_accel);
  imu.addReadCompassFunction(read_compass);
  imu.init();

  delay(5);
}

void loop() {

  if(imu.update()) {
    imu.getRollPitchYaw(angles);
    // angles[0] = imu.getRoll();
    // angles[1] = imu.getPitch();
    // angles[2] = imu.getYaw();
  }

  Serial.print(angles[0]);
  Serial.print(", ");
  Serial.print(angles[1]);
  Serial.print(", ");
  Serial.println(angles[2]);

}
