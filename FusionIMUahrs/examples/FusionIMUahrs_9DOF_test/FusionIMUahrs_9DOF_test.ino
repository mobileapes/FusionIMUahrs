//
// Edit the file FusionIMUahrs.h and set the defined constant "#define IS_9DOF" to 1.
//

#include <Wire.h>
#include <FusionIMUahrs.h>
#include <ITG3200.h>        // Download from: http://code.google.com/p/itg-3200driver/
#include <ADXL345.h>        // Download from: http://code.google.com/p/adxl345driver/
#include <HMC5883L.h>       // Download from: http://code.bildr.org/download/976.zip

float angles[3];  // roll, pitch, yaw

ITG3200 gyro;
ADXL345 accel;
HMC5883L compass;
FusionIMUahrs imu;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  delay(5);

  imu = FusionIMUahrs();

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
