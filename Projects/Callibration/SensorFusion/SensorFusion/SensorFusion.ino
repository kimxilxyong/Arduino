// IMPORTANT
// this is not a working example, this is just to show how to set the library
// if you need a working example please see the other example

#include <NXPMotionSense.h>
#include "SensorFusion.h" //SF
SF fusion;
NXPMotionSense imu;

#define G_PER_COUNT            0.0001220703125f  // = 1/8192
#define DEG_PER_SEC_PER_COUNT  0.0625f  // = 1/16
#define UT_PER_COUNT           0.1f

#define CAL_NO_SCALE

#ifdef CAL_NO_SCALE
const float accelCalScale = 1.0f;
const float gyroCalScale = 1.0f;
const float magCalScale = 1.0f;
#else
const float accelCalScale = G_PER_COUNT;
const float gyroCalScale = DEG_PER_SEC_PER_COUNT;
const float magCalScale = UT_PER_COUNT;
#endif

float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;


void setup() {

  Serial.begin(115200); //serial to display data
  // your IMU begin code goes here
  imu.begin();
  delay(10);
  
  // 11:14:21.303 -> -0.06,-0.02,0.03,  // CAL_SCALED
  // 17:16:50.755 -> 0.0320,-0.0363,-0.0017,  // CAL_NO_SCALE
  float calGyro[3] = {0.0, 0.0, 0.0};
  float calAccel[3] = {0.01, -0.06, 0.02};
  float calMag[3] = {0.0, 0.00, 0.0};


  imu.setAccelCal(accelCalScale, calAccel);
  imu.setGyroCal(gyroCalScale, calGyro);
  imu.setMagCal(magCalScale, calMag);
}

void loop() {

  // now you should read the gyroscope, accelerometer (and magnetometer if you have it also)
  // NOTE: the gyroscope data have to be in radians
  // if you have them in degree convert them with: DEG_TO_RAD example: gx * DEG_TO_RAD

  if (imu.available()) {
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

    deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
    //choose only one of these two:
    fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
    //fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick, it is slower but more accurate

    pitch = fusion.getPitch();
    roll = fusion.getRoll();    //you could also use getRollRadians() ecc
    yaw = fusion.getYaw();

    Serial.print("Pitch:\t"); Serial.print(pitch);
    Serial.print(" Roll:\t"); Serial.print(roll);
    Serial.print(" Yaw:\t"); Serial.print(yaw);
    Serial.println();
    delay(5);
  }
}
