// Orientation (accelerometer & gyroscaope & magnetometer) using Mahony filter.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles (in radians: 1.57 is 90 degrees).
//
// For graphical display, this Processing sketch works:
// https://www.arduino.cc/en/Tutorial/Genuino101CurieIMUOrientationVisualiser

#include <NXPMotionSense.h>
#include <MahonyAHRS.h>
#include <Wire.h>
#include <EEPROM.h>

NXPMotionSense imu;
Mahony filter;

void setup() {
  Serial.begin(115200);
  imu.begin();
  //float calGyro[3] = {0.0, 0.0, 4.0};
  float calGyro[3] = {0.0, 0.0, 0.0};
  float calAccel[3] = {0.0, 0.0, 0.0};
  float calMag[3] = {0, 0.0, 0.0};  
  imu.setAccelCal(calAccel);
  imu.setGyroCal(calGyro);
  imu.setMagCal(calMag);
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, heading;

  if (imu.available()) {
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

    // Scale the gyroscope to the range Mahony expects
    float gyroScale = 0.097656f;
    gx = gx * gyroScale;
    gy = gy * gyroScale;
    gz = gz * gyroScale;

    // Update the Mahony filter
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    //filter.updateIMU(gx, gy, gz, ax, ay, az);
  }

  if (readyToPrint()) {
    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print(heading);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.println(roll);
  }
}


// Decide when to print
bool readyToPrint() {
  static unsigned long nowMillis;
  static unsigned long thenMillis;

  // If the Processing visualization sketch is sending "s"
  // then send new data each time it wants to redraw
  while (Serial.available()) {
    int val = Serial.read();
    if (val == 's') {
      thenMillis = millis();
      return true;
    }
  }
  // Otherwise, print 8 times per second, for viewing as
  // scrolling numbers in the Arduino Serial Monitor
  nowMillis = millis();
  if (nowMillis - thenMillis > 125) {
    thenMillis = nowMillis;
    return true;
  }
  return false;
}
