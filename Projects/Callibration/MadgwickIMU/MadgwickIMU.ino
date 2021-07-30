// Orientation (accelerometer & gyroscaope & magnetometer) using Mahony filter.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles (in radians: 1.57 is 90 degrees).
//
// For graphical display, this Processing sketch works:
// https://www.arduino.cc/en/Tutorial/Genuino101CurieIMUOrientationVisualiser

#include <NXPMotionSense.h>
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <EEPROM.h>


#include <elapsedMillis.h> 

NXPMotionSense imu;
//Mahony filter;
Madgwick filter;

elapsedSeconds secondsRuntime;
elapsedMillis milliSecondsRuntime;
unsigned int MAXIMUM_RUNTIME = 60*10;   // 10 minute
bool LEDState = LOW;

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

void setup() {
  secondsRuntime = 0;
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  imu.begin();
  filter.begin(100);

  // 11:14:21.303 -> -0.06,-0.02,0.03,  // CAL_SCALED
  // 17:16:50.755 -> 0.0320,-0.0363,-0.0017,  // CAL_NO_SCALE
  //float calGyro[3] = {0.0, 0.0, 0.0};
  //float calAccel[3] = {0.0, 0.0, 0.0};
  //float calMag[3] = {0.0, 0.0, 0.0};

  // 11:14:21.303 -> -0.06,-0.02,0.03,  // CAL_SCALED
  // 17:16:50.755 -> 0.0320,-0.0363,-0.0017,  // CAL_NO_SCALE
  float calGyro[3] = {0.0, 0.0, 0.0};
  float calAccel[3] = {0.0277, -0.0199, 0.022};
  float calMag[3] = {0.0, 0.00, 0.0};

  //   // CAL_SCALED
  // 10:57:05.403 -> -0.06,0.00,0.04,   // CAL_NO_SCALE
  // 20:9:17.422 -> 0.00,-0.03,0.03,
  //float calGyro[3] = {0.0, 0.0, -0.0};
  //float calAccel[3] = {0.0, 0.0, 0.0};
  //float calMag[3] = {-0.81, 0.51, -0.81};
  // 0:40:24.882 -> -0.01,-0.03,0.01
  //float calGyro[3] = {0.0, 0.0, -0.0};
  //float calAccel[3] = {0.0, 0.0, 0.0};
  //float calMag[3] = {-0.81, 0.51, -0.81};    

  //   // CAL_SCALED
  // 10:53:16.123 -> -0.06,-0.01,0.16,   // CAL_NO_SCALE
  //float calGyro[3] = {0.0, 1.0, 0.0};
  //float calAccel[3] = {0.0, 0.0, 0.0};
  //float calMag[3] = {0.0, 0.5, 0.0}; 

  //   // CAL_SCALED
  // 10:50:52.420 -> -0.61,-0.05,0.03, twoKi 0.00,   // CAL_NO_SCALE
  //float calGyro[3] = {0.0, 0.0, -0.5};
  //float calAccel[3] = {0.0, 0.0, 0.0};
  //float calMag[3] = {0.0, 0.7, 0.0}; 

  //   // CAL_SCALED
  // 10:19:17.797 -> -0.58,-0.06,0.13, twoKi 0.00, twoKp 1.00    // CAL_NO_SCALE
  //float calGyro[3] = {+0.5, +0.9, -0.5};
  //float calAccel[3] = {0.0, 0.0, 0.0};
  //float calMag[3] = {-0.5, 0.7, 0.0}; 

  //   // CAL_SCALED
  // 10:15:31.302 -> -0.08,-0.02,0.05, twoKi 0.00, twoKp 1.00    // CAL_NO_SCALE
  //float calGyro[3] = {0.0, 0.17, 0.0};
  //float calAccel[3] = {0.0, 0.0, 0.0};
  //float calMag[3] = {0.5, 0.5, 0.0}; 

  imu.setAccelCal(accelCalScale, calAccel);
  imu.setGyroCal(gyroCalScale, calGyro);
  imu.setMagCal(magCalScale, calMag);

  milliSecondsRuntime = 0;
}

#define LOWPASS_GYRO 0.625f
#define LOWPASS_ACCEL 100

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, heading;

  if (imu.available()) {
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

    /// Scale the gyroscope to the range Mahony expects
    float gyroScale = 0.097656f;
    //gx = gx * gyroScale;
    //gy = gy * gyroScale;
    //gz = gz * gyroScale;
    // Lowpass G filter
    gx = abs(gx) > LOWPASS_GYRO ? gx : 0.0; 
    gy = abs(gy) > LOWPASS_GYRO ? gy : 0.0; 
    gz = abs(gz) > LOWPASS_GYRO ? gz : 0.0; 
    // Lowpass A filter
    //ax = trunc(ax*LOWPASS_ACCEL) / LOWPASS_ACCEL; 
    //ay = trunc(ay*LOWPASS_ACCEL) / LOWPASS_ACCEL; 
    //az = trunc(az*LOWPASS_ACCEL) / LOWPASS_ACCEL; 



    // Update the Mahony filter
    //filter.update(gx, gy, gz, ax, ay, az, mx, my, my);
    //filter.update(gx, gy, gz, ax, ay, az, 0.0f, 0.0f, 0.0f);
    //filter.update(gx, gy, gz, 0.0f, 0.0f, 0.0f, mx, my, mz);

    filter.updateIMU(gx, gy, gz, ax, ay, az);   
  }

  
  if (readyToPrint()) {

    if (secondsRuntime >= MAXIMUM_RUNTIME) {
      if (secondsRuntime == MAXIMUM_RUNTIME) {
        Serial.println("Max Runtime reached"); 
        digitalWrite(LED_BUILTIN, LEDState);
      }
      LEDState = !LEDState;
      digitalWrite(LED_BUILTIN, LEDState);
      return;
    }

    // print the heading, pitch and roll
    roll = filter.getRoll();// * RAD_TO_DEG;
    pitch = filter.getPitch();// * RAD_TO_DEG;
    heading = filter.getYaw();// * RAD_TO_DEG;
    Serial.print(heading, 4);
    Serial.print(", \t");
    Serial.print(pitch, 4);
    Serial.print(", \t");
    Serial.print(roll, 4);

    Serial.print("\t Raw A: ");
    Serial.print(ax, 4);
    Serial.print('\t');
    Serial.print(ay, 4);
    Serial.print('\t');
    Serial.print(az, 4);
    Serial.println();
    
    /*
    Serial.print(", twoKi ");
    Serial.print(filter.twoKi, 4);    
    Serial.print(", twoKp ");
    Serial.print(filter.twoKp), 4;
    Serial.print(", status ");
    Serial.println(filter.status);        
    
    Serial.print("Raw A: ");
    Serial.print(ax);
    Serial.print('\t');
    Serial.print(ay);
    Serial.print('\t');
    Serial.print(az);
    Serial.print('\t');
    Serial.print("Raw G: ");
    Serial.print(gx);
    Serial.print('\t');
    Serial.print(gy);
    Serial.print('\t');
    Serial.print(gz);
    Serial.print('\t');
    /*
    Serial.print("RAW M: ");
    Serial.print(mx);
    Serial.print('\t');
    Serial.print(my);
    Serial.print('\t');
    Serial.print(mz);
    

    heading = filter.getYaw();    
    pitch = filter.getPitch();
    roll = filter.getRoll();

    Serial.print('  YPR: ');
    //Serial.print(RAD_TO_DEG * heading, 2);
    Serial.print(heading, 4);
    Serial.print('  ');
    //Serial.print(RAD_TO_DEG * pitch, 2);
    Serial.print(pitch, 2);
    Serial.print('  ');
    //Serial.print(RAD_TO_DEG * roll, 2);    
    Serial.print(roll, 2);    
    Serial.println();
    */
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
  // Otherwise, print 5 times per second, for viewing as
  // scrolling numbers in the Arduino Serial Monitor
  nowMillis = millis();
  if (nowMillis - thenMillis > 200) {
    thenMillis = nowMillis;
    return true;
  }
  return false;
}
