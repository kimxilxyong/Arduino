/*
 This sketch shows to initialize the filter and update it with the imu output. It also shows how to get the euler angles of the estimated heading orientation.
 */

#include <imuFilter.h>
#include <NXPMotionSense.h>
#include <elapsedMillis.h> 

elapsedSeconds secondsRuntime;

// Sensor fusion
constexpr float GAIN = 0.1;     // Fusion gain, value between 0 and 1 - Determines heading correction with respect to gravity vector. 
imuFilter <&GAIN> fusion;

// Imu sensor
NXPMotionSense imu;

// Enable sensor fusion [ 1 = yes; 0 = no]
#define FUSION 1

#define CAL_NO_SCALE 1

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

  imu.begin();

  secondsRuntime = 0;
  pinMode(LED_BUILTIN, OUTPUT);

  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, heading;

  // Calibrate imu
  //imu.setup();
  //imu.setBias();
  //float calGyro[3] = {0.0, 0.0, 0.0};
  //float calAccel[3] = {0.0, 0.0, 0.0};
  //float calMag[3] = {0.0, 0.00, 0.0};
  float calGyro[3] = {0.0, 0.0, 0.0};
  float calAccel[3] = {0.01, -0.06, 0.02};
  float calMag[3] = {0.0, 0.00, 0.0};

  imu.setAccelCal(accelCalScale, calAccel);
  imu.setGyroCal(gyroCalScale, calGyro);
  imu.setMagCal(magCalScale, calMag);
   
  while (!imu.available());

  if (imu.available()) {
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

    #if FUSION
    // Set quaternion with gravity vector
    fusion.setup( ax, ay, az );     
    //fusion.setup( 0.0, 0.0, 1.0 );     
    #else 
    // Just use gyro
    fusion.setup();                                   
    #endif
  }

  Serial.begin(115200);
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, heading;

  // Update filter:
  if (imu.available()) {
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

        /// Scale the gyroscope to the range Mahony expects
    float gyroScale = 0.097656f;
    gx = gx * gyroScale;
    gy = gy * gyroScale;
    gz = gz * gyroScale;
    // Lowpass filter
    #define LOWPASS_GYRO 0.0625f
    gx = abs(gx) > LOWPASS_GYRO ? gx : 0.0; 
    gy = abs(gy) > LOWPASS_GYRO ? gy : 0.0; 
    gz = abs(gz) > LOWPASS_GYRO ? gz : 0.0; 


    #if FUSION
    //Fuse gyro and accelerometer
    fusion.update( gx, gy, gz, ax, ay, az );    
    #else
    // Only use gyroscope
    fusion.update( imu.gx(), imu.gy(), imu.gz() );
    #endif
  }
  if (readyToPrint()) {
    // Display angles:
    Serial.print( fusion.yaw() );
    Serial.print( " " );    
    Serial.print( fusion.pitch() );
    Serial.print( " " );
    Serial.print( fusion.roll() );
    Serial.println(); 
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
