/************************************************************
Michael McCrea Jan 2017
mtm5@uw.edu

The following is a modification of:

MPU9250_DMP_Quaternion
 Quaternion example for MPU-9250 DMP Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

The MPU-9250's digital motion processor (DMP) can calculate
four unit quaternions, which can be used to represent the
rotation of an object.

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/
#include <SparkFunMPU9250-DMP.h>

#define SerialPort SerialUSB

MPU9250_DMP imu;
bool setHome;
float hRoll;
float hPitch;
float hYaw;
float r;
float p;
float y;

void setup() 
{
  setHome = false;
  hRoll = hPitch = hYaw = 0.0;
  
  // Set up MPU-9250 interrupt input (active-low)
  pinMode(4, INPUT_PULLUP); //MPU9250_INT_PIN

  // Set up serial log port
  SerialPort.begin(115200);

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  // Set gyro/accel LPF: options are5, 10, 20, 42, 98, 188 Hz
  imu.setLPF(188); 
  // Set gyro/accel sample rate: must be between 4-1000Hz
  // (note: this value will be overridden by the DMP sample rate)
  imu.setSampleRate(200); 
  // Set compass sample rate: between 4-100Hz
  imu.setCompassSampleRate(100); 
  
  imu.dmpBegin(
               // Enable 6-axis quat
               // DMP_FEATURE_LP_QUAT can also be used. It uses the 
               // accelerometer (3-axis) in low-power mode to estimate quat's.
               DMP_FEATURE_6X_LP_QUAT | 
               DMP_FEATURE_GYRO_CAL,      // Use gyro calibration, resets gyro after 8 sec of no motion
              180);                       // Set DMP FIFO rate to 200 Hz
  }

void loop() 
{

  // The loop constantly checks for new serial input:
  if ( SerialPort.available() )
  {
    // If new input is available on serial port
    parseSerialInput(SerialPort.read()); // parse it
  }
  
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();
      printIMUData();
    }
  }
}

void printIMUData(void)
{  
  r = imu.roll;
  p = imu.pitch;
  y = imu.yaw;

  if ( setHome )
  {
    hRoll = r;
    hPitch = p;
    hYaw = y;
    r = p = y = 0.0;
    setHome = false;
  } else {
    r = (r - hRoll) % 360;
    p = (p - hPitch) % 360;
    y = (y - hYaw) % 360;
  }
  
//  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
//  // are all updated.
//  // Quaternion values are, by default, stored in Q30 long
//  // format. calcQuat turns them into a float between -1 and 1
//  float q0 = imu.calcQuat(imu.qw);
//  float q1 = imu.calcQuat(imu.qx);
//  float q2 = imu.calcQuat(imu.qy);
//  float q3 = imu.calcQuat(imu.qz);
//
//  SerialPort.println("Q: " + String(q0, 4) + ", " +
//                    String(q1, 4) + ", " + String(q2, 4) + 
//                    ", " + String(q3, 4));
//  SerialPort.println("R/P/Y: " + String(imu.roll) + ", "
//            + String(imu.pitch) + ", " + String(imu.yaw));
//  SerialPort.println("Time: " + String(imu.time) + " ms");

  SerialPort.println("e" + String(r) + "," + String(p) + "," + String(y));
  SerialPort.println();
}

// Parse serial input, take action if it's a valid character
void parseSerialInput(char c)
{
  unsigned short temp;
  switch (c)
  {
  case 'h': // Pause logging on SPACE
    setHome = true;
    break;
  default: // If an invalid character, do nothing
    break;
  }
}

