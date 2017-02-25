/************************************************************
  MTM EDIT. Starting from a simple template:

  MPU9250_DMP_Quaternion
  Quaternion example for MPU-9250 DMP Arduino Library
  Jim Lindblom @ SparkFun Electronics
  original creation date: November 23, 2016
  https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

  The MPU-9250's digital motion processor (DMP) can calculate
  four unit quaternions, which can be used to represent the
  rotation of an object.

  This exmaple demonstrates how to configure the DMP to
  calculate quaternions, and prints them out to the serial
  monitor. It also calculates pitch, roll, and yaw from those
  values.

  Development environment specifics:
  Arduino IDE 1.6.12
  SparkFun 9DoF Razor IMU M0

  Supported Platforms:
  - ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/
#include <SparkFunMPU9250-DMP.h>

/*** Absolute orientation filtering *** ------------------------------- */
#include <MahonyAHRS.h>
#include <MadgwickAHRS.h>

#define SerialPort SerialUSB

MPU9250_DMP imu;

// Choose a filter method
Mahony filter;              // lighter weight, for slower systems
// Madgwick filter;

const int UPDATE_RATE = 38;
bool USE_FILTER = true;

// sensor values
float accx, accy, accz;             // accelerometer
float gyrx, gyry, gyrz;             // gyroscope
float magx, magy, magz;             // magnetometer
float roll_out, pitch_out, yaw_out;

void setup()
{
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

  /* Configure sensors */
  if (USE_FILTER)
  {
    imu.setGyroFSR(250);              // Set gyro full-scale range: filter expects 250
  } else {
    imu.setGyroFSR(2000);             // Set gyro full-scale range: default 2000 for imu euler
  }

  imu.setAccelFSR(2);               // Set accel full-scale range: filter expects 2G, default 2 for imu euler
  imu.setLPF(98);                   // Set Accel/Gyro LPF
  imu.setSampleRate(UPDATE_RATE);
  imu.setCompassSampleRate(UPDATE_RATE);

  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat || TODO: if filter doesn't need these, don't enable it!
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
               UPDATE_RATE); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive

   filter.begin(UPDATE_RATE);
}

void loop()
{
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      if (USE_FILTER)
      {
        // OPTION 1: 9-dof sensor fusion using the chosen filter
        fuseFilter();           // updates the filter
      }
      else {
        // OPTION 2: g-dof sensor fusion using built-in DMP
        // computeEulerAngles can be used -- after updating the
        // quaternion values -- to estimate roll, pitch, and yaw
        fuseIMU();
      }

      printIMUData();
    } else {
        SerialPort.println("ERROR updating DMP in FIFO buffer");
    }
  }
}

void fuseIMU(void)
{
  imu.computeEulerAngles();
  roll_out = imu.roll;
  pitch_out = imu.pitch;
  yaw_out = imu.yaw;
}

void fuseFilter(void)
{
  // Update the compass
  if ( imu.updateCompass() != INV_SUCCESS )     // If compass read fails...
  {
    SerialPort.println("ERROR updating Compass");
    return;                                     // (uh, oh) return to top
  }

  // Use the calcAccel, calcGyro, and calcMag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.
  accx = imu.calcAccel(imu.ax);
  accy = imu.calcAccel(imu.ay);
  accz = imu.calcAccel(imu.az);
  gyrx = imu.calcGyro(imu.gx);
  gyry = imu.calcGyro(imu.gy);
  gyrz = imu.calcGyro(imu.gz);
  magx = imu.calcMag(imu.mx);
  magy = imu.calcMag(imu.my);
  magz = imu.calcMag(imu.mz);

  // NOTE: filter uses 6-axis IMU algorithm if magnetometer measurement invalid
  // (avoids NaN in magnetometer normalisation).
  // See filter source if needed, could post if this is the case.
  // Update the filter
  filter.update(gyrx, gyry, gyrz,
                accx, accy, accz,
                magx, magy, magz);

  roll_out = filter.getRoll();
  pitch_out = filter.getPitch();
  yaw_out = filter.getYaw();
  SerialPort.println(roll_out);
}

void printIMUData(void)
{
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  SerialPort.println("R/P/Y: " + String(roll_out) + ", "
                     + String(pitch_out) + ", " + String(yaw_out));
  // SerialPort.println("Time: " + String(imu.time) + " ms");
  SerialPort.println();
}
