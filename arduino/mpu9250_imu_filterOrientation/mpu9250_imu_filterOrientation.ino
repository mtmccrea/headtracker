/***************** *************************************************************

Michael McCrea
Jan 2017
mtm5@uw.edu

Reads Accelerometer, Gyro, and Magnetometer data from 
SparkFun 9DoF Razor IMU.
Applies Magnetometer correction data generated with MotionCal:
https://www.pjrc.com/store/prop_shield.html

Applies Mahony or Madgwick filters via libraries:
MahonyAHRS sketch from
https://github.com/PaulStoffregen/MahonyAHRS
or the MagdwickAHRS sketch from 
https://github.com/PaulStoffregen/MadgwickAHRS

TODO: 
- store MotionCal data in memory rather than hard-coded
- read in calibration data from MotionCal's Send Calibration feature,
  update on the fly and store the values
- change modes to output the data in the format expected from MotionCal
- add flag to swap between Madgwick and Mahony filters on the fly 
  (comfirm this is possible without additional CPU cost)
- add flag to send inverse rotations for e.g. ambisonic scene head tracking
- confirm the sensor/filter update rates match:
  https://learn.adafruit.com/ahrs-for-adafruits-9-dof-10-dof-breakout/sensor-fusion-algorithms?view=all#tuning-the-filter


Modified from the following:

SparkFun 9DoF Razor M0 Example Firmware
Jim Lindblom @ SparkFun Electronics
Original creation date: November 22, 2016
https://github.com/sparkfun/9DOF_Razor_IMU/Firmware

This example firmware for the SparkFun 9DoF Razor IMU M0 
demonstrates how to grab accelerometer, gyroscope, magnetometer,
and quaternion values from the MPU-9250's digital motion processor
(DMP). It prints those values to a serial port and, if a card is
present, an SD card.

Values printed can be configured using the serial port. Settings
can be modified using the included "config.h" file.

Resources:
SparkFun MPU9250-DMP Arduino Library:
  https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library
FlashStorage Arduino Library
  https://github.com/cmaglie/FlashStorage

Development environment specifics:
  Firmware developed using Arduino IDE 1.6.12

Hardware:
  SparkFun 9DoF Razor IMU M0 (SEN-14001)
  https://www.sparkfun.com/products/14001
******************************************************************************/
// MPU-9250 Digital Motion Processing (DMP) Library
#include <SparkFunMPU9250-DMP.h>
// config.h manages default logging parameters and can be used
// to adjust specific parameters of the IMU
#include "config.h"

#include <MahonyAHRS.h>
#include <MadgwickAHRS.h>

// Flash storage (for nv storage on ATSAMD21)
#ifdef ENABLE_NVRAM_STORAGE
#include <FlashStorage.h>
#endif

#define SerialPort SerialUSB

MPU9250_DMP imu; // Create an instance of the MPU9250_DMP class

// magnetic calibration values from MotionCal
// Offsets applied to raw x/y/z values
float mag_offsets[3]            = { -35.83F, -2.23F, -3.92F }; // TODO: see if this is the correct order

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { 1.027, 0.015, 0.007 },
                                    { 0.015, 0.985, 0.033 },
                                    { 0.007, 0.033, 0.990 } }; 

float mag_field_strength        = 34.10F;

// Mahony is lighter weight as a filter and should be used
// on slower systems
Mahony filter;
//Madgwick filter;



///////////////////////
// LED Blink Control //
///////////////////////
//bool ledState = false;
uint32_t lastBlink = 0;
void blinkLED()
{
  static bool ledState = false;
  digitalWrite(HW_LED_PIN, ledState);
  ledState = !ledState;
}

#ifdef ENABLE_NVRAM_STORAGE
  ///////////////////////////
  // Flash Storage Globals //
  ///////////////////////////
  // Logging parameters are all stored in non-volatile memory.
  // They should maintain the previously set config value.
  FlashStorage(flashEnableSDLogging, bool);
  FlashStorage(flashFirstRun, bool);
  FlashStorage(flashEnableSD, bool);
  FlashStorage(flashEnableSerialLogging, bool);
  FlashStorage(flashenableTime, bool);
  FlashStorage(flashEnableCalculatedValues, bool);
  FlashStorage(flashEnableAccel, bool);
  FlashStorage(flashEnableGyro, bool);
  FlashStorage(flashEnableCompass, bool);
  FlashStorage(flashEnableQuat, bool);
  FlashStorage(flashEnableEuler, bool);
  FlashStorage(flashEnableHeading, bool);
  FlashStorage(flashAccelFSR, unsigned short);
  FlashStorage(flashGyroFSR, unsigned short);
  FlashStorage(flashLogRate, unsigned short);
#endif


/////////////////////////////
// Logging Control Globals //
/////////////////////////////
// Defaults all set in config.h
bool enableSDLogging = ENABLE_SD_LOGGING;
bool enableSerialLogging =  ENABLE_UART_LOGGING;
bool enableTimeLog =        ENABLE_TIME_LOG;
bool enableCalculatedValues = ENABLE_CALCULATED_LOG;
bool enableAccel =          ENABLE_ACCEL_LOG;
bool enableGyro =           ENABLE_GYRO_LOG;
bool enableCompass =        ENABLE_MAG_LOG;
bool enableQuat =           ENABLE_QUAT_LOG;
bool enableEuler =          ENABLE_EULER_LOG;
bool enableHeading =        ENABLE_HEADING_LOG;
bool filterMag =            ENABLE_MAG_FILTER;
unsigned short accelFSR =   IMU_ACCEL_FSR;
unsigned short gyroFSR =    IMU_GYRO_FSR;
unsigned short fifoRate =   DMP_SAMPLE_RATE;

// sensor values
float ax; // accelerometer
float ay;
float az; 
float gx; // gyroscope
float gy;
float gz; 
float mx; // magnetometer
float my; 
float mz; 

bool setHome;
float hRoll;
float hPitch;
float hYaw;
float r;
float p;
float y;
bool sendInverse; // send coordinates that would be used for a rotation opposite that of the reading, e.g. ambisonic rotation for a headtracker


void setup()
{
  setHome = false;
  hRoll = hPitch = hYaw = 0.0;
  
  // Initialize LED, interrupt input, and serial port.
  // LED defaults to off:
  initHardware(); 
#ifdef ENABLE_NVRAM_STORAGE
  // Load previously-set logging parameters from nvram:
  initLoggingParams();
#endif

  // Initialize the MPU-9250. Should return true on success:
  if ( !initIMU() ) 
  {
    LOG_PORT.println("Error connecting to MPU-9250");
    while (1) ; // Loop forever if we fail to connect
    // LED will remain off in this state.
  }

  // Filter expects 50 samples per second
  filter.begin(50);
}

void loop()
{
  // The loop constantly checks for new serial input:
  if ( LOG_PORT.available() )
  {
    // If new input is available on serial port
    parseSerialInput(LOG_PORT.read()); // parse it
  }

  // Then check IMU for new data, and log it
  if ( !imu.fifoAvailable() ) // If no new data is available
    return;                   // return to the top of the loop

  // Read from the digital motion processor's FIFO
  if ( imu.dmpUpdateFifo() != INV_SUCCESS )
    return; // If that fails (uh, oh), return to top

  // If enabled, read from the compass.
  if ( (enableCompass || enableHeading) && (imu.updateCompass() != INV_SUCCESS) )
    return; // If compass read fails (uh, oh) return to top

  retrieveSensorData();
  
  if (filterMag) 
    filterSensorData();

  if (enableEuler)
    computeEuler();
  
  // If logging (to either UART and SD card) is enabled
  if ( enableSerialLogging || enableSDLogging)
    postIMUData(); // Log new data
  
}

void retrieveSensorData(void)
{
  if ( enableCalculatedValues ) // If in calculated mode
    { // for orientation from IMU
      ax = imu.calcAccel(imu.ax);
      ay = imu.calcAccel(imu.ay);
      az = imu.calcAccel(imu.az);
      gx = imu.calcGyro(imu.gx);
      gy = imu.calcGyro(imu.gy);
      gz = imu.calcGyro(imu.gz);
      mx = imu.calcMag(imu.mx);
      my = imu.calcMag(imu.my);
      mz = imu.calcMag(imu.mz);
    }
    else
    { // for orientation from filter T

      // TODO: confirm that the filter expects imu.calcX values
      ax = imu.ax;
      ay = imu.ay;
      az = imu.az;
      gx = imu.gx;
      gy = imu.gy;
      gz = imu.gz;
      mx = imu.mx;
      my = imu.my;
      mz = imu.mz;
    }
}

void filterSensorData(void)
{
    // Apply mag offset compensation (base values in uTesla)
  float x = mx - mag_offsets[0];
  float y = my - mag_offsets[1];
  float z = mz - mag_offsets[2];

  // Apply mag soft iron error compensation
  mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

//  // The filter library expects gyro data in degrees/s, but adafruit sensor
//  // uses rad/s so we need to convert them first (or adapt the filter lib
//  // where they are being converted)
//  float gx = gyro_event.gyro.x * 57.2958F;
//  float gy = gyro_event.gyro.y * 57.2958F;
//  float gz = gyro_event.gyro.z * 57.2958F;

  // Update the filter
  filter.update(gx, gy, gz,
                ax, ay, az,
                mx, my, mz);
}

void computeEuler(void)
{
   if (filterMag)
    {
      p = filter.getPitch();
      r = filter.getRoll();
      y = filter.getYaw();
    } else {
      imu.computeEulerAngles();
      p = imu.pitch;
      r = imu.roll;
      y = imu.yaw;
    }

  if ( setHome )
    {
      hRoll = r;
      hPitch = p;
      hYaw = y;
      r = p = y = 0.0;
      setHome = false;
    } else {
      r = r - hRoll;
      p = p - hPitch;
      y = y - hYaw;
    }
}

void postIMUData(void)
{ 
  String imuLog = ""; // Create a fresh line to log
 
  if (filterMag)
  {
    imuLog += "filtered: ";
  } else {
    imuLog += "nofilter: ";
  }
  
  if (enableTimeLog) // If time logging is enabled
  {
    imuLog += String(imu.time) + ", "; // Add time to log string
  }
  
  if (enableAccel) // If accelerometer logging is enabled
  {
     imuLog += String(ax) + ", ";
     imuLog += String(ay) + ", ";
     imuLog += String(az) + ", ";
  }
 
  if (enableGyro) // If gyroscope logging is enabled
  {
     imuLog += String(gx) + ", ";
     imuLog += String(gy) + ", ";
     imuLog += String(gz) + ", ";
  }
 
  if (enableCompass) // If magnetometer logging is enabled
  {
     imuLog += String(mx) + ", ";
     imuLog += String(my) + ", ";
     imuLog += String(mz) + ", ";
  }
 
  if (enableQuat) // If quaternion logging is enabled
  {
    if ( enableCalculatedValues )
    {
      imuLog += String(imu.calcQuat(imu.qw), 4) + ", ";
      imuLog += String(imu.calcQuat(imu.qx), 4) + ", ";
      imuLog += String(imu.calcQuat(imu.qy), 4) + ", ";
      imuLog += String(imu.calcQuat(imu.qz), 4) + ", ";
    }
    else
    {
      imuLog += String(imu.qw) + ", ";
      imuLog += String(imu.qx) + ", ";
      imuLog += String(imu.qy) + ", ";
      imuLog += String(imu.qz) + ", ";      
    }
  }

  if (enableEuler) // If Euler-angle logging is enabled
  {
    imuLog += String(p, 2) + ", ";
    imuLog += String(r, 2) + ", ";
    imuLog += String(y, 2) + ", ";
  }
 
  if (enableHeading) // If heading logging is enabled
  {
    imuLog += String(imu.computeCompassHeading(), 2) + ", ";
  }
  
  // Remove last comma/space:
  imuLog.remove(imuLog.length() - 2, 2);

  imuLog += "\r\n"; // Add a new line

  if (enableSerialLogging)  // If serial port logging is enabled
    LOG_PORT.print(imuLog); // Print log line to serial port

  // Blink LED once every second (if only logging to serial port)
  if ( millis() > lastBlink + UART_BLINK_RATE )
    {
      blinkLED(); 
      lastBlink = millis();
    }
}

void initHardware(void)
{
  // Set up LED pin (active-high, default to off)
  pinMode(HW_LED_PIN, OUTPUT);
  digitalWrite(HW_LED_PIN, LOW);

  // Set up MPU-9250 interrupt input (active-low)
  pinMode(MPU9250_INT_PIN, INPUT_PULLUP);

  // Set up serial log port
  LOG_PORT.begin(SERIAL_BAUD_RATE);
}

bool initIMU(void)
{
  // imu.begin() should return 0 on success. Will initialize
  // I2C bus, and reset MPU-9250 to defaults.
  if (imu.begin() != INV_SUCCESS)
    return false;

  // Set up MPU-9250 interrupt:
  imu.enableInterrupt(); // Enable interrupt output
  imu.setIntLevel(1);    // Set interrupt to active-low
  imu.setIntLatched(1);  // Latch interrupt output

  // Configure sensors:
  // Set gyro full-scale range: options are 250, 500, 1000, or 2000:
  imu.setGyroFSR(250); // filter expects 250
  // Set accel full-scale range: options are 2, 4, 8, or 16 g 
  imu.setAccelFSR(2); // filter expects 2
  // Set gyro/accel LPF: options are 5, 10, 20, 42, 98, 188 Hz
  imu.setLPF(188); 
  // Set gyro/accel sample rate: must be between 4-1000Hz
  // (note: this value will be overridden by the DMP sample rate)
  imu.setSampleRate(50); // filter expects 50
  // Set compass sample rate: between 4-100Hz
  imu.setCompassSampleRate(50); // filter expects 50

  // Configure digital motion processor. Use the FIFO to get
  // data from the DMP.
  unsigned short dmpFeatureMask = 0;
  if (ENABLE_GYRO_CALIBRATION)
  {
    // Gyro calibration re-calibrates the gyro after a set amount
    // of no motion detected
    dmpFeatureMask |= DMP_FEATURE_SEND_CAL_GYRO;
  }
  else
  {
    // Otherwise add raw gyro readings to the DMP
    dmpFeatureMask |= DMP_FEATURE_SEND_RAW_GYRO;
  }
  // Add accel and quaternion's to the DMP
  dmpFeatureMask |= DMP_FEATURE_SEND_RAW_ACCEL;
  dmpFeatureMask |= DMP_FEATURE_6X_LP_QUAT;

  // Initialize the DMP, and set the FIFO's update rate:
  imu.dmpBegin(dmpFeatureMask, fifoRate);

  return true; // Return success
}


// Parse serial input, take action if it's a valid character
void parseSerialInput(char c)
{
  unsigned short temp;
  switch (c)
  {
  case 'h': // Set home orientation
    setHome = true;
    break;
    
  case ENABLE_DATA_FILTER:
    filterMag = !filterMag;
    break;    
    
  case PAUSE_LOGGING: // Pause logging on SPACE
    enableSerialLogging = !enableSerialLogging;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableSerialLogging.write(enableSerialLogging);
#endif
    break;
  case ENABLE_TIME: // Enable time (milliseconds) logging
    enableTimeLog = !enableTimeLog;
#ifdef ENABLE_NVRAM_STORAGE
    flashenableTime.write(enableTimeLog);
#endif
    break;
  case ENABLE_ACCEL: // Enable/disable accelerometer logging
    enableAccel = !enableAccel;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableAccel.write(enableAccel);
#endif
    break;
  case ENABLE_GYRO: // Enable/disable gyroscope logging
    enableGyro = !enableGyro;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableGyro.write(enableGyro);
#endif
    break;
  case ENABLE_COMPASS: // Enable/disable magnetometer logging
    enableCompass = !enableCompass;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableCompass.write(enableCompass);
#endif
    break;
  case ENABLE_CALC: // Enable/disable calculated value logging
    enableCalculatedValues = !enableCalculatedValues;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableCalculatedValues.write(enableCalculatedValues);
#endif
    break;
  case ENABLE_QUAT: // Enable/disable quaternion logging
    enableQuat = !enableQuat;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableQuat.write(enableQuat);
#endif
    break;
  case ENABLE_EULER: // Enable/disable Euler angle (roll, pitch, yaw)
    enableEuler = !enableEuler;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableEuler.write(enableEuler);
#endif
    break;
  case ENABLE_HEADING: // Enable/disable heading output
    enableHeading = !enableHeading;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableHeading.write(enableHeading);
#endif
    break;
  case SET_LOG_RATE: // Increment the log rate from 1-100Hz (10Hz increments)
    temp = imu.dmpGetFifoRate(); // Get current FIFO rate
    if (temp == 1) // If it's 1Hz, set it to 10Hz
      temp = 10;
    else
      temp += 10; // Otherwise increment by 10
    if (temp > 100)  // If it's greater than 100Hz, reset to 1
      temp = 1;
    imu.dmpSetFifoRate(temp); // Send the new rate
    temp = imu.dmpGetFifoRate(); // Read the updated rate
#ifdef ENABLE_NVRAM_STORAGE
    flashLogRate.write(temp); // Store it in NVM and print new rate
#endif
    LOG_PORT.println("IMU rate set to " + String(temp) + " Hz");
    break;
  case SET_ACCEL_FSR: // Increment accelerometer full-scale range
    temp = imu.getAccelFSR();      // Get current FSR
    if (temp == 2) temp = 4;       // If it's 2, go to 4
    else if (temp == 4) temp = 8;  // If it's 4, go to 8
    else if (temp == 8) temp = 16; // If it's 8, go to 16
    else temp = 2;                 // Otherwise, default to 2
    imu.setAccelFSR(temp); // Set the new FSR
    temp = imu.getAccelFSR(); // Read it to make sure
#ifdef ENABLE_NVRAM_STORAGE
    flashAccelFSR.write(temp); // Update the NVM value, and print
#endif
    LOG_PORT.println("Accel FSR set to +/-" + String(temp) + " g");
    break;
  case SET_GYRO_FSR:// Increment gyroscope full-scale range
    temp = imu.getGyroFSR();           // Get the current FSR
    if (temp == 250) temp = 500;       // If it's 250, set to 500
    else if (temp == 500) temp = 1000; // If it's 500, set to 1000
    else if (temp == 1000) temp = 2000;// If it's 1000, set to 2000
    else temp = 250;                   // Otherwise, default to 250
    imu.setGyroFSR(temp); // Set the new FSR
    temp = imu.getGyroFSR(); // Read it to make sure
#ifdef ENABLE_NVRAM_STORAGE
    flashGyroFSR.write(temp); // Update the NVM value, and print
#endif
    LOG_PORT.println("Gyro FSR set to +/-" + String(temp) + " dps");
    break;
  case ENABLE_SD_LOGGING: // Enable/disable SD card logging
    enableSDLogging = !enableSDLogging;
#ifdef ENABLE_NVRAM_STORAGE
    flashEnableSDLogging.write(enableSDLogging);
#endif
    break;
  default: // If an invalid character, do nothing
    break;
  }
}

#ifdef ENABLE_NVRAM_STORAGE
  // Read from non-volatile memory to get logging parameters
  void initLoggingParams(void)
  {
    // Read from firstRun mem location, should default to 0 on program
    if (flashFirstRun.read() == 0) 
    {
      // If we've got a freshly programmed board, program all of the
      // nvm locations:
//      flashEnableSDLogging.write(enableSDLogging);
      flashEnableSerialLogging.write(enableSerialLogging);
      flashenableTime.write(enableTimeLog);
      flashEnableCalculatedValues.write(enableCalculatedValues);
      flashEnableAccel.write(enableAccel);
      flashEnableGyro.write(enableGyro);
      flashEnableCompass.write(enableCompass);
      flashEnableQuat.write(enableQuat);
      flashEnableEuler.write(enableEuler);
      flashEnableHeading.write(enableHeading);
      flashAccelFSR.write(accelFSR);
      flashGyroFSR.write(gyroFSR);
      flashLogRate.write(fifoRate);
      
      flashFirstRun.write(1); // Set the first-run boolean
    }
    else // If values have been previously set:
    {
      // Read from NVM and set the logging parameters:
//      enableSDLogging = flashEnableSDLogging.read();
      enableSerialLogging = flashEnableSerialLogging.read();
      enableTimeLog = flashenableTime.read();
      enableCalculatedValues = flashEnableCalculatedValues.read();
      enableAccel = flashEnableAccel.read();
      enableGyro = flashEnableGyro.read();
      enableCompass = flashEnableCompass.read();
      enableQuat = flashEnableQuat.read();
      enableEuler = flashEnableEuler.read();
      enableHeading = flashEnableHeading.read();
      accelFSR = flashAccelFSR.read();
      gyroFSR = flashGyroFSR.read();
      fifoRate = flashLogRate.read();
    }
  }
#endif
