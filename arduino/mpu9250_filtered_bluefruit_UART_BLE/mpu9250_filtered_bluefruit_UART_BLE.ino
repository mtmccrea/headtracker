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

/*** MPU-9250 Digital Motion Processing (DMP) Library *** ------------- */
#include <SparkFunMPU9250-DMP.h>
#include "config.h"

/*** Absolute orientation filtering *** ------------------------------- */
#include <MahonyAHRS.h>
#include <MadgwickAHRS.h>

/*** BLE *** ---------------------------------------------------------- */
#include <Arduino.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"


/*** Flash storage (for nv storage on ATSAMD21) *** ------------------- */
#ifdef ENABLE_NVRAM_STORAGE
#include <FlashStorage.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS (BLE)

      FACTORYRESET_ENABLE     Perform a factory reset when running this sketch
                              Factory reset will erase the non-volatile memory
                              where config data is stored, setting it back to
                              factory default values.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"


// Create the bluefruit object, using hardware serial, which does not need the RTS/CTS pins
Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN); // Serial1

// A small helper
void error(const __FlashStringHelper*err) {
  LOG_PORT.println(err);
  while (1);
}
// LED Blink Control
uint32_t lastBlink = 0;
void blinkLED()
{
  static bool ledState = false;
  digitalWrite(HW_LED_PIN, ledState);
  ledState = !ledState;
}


/*** Magnetometer / Absolute Orientation Filtering ***/
// Magnetometer Filtering: magnetic calibration values from MotionCal
// Offsets applied to raw x/y/z values
float mag_offsets[3] = { 32.52F, -7.28F, -1.71F }; // TODO: see if this is the correct order

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = {
  { 0.991, -0.007, -0.007 },
  { -0.007, 1.045, -0.012 },
  { 0.007, -0.012, 0.967 }
};
float mag_field_strength = 32.39F;

// Choose a filter method
// Mahony is lighter weight as a filter and should be used on slower systems
//Mahony filter;
 Madgwick filter;


/*** IMU ***/
MPU9250_DMP imu; // Create an instance of the MPU9250_DMP class


#ifdef ENABLE_NVRAM_STORAGE
///////////////////////////
// Flash Storage Globals //
///////////////////////////
// Logging parameters are all stored in non-volatile memory.
// They should maintain the previously set config value.
FlashStorage(flashEnableSDLogging, bool);
FlashStorage(flashFirstRun, bool);
//FlashStorage(flashEnableSD, bool);
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

// Define the size of each value in a data packet, in (hex) characters
const int HEX_LEN = 3;        // 3 for 4096 resolution, 4 for 65536 resolution
const int PACKET_SIZE = 3;    // how many values in each data "packet"

float RES_SCALE;
char formatSpecs[5];
const int valBufLen = HEX_LEN + 1;
char pbuf[valBufLen];
char rbuf[valBufLen];
char ybuf[valBufLen];

const int sndBufLen = HEX_LEN * PACKET_SIZE + 2; // '<123abc4de\0'='<',PACKET_SIZE*HEX_LEN,NULL char
char buffer[sndBufLen];

// sensor values
float ax, ay, az;             // accelerometer
float gx, gy, gz;             // gyroscope
float mx, my, mz;             // magnetometer

bool setHome;
float hRoll, hPitch, hYaw;
float r, p, y;                // rotate = yaw
// tilt = pitch
// tumble = roll
bool sendInverse;             // send coordinates that would be
// used for a rotation opposite
// that of the reading, e.g. ambisonic
// rotation for a headtracker


/* NOTE/TODO: 
 *  Kriw Winner (https://github.com/kriswiner/MPU-9250/issues/6): 
 *  I would use a 200 - 400 Hz acc/gyro sample rate, not 1.6 kHz. 
 *  You want the sensor fusion rate to be four or five times the data 
 *  sample rate since the fusion algorithm must iterate a few times 
 *  to reach a stable solution between sample data updates. If you 
 *  want to run the fusion filter at 1 kHz, a reasonable rate, you 
 *  would sample at 200 Hz using a 25 - 40 Hz bandwidth to filter 
 *  out high frequency noise. This is typically how I run the sensor 
 *  fusion with the MPU9250.
 */

void setup()
{
  RES_SCALE = (pow(16, HEX_LEN) - 1) / 360.0;
  snprintf (formatSpecs, 5, "%%0%iX", HEX_LEN);

  // -------------------- /***IMU Setup***/ --------------------
  setHome = false;
  hRoll = hPitch = hYaw = 0.0;

  // init hardware
  pinMode(HW_LED_PIN, OUTPUT);                // Set up LED pin (active-high, default to off)
  digitalWrite(HW_LED_PIN, LOW);
  pinMode(MPU9250_INT_PIN, INPUT_PULLUP);     // Set up MPU-9250 interrupt input (active-low)
  LOG_PORT.begin(SERIAL_BAUD_RATE);           // Set up serial log port

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

  filter.begin(IMU_AG_SAMPLE_RATE);

  initBLE();
}

/* ----------------------------------------------------------------- */
/* ------------------------------- LOOP ---------------------------- */
/* ----------------------------------------------------------------- */

void loop()
{
  // The loop constantly checks for new serial input (from Serial Monitor):
  if ( LOG_PORT.available() )
  {
    // If new input is available on USB serial port
    parseSerialInput(LOG_PORT.read());  // parse it
  }

  // Then check IMU for new data, and log it
  if ( !imu.fifoAvailable() )                   // If no new data is available
    return;                                     // return to the top of the loop

  // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
  if ( imu.dmpUpdateFifo() != INV_SUCCESS )
    return;                                     // If that fails (uh, oh), return to top

/* 
 * TODOOOOOOOOOOOOOOOOOOOO: why isn't the compass/magnetometer value updating ???
 */

  // If enabled, read from the compass.
  if ( imu.updateCompass() != INV_SUCCESS )
  {
    LOG_PORT.print("Compass ERROR!!");
    return;                                     // If compass read fails (uh, oh) return to top
  }

//  LOG_PORT.println("MAG-raw: " + String(imu.mx) + "  " + String(imu.my) + "  " + String(imu.mz));

  if (filterMag) filterSensorData();            // filter sensor data
  computeEuler();                               // computer Euler values for output in degrees
  sendHexToBLE();                               // send to BLE module
  //  sendIntsToBLE();

  while ( ble.available() )                     // Listen back from the BLE module
  {
    int c = ble.read();
    LOG_PORT.print((char)c);                    // Echo received data from BLE to Serial USB
  }

}

void filterSensorData(void)
{

  // Use the calcAccel, calcGyro, and calcMag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.
  ax = imu.calcAccel(imu.ax); // acceleration in g's
  ay = imu.calcAccel(imu.ay);
  az = imu.calcAccel(imu.az);
  gx = imu.calcGyro(imu.gx);  // rotation in dps
  gy = imu.calcGyro(imu.gy);
  gz = imu.calcGyro(imu.gz);
  mx = imu.calcMag(imu.mx);   // x-axis magnetic field in uT
  my = imu.calcMag(imu.my);
  mz = imu.calcMag(imu.mz);

 LOG_PORT.println("MAG-units: " + String(mx) + "  " + String(my) + "  " + String(mz));


  // Apply mag offset compensation (base values in uTesla)
  float x = mx - mag_offsets[0];
  float y = my - mag_offsets[1];
  float z = mz - mag_offsets[2];

  // Apply mag soft iron error compensation
  mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

//  // imu gyroscope returns uses degrees/second
//  // Mahony converts to radians/second internally
//  float gyroScale = 3.14159f / 180.0f;
//  gx = gx * gyroScale;
//  gy = gy * gyroScale;
//  gz = gz * gyroScale;


  /*
    * From Razor IMU hookup guide:
    * "Note that the magnetometer’s x and y axes are 
    * flipped from those of the accelerometer and gyroscope, 
    * and the z-axis is inverted as well."
  */

  mz *= -1.0;
//  az *= -1.0;
//  gz *= -1.0;
  
  // NOTE: filter uses 6-axis IMU algorithm if magnetometer measurement invalid
  // (avoids NaN in magnetometer normalisation)
  // - see filter source if needed, could post if this is the case
  // Update the filter
  filter.update(
    gx, gy, gz,
    ax, ay, az,
    my, mx, mz // note mx<>my - // should this happen after the mag compensation? before?? at all??
                                // should this be corrected in the sketch used to get mag cal from MotionCal?
  );
}


void computeEuler(void)
{
  if (filterMag)
  { // get euler from the filter
    LOG_PORT.println("Values from filter");
    p = filter.getPitch();
    r = filter.getRoll();
    y = filter.getYaw();
  } else {
    imu.computeEulerAngles(true);     // get euler from internal IMU
    p = imu.pitch;
    r = imu.roll;
    y = imu.yaw;
  }

  LOG_PORT.println("Y/P/R  " + String(y) + "  " + String(p) + "  " + String(r));
  
  // if setting a new home position...
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

  LOG_PORT.println("Y/P/R-H " + String(y) + "  " + String(p) + "  " + String(r));

  // deal with negative values, normalize, scale to n-digit hex range
  if (y < 0.0) y += 360.0;
  if (p < 0.0) p += 360.0;
  if (r < 0.0) r += 360.0;

  LOG_PORT.println("Y/P/R-M " + String(y) + "  " + String(p) + "  " + String(r));
}

void sendHexToBLE(void)
{
  //  if (enableEuler) // If Euler-angle logging is enabled
  //  {
  /* testing */
  //    // deg // normalized 0 to 4095
  //    p = 0.5; // 5.68
  //    r = 3.0; // 34
  //    y = 358; // 4073

  p *= RES_SCALE;
  r *= RES_SCALE;
  y *= RES_SCALE;

  snprintf (pbuf, valBufLen, formatSpecs, int(p + 0.5)); // formatSpecs: e.g. "%03X"
  snprintf (rbuf, valBufLen, formatSpecs, int(r + 0.5));
  snprintf (ybuf, valBufLen, formatSpecs, int(y + 0.5));
  // sent in format yaw, pitch, roll (rotate, tilt, tumble)
  snprintf(buffer, sndBufLen, "<%s%s%s", ybuf, pbuf, rbuf); // snprint won't exceed buffer size like sprintf
  // LOG_PORT.println(buffer);
  //  } else {
  //    // Handle other conditions, e.g. individual sensors only, quat only, etc...
  //  }

  // print char buffer to the BLE module
  ble.writeBLEUart(buffer); // we can assume we're in data (UART) mode
  //  ble.print(buffer);
}


void sendIntsToBLE(void)
{

  char buffer[16]; // '<360136013601>' // TODO: is eol char needed?
  char pbuf[5];
  char rbuf[5];
  char ybuf[5];

  if (enableEuler) // If Euler-angle logging is enabled
  {
    // deal with negative values
    // TODO: this could be more efficient to get into int before performing mod
    p = fmod(p, 360.0);
    r = fmod(r, 360.0);
    y = fmod(y, 360.0);

    // cap each float to 5 chars 3601\0
    snprintf (pbuf, 5, "%i", int(p * 10 + 0.5));
    snprintf (rbuf, 5, "%i", int(r * 10 + 0.5));
    snprintf (ybuf, 5, "%i", int(y * 10 + 0.5));
    snprintf(buffer, 16, "<%s%s%s>", ybuf, pbuf, rbuf); // send as ypr/rtt

  } else {
    // Handle other conditions, e.g. individual sensors only, quat only, etc...
  }

  // print char buffer to the BLE module
  //  ble.print(buffer);
  ble.writeBLEUart(buffer); // we can assume we're in data mode
}

// post IMU data to Serial port
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

  imuLog.remove(imuLog.length() - 2, 2);  // Remove last comma/space:
  imuLog += "\r\n";                       // Add a new line

  if (enableSerialLogging)  // If serial port logging is enabled
    LOG_PORT.print(imuLog); // Print log line to serial port

  // Blink LED once every second (if only logging to serial port)
  if ( millis() > lastBlink + UART_BLINK_RATE )
  {
    blinkLED();
    lastBlink = millis();
  }
}

bool initIMU(void)
{
  // imu.begin() should return 0 on success. Will initialize
  // I2C bus, and reset MPU-9250 to defaults.
  if (imu.begin() != INV_SUCCESS)
    return false;

  // Set up MPU-9250 interrupt:
  imu.enableInterrupt();            // Enable interrupt output
  imu.setIntLevel(1);               // Set interrupt to active-low
  imu.setIntLatched(1);             // Latch interrupt output

  /* Configure sensors */
  //  imu.setGyroFSR(IMU_GYRO_FSR);     // Set gyro full-scale range: filter expects 250
  if (filterMag) 
  {
    imu.setGyroFSR(250);              // Set gyro full-scale range: filter expects 250
  } else {
    imu.setGyroFSR(2000);             // Set gyro full-scale range: default 2000 for imu euler
  }
  imu.setAccelFSR(IMU_ACCEL_FSR);   // Set accel full-scale range: filter expects 2G
  imu.setLPF(IMU_AG_LPF);           // Set Accel/Gyro LPF

  // Set gyro/accel sample rate: must be between 4-1000Hz
  // (note: this value will be overridden by the DMP sample rate)
  imu.setSampleRate(IMU_AG_SAMPLE_RATE);

  // Set compass sample rate: between 4-100Hz
  imu.setCompassSampleRate(IMU_COMPASS_SAMPLE_RATE);

  // Configure digital motion processor.
  // Use the FIFO to get data from the DMP.
  unsigned short dmpFeatureMask = 0;
  if (ENABLE_GYRO_CALIBRATION)
  {
    // Gyro calibration re-calibrates the gyro
    // after a set amount of no motion detected
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

// -------------------- /***BLE Setup***/ --------------------
void initBLE(void)
{
  while (!LOG_PORT);  // required for Flora & Micro
  delay(500);

  /* Initialise the module */
  LOG_PORT.println(F("------------------------------------------------"));
  LOG_PORT.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  LOG_PORT.println( F("BLE initialized!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    LOG_PORT.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Command echo from Bluefruit */
  ble.echo(true);

  LOG_PORT.println("Requesting Bluefruit info:");
  ble.info();           // Print Bluefruit information
  ble.verbose(false);   // debug info suppressed

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  { // Change Mode LED Activity
    LOG_PORT.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  LOG_PORT.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  LOG_PORT.println(F("******************************"));
}


// Parse serial input, take action if it's a valid character
void parseSerialInput(char c)
{
  unsigned short temp;
  switch (c)
  {
    case 'h':                       // Set home orientation
      setHome = true;
      break;

    case ENABLE_DATA_FILTER:        // En/Disable magnetometer filter
      LOG_PORT.print("toggling filter: ");
      filterMag = !filterMag;
      if (filterMag)
      {
        imu.setGyroFSR(250);              // Set gyro full-scale range: filter expects 250
      } else {
        imu.setGyroFSR(2000);             // Set gyro full-scale range: default 2000 for imu euler
      }
      LOG_PORT.println(filterMag);
      break;

    case PAUSE_LOGGING:             // Pause logging on SPACE
      enableSerialLogging = !enableSerialLogging;
#ifdef ENABLE_NVRAM_STORAGE
      flashEnableSerialLogging.write(enableSerialLogging);
#endif
      break;

    case ENABLE_TIME:               // Enable time (milliseconds) logging
      enableTimeLog = !enableTimeLog;
#ifdef ENABLE_NVRAM_STORAGE
      flashenableTime.write(enableTimeLog);
#endif
      break;

    case ENABLE_ACCEL:              // Enable/disable accelerometer logging
      enableAccel = !enableAccel;
#ifdef ENABLE_NVRAM_STORAGE
      flashEnableAccel.write(enableAccel);
#endif
      break;

    case ENABLE_GYRO:               // Enable/disable gyroscope logging
      enableGyro = !enableGyro;
#ifdef ENABLE_NVRAM_STORAGE
      flashEnableGyro.write(enableGyro);
#endif
      break;

    case ENABLE_COMPASS:            // Enable/disable magnetometer logging
      enableCompass = !enableCompass;
#ifdef ENABLE_NVRAM_STORAGE
      flashEnableCompass.write(enableCompass);
#endif
      break;

    case ENABLE_CALC:               // Enable/disable calculated value logging
      enableCalculatedValues = !enableCalculatedValues;
#ifdef ENABLE_NVRAM_STORAGE
      flashEnableCalculatedValues.write(enableCalculatedValues);
#endif
      break;

    case ENABLE_QUAT:               // Enable/disable quaternion logging
      enableQuat = !enableQuat;
#ifdef ENABLE_NVRAM_STORAGE
      flashEnableQuat.write(enableQuat);
#endif
      break;

    case ENABLE_EULER:              // Enable/disable Euler angle (roll, pitch, yaw)
      enableEuler = !enableEuler;
#ifdef ENABLE_NVRAM_STORAGE
      flashEnableEuler.write(enableEuler);
#endif
      break;

    case ENABLE_HEADING:            // Enable/disable heading output
      enableHeading = !enableHeading;
#ifdef ENABLE_NVRAM_STORAGE
      flashEnableHeading.write(enableHeading);
#endif
      break;

    case INC_LOG_RATE:              // Increment the log rate from 1-60Hz (2Hz increments)
      // NOTE: This is for testing only, it doesn't update the filter
      // rate, which should match the DMP sample rate
      temp = imu.dmpGetFifoRate();  // Get current FIFO rate
      if (temp == 1)                // If it's 1Hz, set it to 2Hz
        temp = 2;
      else
        temp += 2;                  // Otherwise increment by 2
      if (temp > 60)                // If it's greater than 60Hz, reset to 1
        temp = 1;
      imu.dmpSetFifoRate(temp);     // Send the new rate
      temp = imu.dmpGetFifoRate();  // Read the updated rate
#ifdef ENABLE_NVRAM_STORAGE
      flashLogRate.write(temp);   // Store it in NVM and print new rate
#endif
      LOG_PORT.println("IMU rate set to " + String(temp) + " Hz");
      break;

    case DEC_LOG_RATE:              // Decrement the log rate from 1-60Hz (2Hz increments)
      // NOTE: This is for testing only, it doesn't update the filter
      // rate, which should match the DMP sample rate
      temp = imu.dmpGetFifoRate();
      if (temp == 2)
        temp = 60;
      else
        temp -= 2;
      if (temp < 1)
        temp = 60;
      if (temp > 60)
        temp = 1;
      imu.dmpSetFifoRate(temp);
      temp = imu.dmpGetFifoRate();
#ifdef ENABLE_NVRAM_STORAGE
      flashLogRate.write(temp);
#endif
      LOG_PORT.println("IMU rate set to " + String(temp) + " Hz");
      break;

    case SET_ACCEL_FSR:                 // Increment accelerometer full-scale range
      temp = imu.getAccelFSR();         // Get current FSR
      if (temp == 2) temp = 4;          // If it's 2, go to 4
      else if (temp == 4) temp = 8;     // If it's 4, go to 8
      else if (temp == 8) temp = 16;    // If it's 8, go to 16
      else temp = 2;                    // Otherwise, default to 2
      imu.setAccelFSR(temp);            // Set the new FSR
      temp = imu.getAccelFSR();         // Read it to make sure
#ifdef ENABLE_NVRAM_STORAGE
      flashAccelFSR.write(temp);     // Update the NVM value, and print
#endif
      LOG_PORT.println("Accel FSR set to +/-" + String(temp) + " g");
      break;

    case SET_GYRO_FSR:                    // Increment gyroscope full-scale range
      temp = imu.getGyroFSR();            // Get the current FSR
      if (temp == 250) temp = 500;        // If it's 250, set to 500
      else if (temp == 500) temp = 1000;  // If it's 500, set to 1000
      else if (temp == 1000) temp = 2000; // If it's 1000, set to 2000
      else temp = 250;                    // Otherwise, default to 250
      imu.setGyroFSR(temp); // Set the new FSR
      temp = imu.getGyroFSR(); // Read it to make sure
#ifdef ENABLE_NVRAM_STORAGE
      flashGyroFSR.write(temp); // Update the NVM value, and print
#endif
      LOG_PORT.println("Gyro FSR set to +/-" + String(temp) + " dps");
      break;

    default:                              // If an invalid character, do nothing
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
