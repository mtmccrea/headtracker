////////////////////////////////
// Default Logging Parameters //
////////////////////////////////
#define ENABLE_TIME_LOG       false
#define ENABLE_CALCULATED_LOG true      // filter expects calculated values
#define ENABLE_ACCEL_LOG      false
#define ENABLE_GYRO_LOG       false
#define ENABLE_MAG_LOG        false
#define ENABLE_QUAT_LOG       false
#define ENABLE_EULER_LOG      true
#define ENABLE_HEADING_LOG    false
#define ENABLE_MAG_FILTER     true


////////////////////////////////////////
// Enable Non-Volatile Memory Storage //
////////////////////////////////////////
#define ENABLE_NVRAM_STORAGE            // If defined, FlashStorage library must be installed


////////////////////////
// Serial Port Config //
////////////////////////
#define ENABLE_UART_LOGGING true
#define LOG_PORT SERIAL_PORT_USBVIRTUAL // Select the Serial port to log to.
                                        // Either SERIAL_PORT_USBVIRTUAL (SerialUSB)
                                        // or LOG_PORT SERIAL_PORT_HARDWARE (Serial1)
#define SERIAL_BAUD_RATE 115200         // Serial port baud


////////////////
// LED Config //
////////////////
#define HW_LED_PIN 13                   // LED attached to pin 13
#define UART_BLINK_RATE 1000            // Blink rate when only UART logging


/////////////////////////
// IMU Default Configs //
/////////////////////////
/* 
 *  Note: Some of these params can be overwritten using serial
 *  commands. These are just defaults on initial programming
 *  NOTE: TBD: the mahony/madgwick filter update rate should be set to the 
 *  rate that it's given samples (dmp sample rate?)
 *  but if the dmp is used for sensor fusion, it's being replaced
 *  by the filters, so higher rates could be used?
 *  test these limits (as well as accuracy, of course)
 */
#define DMP_SAMPLE_RATE    38         // Logging/DMP sample rate(4-200 Hz) This is the FIFO rate
#define IMU_COMPASS_SAMPLE_RATE 38    // Compass sample rate (4-100 Hz)
#define IMU_AG_SAMPLE_RATE 38         // Accel/gyro sample rate Must be between 4Hz and 1kHz
#define IMU_GYRO_FSR       250        // Gyro full-scale range (250, 500, 1000, or 2000), filter expects 250
#define IMU_ACCEL_FSR      2          // Accel full-scale range (2, 4, 8, or 16), filter expects 2
#define IMU_AG_LPF         42         // Accel/Gyro LPF corner frequency (5, 10, 20, 42, 98, or 188 Hz)
#define ENABLE_GYRO_CALIBRATION true


/////////////////////
// Serial Commands //
/////////////////////
#define PAUSE_LOGGING     ' ' // Space - Pause SD/UART logging
#define ENABLE_TIME       't' // Enable/disable time log (milliseconds)
#define ENABLE_ACCEL      'a' // Enable/disable accelerometer log (ax, ay, az)
#define ENABLE_GYRO       'g' // Enable/disable gyroscope log (gx, gy, gz)
#define ENABLE_COMPASS    'm' // Enable/disable magnetometer log (mx, my, mz)
#define ENABLE_CALC       'c' // Enable/disable calculated values
#define ENABLE_QUAT       'q' // Enable/disable quaternion logging (qw, qx, qy, qz)
#define ENABLE_EULER      'e' // Enable/disable estimated euler angles (roll, pitch, yaw)
#define ENABLE_HEADING    'H' // Enable/disable estimated heading logging
#define INC_LOG_RATE      '=' // Adjust logging rate from 1-200 Hz (in 10 Hz increments)
#define DEC_LOG_RATE      '-' // Adjust logging rate from 1-200 Hz (in 10 Hz increments)
#define SET_ACCEL_FSR     'A' // Set accelerometer FSR (2, 4, 8, 16g)
#define SET_GYRO_FSR      'G' // Set gyroscope FSR (250, 500, 1000, 2000 dps)
#define ENABLE_DATA_FILTER'f' // Enable/disable magnetometer filtering


//////////////////////////
// Hardware Definitions //
//////////////////////////
/* 
 * Danger - don't change unless using a different platform! 
 */
#define MPU9250_INT_PIN 4
#define MPU9250_INT_ACTIVE LOW