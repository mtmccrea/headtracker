/*********************************************************************
 * Connect to the BLE module and in Arudino's serial monitor, use AT 
 * commands to create a service with a characteristic:
 *    create a service
 *  AT+GATTADDSERVICE=UUID128=00-11-00-11-44-55-66-77-88-99-AA-BB-CC-DD-EE-FF
 *  
 *  create a characteristic: make sure bytes 3 and 4 don't conflict with 
 *    the service UUID 
 *    the following is set to notify (0x10), is 2 bytes long (for 16-bit ints), 
 *    Datatype is int (3)
 *  AT+GATTADDCHAR=UUID128=00-11-22-33-44-55-66-77-88-99-AA-BB-CC-DD-EE-FF,PROPERTIES=0x10,MIN_LEN=1,MAX_LEN=2,DATATYPE=3,VALUE=154
 *  
 *  this returns an ID of the characteristic, which will be needed to set it.
 *  
 *  Note this can be done directly from the micro controller, with something like this:
 *    ble.sendCommandCheckOK( F("AT+GATTADDSERVICE=uuid=0x1234") );
 *    ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2345,PROPERTIES=0x08,MIN_LEN=1,MAX_LEN=6,DATATYPE=string,DESCRIPTION=string,VALUE=abc"), &charid_string);
 *  the characteristic ID in this case is saved to 'charred_string'
 *  
 *  The catch with this approach is that the value is set to the BLE module via AT commands:
 *    char sndstr[19];
 *    char charval[5];
 *    snprintf (charval, sizeof(charval), "%i", sendval);
 *    strcpy (sndstr,"AT+GATTCHAR=1,");
 *    strcat (sndstr,charval);
 *    ble.sendCommandCheckOK( sndstr ); // set the value of the characteristic on the module
 *    
 *  This is essentially serial strings which need to be parsed by the processor on the BLE.  
 *  This seems to be a bottleneck when trying to set this value at even moderately high rates.
 */

/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerialUSB.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

/* This example demonstrates how to use Bluefruit callback API :
 * - setConnectCallback(), setDisconnectCallback(), setBleUartRxCallback(),
 * setBleGattRxCallback() are used to install callback function for specific
 * event. 
 * - Furthermore, update() must be called inside loop() for callback to
 * be executed.
 * 
 * The sketch will add an custom service with 2 writable characteristics,
 * and install callback to execute when there is an update from central device
 * - one hold string
 * - one hold a 4-byte integer
  */

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE     Perform a factory reset when running this sketch
   
                            Enabling this will put your Bluefruit LE module
                            in a 'known good' state and clear any config
                            data set in previous sketches or projects, so
                            running this at least once is a good idea.
   
                            When deploying your project, however, you will
                            want to disable factory reset by setting this
                            value to 0.  If you are making changes to your
                            Bluefruit LE device via AT commands, and those
                            changes aren't persisting across resets, this
                            is the reason why.  Factory reset will erase
                            the non-volatile memory where config data is
                            stored, setting it back to factory default
                            values.
       
                            Some sketches that require you to bond to a
                            central device (HID mouse, keyboard, etc.)
                            won't work at all with this feature enabled
                            since the factory reset will clear all of the
                            bonding data stored on the chip, meaning the
                            central device won't be able to reconnect.
                            
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features    
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE        0
    #define MINIMUM_FIRMWARE_VERSION   "0.7.0"
/*=========================================================================*/



// Create the bluefruit object, either software SerialUSB...uncomment these lines

//SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

//Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
//                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);


/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
 Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

int32_t charid_string;
int32_t charid_number;

float time = 0.0;
float fs = 0.0;

// A small helper
void error(const __FlashStringHelper*err) {
  SerialUSB.println(err);
  while (1);
}

void connected(void)
{
  SerialUSB.println( F("Connected") );
}

void disconnected(void)
{
  SerialUSB.println( F("Disconnected") );
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  SerialUSB.begin(115200);
  SerialUSB.println(F("Example writing a characteristic to a custom service"));
  SerialUSB.println(F("-------------------------------------"));

  /* Initialise the module */
  SerialUSB.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  SerialUSB.println( F("OK!") );

  // NO factory reset because we wrote the service to the BLE module and 
  // don't want it erased! Otherwise we'd have to re-write it:
  // (via serial monitor and FTDI USB connection:
  // at+gattaddservice=UUID128=00-11-00-11-44-55-66-77-88-99-AA-BB-CC-DD-EE-FF
  // // max_len=2 for a single 16-bit value (2 bytes), datatype=3 for int
  // at+gattaddchar=UUID128=00-11-22-33-44-55-66-77-88-99-AA-BB-CC-DD-EE-FF,PROPERTIES=0x10,MIN_LEN=1,MAX_LEN=2,DATATYPE=3,VALUE=154
  // power cycle the module to get the service to show up to central devices
  // note: maybe just need to call flush()
  
  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    SerialUSB.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }
  
  if ( !ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    error( F("Callback requires at least 0.7.0") );
  }

//  SerialUSB.println( F("Adding Service 0x1234 with 2 chars 0x2345 & 0x6789") );
//  ble.sendCommandCheckOK( F("AT+GATTADDSERVICE=uuid=0x1234") );
//  ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2345,PROPERTIES=0x08,MIN_LEN=1,MAX_LEN=6,DATATYPE=string,DESCRIPTION=string,VALUE=abc"), &charid_string);
//  ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x6789,PROPERTIES=0x08,MIN_LEN=4,MAX_LEN=4,DATATYPE=INTEGER,DESCRIPTION=number,VALUE=0"), &charid_number);

    // print the custom services to make sure yours is there
    ble.sendCommandCheckOK( F("AT+GATTLIST"));

//  ble.reset();

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  SerialUSB.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
  
  /* Set callbacks */
//  ble.setConnectCallback(connected);
//  ble.setDisconnectCallback(disconnected);
//  ble.setBleUartRxCallback(BleUartRX);
  
  /* Only one BLE GATT function should be set, it is possible to set it 
  multiple times for multiple Chars ID  */
//  ble.setBleGattRxCallback(charid_string, BleGattRX);
//  ble.setBleGattRxCallback(charid_number, BleGattRX);
}



/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{ 
  float ran = random(65536) / 65535.0 * 3600; // random value to test, degrees multiplied by factor of 10
  int sendval = int(ran); // sending a 16-bit int for now
  float thisTime;


// Some variables to caculate the fs of the loop generating the random numbers
//SerialUSB.print("time: ");
//  SerialUSB.println(time);
  thisTime = millis();
  fs = 1.0 / ((thisTime - time) / 1000.0);
  SerialUSB.print("fs: ");
  SerialUSB.println(fs);
  time = thisTime;
//  SerialUSB.print("thisTime: ");
//  SerialUSB.println(time);
  
  
  char sndstr[19];
  char charval[5];
  snprintf (charval, sizeof(charval), "%i", sendval);
  strcpy (sndstr,"AT+GATTCHAR=1,");
  strcat (sndstr,charval);
  
//  TO TRY: String sendstr = String(sendval, HEX);
  
//  ble.sendCommandCheckOK( sndstr ); // does this wait for OK? if not, may as well just send the atcommand directly
//  ble.atcommand_full( sndstr, NULL, 0, NULL, NULL ); // still waits for OK
  Serial1.println(sndstr); // send the AT command directly... 
                           // doesn't wait for response, etc, try to save overhead
                           // but does this create flow control problems?

  delay(5); // add throttle for now

//  ble.update(200);
}

