# Example of interaction with a BLE UART device using a CUSTOM service
# implementation. Modification of the UART service example by Tony DiCola.

# Note: this file requires custom_service.py  to be placed in the
# Adafruit_BluefruitLE in /services)
# also, in that same folder, the __init__.py file needs to have the line added:
#   from custom_service import CUSTOM # mtm added


import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import CUSTOM
import numpy as np
import re
import time
from threading import Timer
import atexit
import liblo, sys

# user-defined
OSC_URL  = '127.0.0.1'
OSC_PORT = 57120
# OSC_PATH = "/imu" # defined in custom_service.py in BLE library (/services)
runTime  = 30           # testing - run for this long after connecting

# Get the BLE provider for the current platform.
ble = Adafruit_BluefruitLE.get_provider()

# set up and Address for OSC forwarding
# send all messages to port 57120 on the local machine
try:
    oscTarget = liblo.Address(OSC_URL,OSC_PORT)
except liblo.AddressError as err:
    print(err)
    sys.exit()

# Main function implements the program logic so it can run in a background
# thread.  Most platforms require the main thread to handle GUI events and other
# asyncronous events like BLE actions.  All of the threading logic is taken care
# of automatically though and you just need to provide a main function that uses
# the BLE provider.
def main():
    global readMore
    # Clear any cached data because both bluez and CoreBluetooth have issues with
    # caching data and it going stale.
    ble.clear_cached_data()

    # Get the first available BLE network adapter and make sure it's powered on.
    adapter = ble.get_default_adapter()
    adapter.power_on()
    print('Using adapter: {0}'.format(adapter.name))

    # Disconnect any currently connected CUSTOM devices.  Good for cleaning up and
    # starting from a fresh state.
    print('Disconnecting any connected CUSTOM devices...')
    CUSTOM.disconnect_devices()

    # Scan for CUSTOM devices.
    print('Searching for CUSTOM device...')

    try:
        adapter.start_scan()
        # # Search for the first CUSTOM device found (will time out after 60 seconds
        # # but you can specify an optional timeout_sec parameter to change it).
        # device = CUSTOM.find_device()

        test = True
        KNOWN_customs = set()
        while test:
            # Call CUSTOM.find_devices to get a list of any CUSTOM devices that
            # have been found.  This call will quickly return results and does
            # not wait for devices to appear.
            found = set(CUSTOM.find_devices())
            # Check for new devices that haven't been seen yet and print out
            # their name and ID (MAC address on Linux, GUID on OSX).
            new = found - KNOWN_customs
            for dev in new:
                print('Found CUSTOM: {0} [{1}]'.format(dev.name, dev.id))
                if (dev.name == 'Adafruit Bluefruit LE'):
                    device = dev
                    test = False
                    print('FOUND IT')

            KNOWN_customs.update(new)
            # Sleep for a second and see if new devices have appeared.
            time.sleep(1.0)

        print 'found devices:'
        for dev in KNOWN_customs:
            print dev.name

        if device is None:
            raise RuntimeError('Failed to find CUSTOM device!')
    finally:
        # Make sure scanning is stopped before exiting.
        adapter.stop_scan()

    print('Connecting to device...')
    device.connect()  # Will time out after 60 seconds, specify timeout_sec parameter
                      # to change the timeout.

    # Once connected do everything else in a try/finally to make sure the device
    # is disconnected when done.
    try:
        # Wait for service discovery to complete for the CUSTOM service.  Will
        # time out after 60 seconds (specify timeout_sec parameter to override).
        print('Discovering services...')
        CUSTOM.discover(device)
        print 'device name: {}'.format(device.name)

        # Once service discovery is complete create an instance of the service
        # and start interacting with it.
        print('passing OSC target: {}'.format(oscTarget))
        custom = CUSTOM(device, oscTarget)
        print 'created instance of CUSTOM'

        # # Write a string to the TX characteristic.
        # initstr = "start!"
        # hex = [elem.encode("hex") for elem in initstr]
        # custom.write(hex)
        # print("Sent 'start!' to the device.")

        print('Waiting to receive data from the device...')
        # atexit.register(disconnect(device)) # register to cleanup even on cmd-c
        # Timer(runTime, stopReading).start() # schedule to stop reading the data Queue in x seconds
        Timer(0, showTimer).start()
        print 'HERE'

        time.sleep(runTime)

        print('stopping and waiting a second before disconnecting')
        time.sleep(1)
        print '1 and done'
    finally:
        disconnect(device)
        # Make sure device is disconnected on exit.
        # device.disconnect()

def disconnect(device):
    print 'disconnecting...'
    device.disconnect()

# def stopReading():
#     global readMore
#     print 'stopping the read!'
#     readMore = False

def showTimer():
    # for i in range(int(howLong)):
    for i in range(int(runTime)):
        print i
        time.sleep(1)

# Initialize the BLE system.  MUST be called before other BLE calls!
ble.initialize()

# Start the mainloop to process BLE events, and run the provided function in
# a background thread.  When the provided main function stops running, returns
# an integer status code, or throws an error the program will exit.
ble.run_mainloop_with(main)
