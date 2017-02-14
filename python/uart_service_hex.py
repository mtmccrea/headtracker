# Example of interaction with a BLE UART device using a UART service
# implementation.
# Author: Tony DiCola
# Modification: Michael McCrea

import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import UART
import numpy as np
import re
import time
from threading import Timer
import atexit
import liblo, sys

# user-defined
OSC_URL  = '127.0.0.1'
OSC_PORT = 57120
OSC_PATH = "/imu"
runTime  = 20                           # testing - run for this long after connecting

HEX_SIZE = 3                            # number of hex digits expected in each value
PACKET_SIZE = 2                         # number of values in a data "packet"
PACKET_BYTES = HEX_SIZE * PACKET_SIZE   # pre-compute some scalars
BITDEPTH = pow(16, HEX_SIZE) - 1
TO_DEGREES = 360.0 / BITDEPTH

# Get the BLE provider for the current platform.
ble = Adafruit_BluefruitLE.get_provider()
readMore = True
concat = ''
initialised = False
packets = []

# set up and Address for OSC forwarding
# send all messages to port 57120 on the local machine
try:
    oscTarget = liblo.Address(OSC_URL,OSC_PORT)
except liblo.AddressError as err:
    print(err)
    sys.exit()

def readQueue(uart):
    global concat
    global readMore

    print "reading..."
    while readMore:
        # print '...'
        received = uart.read()
        # print 'received: {}'.format(received)
        if received is not None:
            print('\nreceived: {}'.format(received))
            concat = ''.join([concat, received])    # join recieved string with parsing buffer
            parseStr(1)                            # search the string for matches,

            # else:
            #     print "No match found :("

    print('done reading')

def initializeStr():
    global concat
    global initialised
    # find beginning of the most recent data cluster
    startFrom = concat.rfind('<', 0, len(concat))
    if startFrom == (len(concat)-1):
        concat = ''
        initialised = True
        print 'initialised'
    else:
        if (startFrom > -1):
            concat = concat[startFrom+1:] # '<' is stripped
            # print 'clipped: {}'.format(concat)
            initialised = True
            print 'initialised'
            parseStr(1)
        else:
            concat = '' # didn't finda a data start point, clear it out

def parseStr(passCnt):
    global concat
    global packets

    if not initialised:
        initializeStr()
    else:
        print 'processing: {}'.format(concat)

        foundBytes = len(concat)
        if foundBytes >= PACKET_BYTES:
            endAt = concat.find('<', 0, len(concat))    # look for beginning of another data packet
            if (endAt > -1):                            # found beginning of next data chunk
                if (concat[0] == '<'):
                    concat = concat[1:]                 # found leading '<', strip it, and
                    parseStr(1)                          # process the rest of the string via self, don't count it as a "pass"
                else:
                    found = concat[:endAt]                  # copy full packet from concat
                    print '\tfound: {} after: {}'.format(found, passCnt)
                    packets.append(found)                       # append found packet to result list
                    print '\t\t remaining: {}'.format(concat[endAt:])
                    processResult()                         # process result and dispatch

                    if endAt == (len(concat)-1):            # was '<' found at the end of the buffer?
                        concat = ''                         # clear it, ready for next data bundle
                    else:
                        concat = concat[endAt+1:]           # strip the '<'
                        parseStr(passCnt+1)                          # process the rest of the string via self

            else:                                       # didn't find beginning of another packet so it's all part of this one
                print 'found alone: {} after: {}'.format(concat, passCnt)
                packets.append(concat)                  # append found packet to result list
                processResult()                         # process result and dispatch
                concat = ''                             # clear the concat buffer

# def parseStr():
#     global concat
#     global packets
#
#     if not initialised:
#         initializeStr()
#     else:
#         print 'processing: {}'.format(concat)
#
#         # look for beginning of next data chunk
#         endAt = concat.find('<', 0, len(concat))
#         # NOTE: TODO: account for the fact that by virtue of the next chunk
#         # arriving now, the old data is already "behind" realtime!!!
#         # need to perhaps just find the beginning then count PACKET_BYTES characters
#         # and dispatch
#         if (endAt > -1):                            # found beginning of next data chunk
#             found = concat[:endAt]                  # copy full packet from concat
#             print 'found: {}'.format(found)
#             packets.append(found)                       # append found packet to result list
#             print '\t remaining: {}'.format(packets)
#             processResult()                         # process result and dispatch
#
#             if endAt == (len(concat)-1):            # was '<' found at the end of the buffer?
#                 concat = ''                         # clear it, ready for next data bundle
#             else:
#                 concat = concat[endAt+1:]           # strip the '<'
#                 parseStr()                          # process the rest of the string via self

def processResult():
    global packets
    # print packets

    if len(packets) > 0:
        for item in packets:
            isize = len(item)
            if (isize == PACKET_BYTES):              # confirm data packet is correct size
                # TODO: monitor if this condition is ever caught: if not remove this check
                if "<" not in item:
                    split = [item[x:x+HEX_SIZE] for x in range(0, len(item), HEX_SIZE)] # split into HEX_SIZE chunks
                    # print '\t\t split: {}'.format(split)
                    try:
                        ints = [int(val, 16) for val in split]    # convert hex to ints
                        result = [(val * TO_DEGREES) for val in ints] # convert normalized ints to degrees
                        dispatch(result)
                    except:
                        print "Error in splitting data: {}".format(split)

                else:
                    print "Error: result looks like more than one packet: {}".format(result)
                    initializeStr()
            else:
                print "Error: wrong packet size! {}".format(item)       # TODO: how to properly handle returning on error



        packets = []                                # reset found packets array

def dispatch(data):
    print '\t\t\t dispatching: {}'.format(data)
    msg = liblo.Message(OSC_PATH)           # create a message
    for val in data:
        msg.add(val)                        # ... append arguments
    liblo.send(oscTarget, msg)              # ... and then send it
    # print msg

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

    # Disconnect any currently connected UART devices.  Good for cleaning up and
    # starting from a fresh state.
    print('Disconnecting any connected UART devices...')
    UART.disconnect_devices()

    # Scan for UART devices.
    print('Searching for UART device...')

    try:
        adapter.start_scan()
        # # Search for the first UART device found (will time out after 60 seconds
        # # but you can specify an optional timeout_sec parameter to change it).
        # device = UART.find_device()

        test = True
        known_uarts = set()
        while test:
            # Call UART.find_devices to get a list of any UART devices that
            # have been found.  This call will quickly return results and does
            # not wait for devices to appear.
            found = set(UART.find_devices())
            # Check for new devices that haven't been seen yet and print out
            # their name and ID (MAC address on Linux, GUID on OSX).
            new = found - known_uarts
            for dev in new:
                print('Found UART: {0} [{1}]'.format(dev.name, dev.id))
                if (dev.name == 'Adafruit Bluefruit LE'):
                    device = dev
                    test = False
                    print('FOUND IT')

            known_uarts.update(new)
            # Sleep for a second and see if new devices have appeared.
            time.sleep(1.0)

        print 'found devices:'
        for dev in known_uarts:
            print dev.name

        if device is None:
            raise RuntimeError('Failed to find UART device!')
    finally:
        # Make sure scanning is stopped before exiting.
        adapter.stop_scan()

    print('Connecting to device...')
    device.connect()  # Will time out after 60 seconds, specify timeout_sec parameter
                      # to change the timeout.

    # Once connected do everything else in a try/finally to make sure the device
    # is disconnected when done.
    try:
        # Wait for service discovery to complete for the UART service.  Will
        # time out after 60 seconds (specify timeout_sec parameter to override).
        print('Discovering services...')
        UART.discover(device)
        print 'device name: {}'.format(device.name)

        # Once service discovery is complete create an instance of the service
        # and start interacting with it.
        uart = UART(device)

        # Write a string to the TX characteristic.
        uart.write('Hello world!\r\n')
        print("Sent 'Hello world!' to the device.")

        # Now wait up to one minute to receive data from the device.
        print('Waiting to receive data from the device...')

        # atexit.register(disconnect(device)) # register to cleanup even on cmd-c
        Timer(runTime, stopReading).start() # schedule to stop reading the data Queue in x seconds
        Timer(0, showTimer).start()
        print 'HERE'
        readQueue(uart) # start the inf loop reading from the data queue
                        # this thread holds while the read loop is running
        print 'HERE2'
        # received = uart.read(timeout_sec=20)
        # if received is not None:
        #     # Received data, print it out.
        #     print('Received: {0}'.format(received))
        # else:
        #     # Timeout waiting for data, None is returned.
        #     print('Received no data!')

        # ... once stopReading is called, this thread resumes, and wraps up
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

def stopReading():
    global readMore
    print 'stopping the read!'
    readMore = False

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
