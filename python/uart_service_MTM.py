# Example of interaction with a BLE UART device using a UART service
# implementation.
# Author: Tony DiCola
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
runTime  = 20           # testing - run for this long after connecting

# Get the BLE provider for the current platform.
ble = Adafruit_BluefruitLE.get_provider()
readMore = True
concat = ''
initialised = False
res = []

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
    global res

    print "reading..."
    while readMore:
        # print '...'
        received = uart.read()
        # print 'received: {}'.format(received)
        if received is not None:
            # print('got some: {}'.format(received))
            concat = ''.join([concat, received])

            # new parse edit
            processStr() # search the string for matches, pack them into res array

            if len(res) > 0:
                # print res
                for item in res:
                    if "><" not in item:
                        split = item.split(',')
                        # print 'split: {}'.format(split)
                        try:
                            result = np.array(map(float, split))
                        except:
                            print "Error in splitting data: {}".format(split)
                        dispatch(result)
                    else:
                        print "Error: result looks like more than one packet: {}".format(result)
                        initializeStr()

                res = [] # reset result array

            # result = processStr()
            #
            # if result is not None:
            #
            #     if "><" not in result:
            #         remainder = concat[len(result)+2:]
            #         split = result.split(',')
            #         # print 'split: {}, remainder: {}'.format(split, remainder)
            #         try:
            #             result = np.array(map(float, split))
            #         except:
            #             print "Error in splitting data: {}".format(split)
            #         # print 'result: {}, remainder: {}'.format(result, remainder)
            #         concat = remainder
            #         dispatch(result)
            #     else:
            #         print "result looks like more than one packet: {}".format(result)
            #         initializeStr()

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
            processStr()
        else:
            concat = '' # didn't finda a data start point, clear it out

def processStr():
    global concat
    global res

    if not initialised:
        initializeStr()
    else:
        # print 'processing: {}'.format(concat)
        # look for beginning of next data chunk
        endAt = concat.find('<', 0, len(concat))
        if (endAt > -1):
            # found beginning of next data chunk
            # make sure it isn't the first char
            if endAt > 1:
                # store found chunk
                found = concat[:endAt-1]
                # print 'found: {}'.format(found)
                res.append(found)
                # print 'remaining: {}'.format(res)

            if endAt == (len(concat)-1):
                concat = '' # we're at the end, strip the '<'
            else:
                # strip the '<' and process the rest of the string
                concat = concat[endAt+1:]
                processStr() # call self to check remaining matches

# def initializeStr():
#     global concat
#     global initialised
#     # find beginning of the most recent data cluster
#     startFrom = concat.rfind('<', 0, len(concat))
#     if startFrom == (len(concat)-1)
#
#     if (startFrom > -1):
#         concat = concat[startFrom:]
#         # print 'clipped: {}'.format(concat)
#         initialised = True
#         processStr()
#     else:
#         concat = '' # didn't finda a data start point, clear it out


# def processStr():
#     if not initialised:
#         initializeStr()
#     else:
#         found = re.search('<(.*)>', concat) # there's likely a faster way...
#         if found is not None:
#             # print('found a full match')
#             return found.group(1)
#         else:
#             return found




def dispatch(data):
    # print 'dispatching: {}'.format(data)
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
