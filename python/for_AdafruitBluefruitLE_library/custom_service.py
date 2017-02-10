# MODIFICATION BY MTM, of Bluetooth LE UART service class
# to create a "custom" service, i.e. one in which you can
# define your own service and characteristics
# Also, in this same folder (Adafruit_BluefruitLE/services),
# the __init__.py file needs to have the line added:
#   from custom_service import CUSTOM


# Provides an easy to use interface to read
# and write data from a bluezle device that implements the UART service.
# Author: Tony DiCola
#
# Copyright (c) 2015 Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import Queue
import uuid

from .servicebase import ServiceBase

# OSC support
import liblo, sys

# user-defined
# OSC_URL  = '127.0.0.1'
# OSC_PORT = 57120
OSC_PATH = "/imu"


# Define service and characteristic UUIDs.
CUSTOM_SERVICE_UUID = uuid.UUID('00-11-00-11-44-55-66-77-88-99-AA-BB-CC-DD-EE-FF')
VAL_CHAR_UUID       = uuid.UUID('00-11-22-33-44-55-66-77-88-99-AA-BB-CC-DD-EE-FF')
BYTEARR_CHAR_UUID   = uuid.UUID('00-11-44-55-44-55-66-77-88-99-AA-BB-CC-DD-EE-FF')
OSC_dest = None

class CUSTOM(ServiceBase):
    """Modified: Bluetooth LE UART service object."""

    # Configure expected services and characteristics for the UART service.
    ADVERTISED = [CUSTOM_SERVICE_UUID]
    SERVICES = [CUSTOM_SERVICE_UUID]
    CHARACTERISTICS = [VAL_CHAR_UUID] #, BYTEARR_CHAR_UUID]
    # OSC_dest = None

    def __init__(self, device, oscTarget):
        global OSC_dest

        """Initialize UART from provided bluez device."""
        print('received oscTarget argument: {}'.format(oscTarget))
        OSC_dest = oscTarget

        print "initializing custom service. Finding service..."
        # Find the UART service and characteristics associated with the device.
        self._custom = device.find_service(CUSTOM_SERVICE_UUID)
        print self._custom # debug
        self._val = self._custom.find_characteristic(VAL_CHAR_UUID)
        # self._bytearr = self._custom.find_characteristic(BYTEARR_CHAR_UUID)

        # not using this...
        # Use a queue to pass data received from the RX property change back to
        # the main thread in a thread-safe way.
        # self._queue = Queue.Queue()

        # Subscribe to _val characteristic changes to receive data.
        self._val.start_notify(self._rx_received)
        # self._bytearr.start_notify(self._rx_received)

        print 'began notification'

    def _rx_received(self, data):
        global OSC_dest
        # Callback that's called when data is received on the RX characteristic.
        # Just throw the new data in the queue so the read function can access
        # it on the main thread.
        # data is a string of bytes of the changed
        # characteristic value.
        # print "(Callback) change recieved: {}".format(data)

        # manual conversion of the byte array... there must be a built in way???
        # expecting 2 bytes per value
        bytearr = []
        for i, item in enumerate(data):
            val = int(item.encode('hex'), 16) #NOTE: this may be unnecessary!
            bytearr.append(val)

        output = bytearr[1]*256 + bytearr[0]
        print output

        # forward the output via OSC
        if OSC_dest is not None:
            msg = liblo.Message(OSC_PATH)           # create a message
            msg.add(output)
            liblo.send(OSC_dest, msg)              # ... and then send it
        else:
            print('no OSC destination provided')

        # not using this...
        # self._queue.put(data)

    def write(self, data):
        """Write a string of data to the UART device."""
        # self._tx.write_value(data)

    def read(self, timeout_sec=None):
        """Block until data is available to read from the UART.  Will return a
        string of data that has been received.  Timeout_sec specifies how many
        seconds to wait for data to be available and will block forever if None
        (the default).  If the timeout is exceeded and no data is found then
        None is returned.
        """
        # try:
        #     return self._queue.get(timeout=timeout_sec)
        # except Queue.Empty:
        #     # Timeout exceeded, return None to signify no data received.
        #     return None
