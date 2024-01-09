#!/usr/bin/env python3

"""
#Install:
### install the last available release (stable)
sudo pip install pyModbusTCP
sudo pip3 install paho-mqtt

### install a specific version (here release v0.2.1) directly from github servers
sudo pip install git+https://github.com/sourceperl/pyModbusTCP.git@v0.2.1
"""

"""
Modbus/TCP basic gateway (RTU slave(s) attached)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

[pyModbusTCP server] -> [ModbusSerialWorker] -> [serial RTU devices]

Run this as root to listen on TCP privileged ports (<= 1024).

Open /dev/ttyUSB0 at 115200 bauds and relay it RTU messages to slave(s).
$ sudo ./server_serial_gw.py --baudrate 115200 /dev/ttyUSB0
"""

import os
import argparse
import logging
import struct
import binascii
import time
import random
import threading
from queue import Queue, Full, Empty
import asyncio
from pyModbusTCP.server import ModbusServer
from pyModbusTCP.utils import crc16
from pyModbusTCP.constants import EXP_GATEWAY_PATH_UNAVAILABLE, EXP_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND
# need sudo pip install pyserial==3.4
import serial
from paho.mqtt import client as mqtt_client
import json

FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60
_ASCII_HEADER = b":"
_ASCII_FOOTER = b"\r\n"

WorkerArray = []
WrkCount = 0
thrLock = threading.Lock()
MqTopic = ""
MqPub_Prop = None

def sleepEx(duration):
    startTime = time.perf_counter()
    while True:
        elapsedTime = time.perf_counter() - startTime
        remainingTime = duration - elapsedTime
        if remainingTime <= 0:
            break
        if remainingTime > 0.02:
            time.sleep(max(remainingTime/2, 0.01))  # Sleep for remaining Time or min. Sleep-Interval
        else:
            pass


# --------------------------------------------------------------------------- #
# Error Detection Functions
# --------------------------------------------------------------------------- #

def __generate_crc16_table():
    """Generate a crc16 lookup table.

    .. note:: This will only be generated once
    """
    result = []
    for byte in range(256):
        crc = 0x0000
        for _ in range(8):
            if (byte ^ crc) & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            byte >>= 1
        result.append(crc)
    return result


__crc16_table = __generate_crc16_table()


def computeCRC(data):  # pylint: disable=invalid-name
    """Compute a crc16 on the passed in string.

    For modbus, this is only used on the binary serial protocols (in this
    case RTU).

    The difference between modbus's crc16 and a normal crc16
    is that modbus starts the crc value out at 0xffff.

    :param data: The data to create a crc16 of
    :returns: The calculated CRC
    """
    crc = 0xFFFF
    for data_byte in data:
        idx = __crc16_table[(crc ^ int(data_byte)) & 0xFF]
        crc = ((crc >> 8) & 0xFF) ^ idx
    swapped = ((crc << 8) & 0xFF00) | ((crc >> 8) & 0x00FF)
    return swapped


def checkCRC(data, check):  # pylint: disable=invalid-name
    """Check if the data matches the passed in CRC.

    :param data: The data to create a crc16 of
    :param check: The CRC to validate
    :returns: True if matched, False otherwise
    """
    return computeCRC(data) == check


def computeLRC(data):  # pylint: disable=invalid-name
    """Use to compute the longitudinal redundancy check against a string.

    This is only used on the serial ASCII
    modbus protocol. A full description of this implementation
    can be found in appendix B of the serial line modbus description.

    :param data: The data to apply a lrc to
    :returns: The calculated LRC

    """
    lrc = sum(int(a) for a in data) & 0xFF
    lrc = (lrc ^ 0xFF) + 1
    return lrc & 0xFF


def checkLRC(data, check):  # pylint: disable=invalid-name
    """Check if the passed in data matches the LRC.

    :param data: The data to calculate
    :param check: The LRC to validate
    :returns: True if matched, False otherwise
    """
    return computeLRC(data) == check


class WorkerData:
    def __init__(self, Baudrate, Bytesize, Parity, Stopbits, FrameType):
        #public
        self.ix = 0
        self.MqQueue = None
        self.SerialName = ""
        self.serial_port = None
        self.inter_char_timeout: float = 0
        self.silent_interval: float = 0
        if Baudrate is None:
            self.baudrate = 19200
        elif Baudrate == "":
            self.baudrate = 19200
        else:
            self.baudrate = int(Baudrate)
        if Bytesize is None:
            self.bytesize = 8
        elif Bytesize == "":
            self.bytesize = 8
        else:
            self.bytesize = int(Bytesize)
        if Parity is None:
            self.parity = 'N'
        elif Parity == "":
            self.parity = 'N'
        elif Parity == "O":
            self.parity = 'O'
        elif Parity == "E":
            self.parity = 'E'
        else:
            self.parity = 'N'
        if Stopbits is None:
            self.stopbits = 1
        elif Stopbits == "":
            self.stopbits = 1
        else:
            self.stopbits = float(Stopbits)
        if FrameType is None:
            self.frametype = "RTU"
        elif FrameType == "":
            self.frametype = "RTU"
        elif FrameType == "ASCII":
            self.frametype = "ASCII"
        else:
            self.frametype = "RTU"
        self.serial_worker = None
        self.mqClient = None
        #ToDo:
        #    XonXoff, RtsCts
        self._t0 = float(1 + self.bytesize + self.stopbits) / self.baudrate
        # Check every 4 bytes / 2 registers if the reading is ready
        self._recv_interval = self._t0 * 4
        # Set a minimum of 1ms for high baudrates
        self._recv_interval = max(self._recv_interval, 0.001)

        if self.baudrate > 19200:
            self.silent_interval = 1.75 / 1000  # ms
        else:
            self.inter_char_timeout = 1.5 * self._t0
            self.silent_interval = 3.5 * self._t0
        self.silent_interval = round(self.silent_interval, 6)

    def end_of_frame(self, ByteCount):
        """Return how long to wait for data."""
        if ByteCount == 0:
            return self._t0 + (self._t0 * 3.5)
        return(self._t0 * ByteCount) + (self._t0 * 3.5)


# some class
class ModbusRTUFrame:
    """ Modbus RTU frame container class.

        [ Start Wait ] [Address ][ Function Code] [ Data ][ CRC ][  End Wait  ]
          3.5 chars     1b         1b               Nb      2b      3.5 chars

    Wait refers to the amount of time required to transmit at least x many
    characters.  In this case it is 3.5 characters.  Also, if we receive a
    wait of 1.5 characters at any point, we must trigger an error message.
    Also, it appears as though this message is little endian. The logic is
    simplified as the following::

        block-on-read:
            read until 3.5 delay
            check for errors
            decode

    The following table is a listing of the baud wait times for the specified
    baud rates::

        ------------------------------------------------------------------
         Baud  1.5c (18 bits)   3.5c (38 bits)
        ------------------------------------------------------------------
         1200   13333.3 us       31666.7 us
         4800    3333.3 us        7916.7 us
         9600    1666.7 us        3958.3 us
        19200     833.3 us        1979.2 us
        38400     416.7 us         989.6 us
        ------------------------------------------------------------------
        1 Byte = start + 8 bits + parity + stop = 11 bits
        (1/Baud)(bits) = delay seconds
    """

    def __init__(self, raw=b''):
        # public
        self.raw = raw

    @property
    def read_byte_start(self):
        """Return how many byte to read on start."""
        return 3

    @property
    def pdu(self):
        """Return PDU part of frame."""
        return self.raw[1:-2]

    @property
    def slave_address(self):
        """Return slave address part of frame."""
        return self.raw[0]

    @property
    def function_code(self):
        """Return function code part of frame."""
        return self.raw[1]

    @property
    def data_len(self):
        """Return reported byte-count part of frame
           or the predefined bytes by Function-Code.
        """
        if len(self.raw) < 3:
            return 0

        if self.raw[1] == 5 or self.raw[1] == 6 or self.raw[1] == 8 or self.raw[1] == 11 or self.raw[1] == 15 or self.raw[1] == 16:
            # 4 Bytes + CRC
            return 6
        elif self.raw[1] == 22:
            # 6 Bytes + CRC
            return 8
        elif self.raw[1] == 7:
            # 1 Byte + CRC
            return 3
        elif self.raw[1] == 24:
            # 2 Bytes + CRC
            dLen = self.raw[2]<<8 | self.raw[3]
            return dLen+2
        elif self.raw[1] > 127:
            # 1 Byte + CRC This is a Error-Code-Response
            return 3
        return self.raw[2]

    @property
    def to_Log(self):
        return bytes.hex(self.raw)

    @property
    def is_valid(self):
        """Check if frame is valid.

        :return: True if frame is valid
        :rtype: bool
        """
        return len(self.raw) > 4 and crc16(self.raw) == 0

    def reset(self):
        """Clear raw buffer."""
        self.raw = b''

    def build(self, raw_pdu, slave_ad):
        """Build a full modbus RTU message from PDU and slave address.

        :param raw_pdu: modbus as raw value
        :type raw_pdu: bytes
        :param slave_ad: address of the slave
        :type slave_ad: int
        """
        # [address] + PDU
        tmp_raw = struct.pack('B', slave_ad) + raw_pdu
        # [address] + PDU + [CRC 16]
        tmp_raw += struct.pack('<H', crc16(tmp_raw))
        self.raw = tmp_raw

    def buildFull(self, slave_ad, fc, adr, value):
        """Build a full modbus RTU message from slave address, Function Code, Address and Value.

        :param raw_pdu: modbus as raw value
        :type raw_pdu: bytes
        :param slave_ad: address of the slave
        :type slave_ad: int
        """
        #tmp_raw = '0x{:02x}{:02x}{:04x}{:04x}'.format(slave_ad, fc, adr, value)
        #B...unsigned byte,   H...unsigned short
        tmp_raw = struct.pack('>BBHH', slave_ad, fc, adr, value)
        # [address] + PDU + [CRC 16]
        tmp_raw += struct.pack('<H', crc16(tmp_raw))
        self.raw = tmp_raw

class ModbusASCIIFrame:
    """ Modbus ASCII frame container class.

        [ Start ][Address ][ Function ][ Data ][ LRC ][ End ]
          1c        2c         2c         Nc     2c      2c

        * start is ":"
        * data can be 0 - 2x252 chars
        * end is "\\r\\n" (Carriage return line feed), however the line feed
          character can be changed via a special command

        1 start bit
        7 data bits with the least significant bit sent first
        1 bit for parity completion
        1 stop bit

    This framer is used for serial transmission.  Unlike the RTU protocol,
    the data in this framer is transferred in plain text ascii.
    """

    def __init__(self):
        # public
        self.raw_rtu = b''
        self.raw = ""
        self.space_sep = False

    @property
    def to_Log(self):
        return self.raw

    @property
    def to_RTU(self):
        """Return Frame RTU encoded."""
        return self.raw_rtu

    @property
    def to_ASCII(self):
        """Return Frame ASCII encoded."""
        return self.raw

    @property
    def read_byte_start(self):
        """Return how many byte to read on start."""
        return 5

    @property
    def pdu(self):
        """Return PDU part of frame."""
        return self.raw[3:-4]

    @property
    def slave_address(self):
        """Return slave address part of frame."""
        return int(self.raw[1:2])

    @property
    def function_code(self):
        """Return function code part of frame."""
        return int(self.raw[3:4])

    @property
    def data_len(self):
        """Return reported byte-count part of frame
           or the predefined bytes by Function-Code.
        """
        if len(self.raw) < 6:
            return 0

        if self.raw[3:4] == "05" or self.raw[3:4] == "06" or self.raw[3:4] == "08" or self.raw[3:4] == "11" or self.raw[3:4] == "15" or self.raw[3:4] == "16":
            # 4 Bytes + CRC + CRLF
            return 8
        elif self.raw[3:4] == "22":
            # 6 Bytes + CRC + CRLF
            return 10
        elif self.raw[3:4] == "07":
            # 1 Byte + CRC + CRLF
            return 5
        elif self.raw[3:4] == "24":
            # 2 Bytes + CRC CRLF
            dLen = int(self.raw[5])<<8 | int(self.raw[6])
            return dLen+2
        elif int(self.raw[1]) > 127:
            # 1 Byte + CRC CRLF This is a Error-Code-Response
            return 5
        #ToDo:
        #   Check if returned Buffer-Count is 1 or 2 Byte
        return int(self.raw[5:6])

    @property
    def is_valid(self):
        """Check if frame is valid.

        :return: True if frame is valid
        :rtype: bool
        """
        if self.raw[-2] != "\r":
            return False
        if self.raw[-1] != "\n":
            return False
        lrc = computeLRC(self.raw[1:-4])
        return checkLRC(lrc, self.raw[-4:-3])

    def reset(self):
        """Clear raw buffer."""
        self.raw_rtu = b''
        self.raw = ""

    def buildFromASCII(self):
        if self.raw:
            data = self.raw.replace(" ", "")
            self.raw_rtu = binascii.a2b_hex(data[1:])

    def buildFromRTU(self):
        #BYTE_ORDER = ">" #big
        #FRAME_HEADER = "BBB"
        #buffer = struct.pack(">BBB", _ASCII_HEADER, slave_ad, fc)
        #checksum = computeLRC(encoded + buffer)

        #packet = bytearray()
        #packet.extend(":")
        #if self.space_sep:
            #packet.extend(f" {slave_ad:02x} {fc:02x} ".encode())
        #else:
            #packet.extend(f"{slave_ad:02x}{fc:02x}".encode())
        #if self.space_sep:
            #packet.extend(binascii.b2a_hex(encoded, " "))
        #else:
            #packet.extend(binascii.b2a_hex(encoded))
        #if self.space_sep:
            #packet.extend(f" {checksum:02x} ".encode())
        #else:
            #packet.extend(f"{checksum:02x}".encode())
        ##packet.extend(':')
        #packet.extend("\r\n")

        if self.raw_rtu:
            data = self.raw_rtu[:-2]
            tmpData = binascii.b2a_hex(data)
            checksum = computeLRC(tmpData)
            if self.space_sep:
                self.raw = binascii.b2a_hex(data, " ")
                self.raw += f" {checksum:02x} ".encode()
            else:
                self.raw = binascii.b2a_hex(data)
                self.raw += f"{checksum:02x}".encode()
            self.raw += "\r\n"


    def build(self, raw_pdu, slave_ad):
        """Build a full modbus ASCII message from PDU and slave address.

        :param raw_pdu: modbus as raw value
        :type raw_pdu: bytes
        :param slave_ad: address of the slave
        :type slave_ad: int
        """
        #ToDo:
        # [address] + PDU
        # h....Signed short,  H....Unsigned short
        #value = struct.unpack(">H", packed_bytes)[0]
        tmp_raw = ":" + slave_ad + raw_pdu
        # [address] + PDU + [CRC 16]
        tmp_raw += struct.pack('<H', crc16(tmp_raw))

    def buildFull(self, slave_ad, fc, adr, value):
        """Build a full modbus ASCII message from slave address, Function Code, Address and Value.

        :param raw_pdu: modbus as raw value
        :type raw_pdu: bytes
        :param slave_ad: address of the slave
        :type slave_ad: int
        """
        #tmp_raw = '0x{:02x}{:02x}{:04x}{:04x}'.format(slave_ad, fc, adr, value)
        #B...unsigned byte,   H...unsigned short
        tmp_raw = struct.pack('>BBHH', slave_ad, fc, adr, value)
        # [address] + PDU + [CRC 16]
        tmp_raw += struct.pack('<H', crc16(tmp_raw))


class RtuRequest:
    """ Request container to deal with modbus serial worker. """

    def __init__(self):
        self.request = ModbusRTUFrame()


class RtuQuery:
    """ Request container to deal with modbus serial worker. """

    def __init__(self):
        self.completed = threading.Event()
        self.request = ModbusRTUFrame()
        self.response = ModbusRTUFrame()


class ModbusSerialWorker:
    """ A serial worker to manage I/O with RTU devices. """

    def __init__(self, wrkData, timeout=1.0, waittime: float =0, end_of_frame=0.05):
        # public
        self.session = wrkData
        self.serial_port = wrkData.serial_port
        self.timeout = timeout
        self.end_of_frame = end_of_frame
        self.recvFrame = None
        self.asciiFrame = None
        if wrkData.frametype == "ASCII":
            self.recvFrame = ModbusASCIIFrame()
            self.asciiFrame = ModbusASCIIFrame()
        else:
            self.recvFrame = ModbusRTUFrame()
        FramePause_ms = wrkData.silent_interval * 1000
        if waittime > FramePause_ms:
            self.waittime = waittime
        else:
            self.waittime = FramePause_ms
        # internal request queue
        # accept 256 simultaneous requests before overloaded exception is return
        self.rtu_queries_q = wrkData.MqQueue

    def run(self):
        wrkLoop = threading.Thread(target=self.loop)
        wrkLoop.start()

    def loop(self):
        while self.serial_port is None:
            try:
                self.serial_port = open_serial(self.session.SerialName, Bauds=self.session.baudrate,
                         ByteSize=self.session.bytesize, StopBits=self.session.stopbits, Parity=self.session.parity)
            except serial.serialutil.SerialException as e:
                logger.critical('Serial device {0} error: {1}'.format(self.session.SerialName, e))
                sleep(5)

        tDelta = self.waittime
        tStart = time.perf_counter() - self.waittime
        """Serial worker main loop."""
        while True:
            try:
                # get next exchange from queue
                rtu_query = self.rtu_queries_q.get()
                if self.asciiFrame:
                    self.asciiFrame.raw_rtu = rtu_query.request.raw
                    self.asciiFrame.buildFromRTU()
                # send to serial
                if not self.serial_port.is_open:
                    #self.serial_port.close()
                    self.serial_port.open()
                self.serial_port.reset_input_buffer()
                # Check Serial Waittime before sending data
                tEnd = time.perf_counter()
                tDelta = tEnd - tStart
                logger.debug('Check Waittime: {0:f} >= {1:f}'.format(tDelta, self.waittime))
                if tDelta < self.waittime:
                    sleepEx((self.waittime-tDelta)/1000)
                tStart = time.perf_counter()
                if self.asciiFrame:
                    self.serial_port.write(self.asciiFrame.raw)
                    logger.debug('Serial write: %s', self.asciiFrame.to_Log)
                else:
                    self.serial_port.write(rtu_query.request.raw)
                    logger.debug('Serial write: %s', bytes.hex(rtu_query.request.raw))
                # receive from serial
                # wait for first byte of data until timeout delay
                self.recvFrame.reset()
                self.serial_port.timeout = self.timeout
                recvStart = time.perf_counter()
                rx_raw = self.serial_port.read(self.recvFrame.read_byte_start)
                # if ok, wait for the remaining
                if rx_raw is None:
                    pass
                elif len(rx_raw) < self.recvFrame.read_byte_start:
                    pass
                else:
                    self.recvFrame.raw = rx_raw
                    rx_raw = None
                    if self.recvFrame.function_code == 24:
                        readBytes = 1
                        if self.asciiFrame:
                            readBytes = 2
                        self.serial_port.timeout = self.session.end_of_frame(readBytes)
                        rx_chunk = self.serial_port.read(readBytes)
                        if rx_chunk:
                            self.recvFrame.raw += rx_chunk
                        else:
                            #This should not happen
                            if self.asciiFrame:
                                logger.warning('Serial recv-error: %s, add extra byte !', self.recvFrame.to_Log)
                                self.recvFrame.raw += "0"
                            else:
                                logger.warning('Serial recv-error: %s, add extra byte !', self.recvFrame.to_Log)
                                self.recvFrame.raw += b'0'
                    self.serial_port.timeout = self.session.end_of_frame(self.recvFrame.data_len)
                    # wait for next bytes of data until end of frame delay
                    while True:
                        rx_chunk = self.serial_port.read(self.recvFrame.data_len)
                        tStart = time.perf_counter()
                        if not rx_chunk:
                            break
                        else:
                            if rx_raw is None:
                                rx_raw = rx_chunk
                            else:
                                rx_raw += rx_chunk
                    if not rx_raw is None:
                        self.recvFrame.raw += rx_raw
                    if self.asciiFrame:
                        self.asciiFrame.buildFromASCII()
                    #logger.debug('Serial read: %s', bytes.hex(self.recvFrame.raw))
                    ##   Check Response SlaveID and FC and Address
                    ##   so we can check if Response is for this Request
                    ##   if not add a new Queue-Entry
                    ##rtu_query.request.slave_address, function_code, pdu => pdu[0]..FC, pdu[1]..Returned Bytes
                    if rtu_query.request.slave_address != self.recvFrame.slave_address:
                        ##ToDo:
                        ##   if mqClient => publish  rx_chunk
                        #rx_raw = None   #ToDo: Add rx_chunk to Receive-Queue as a new Event
                        logger.debug('Serial response from wrong slave: %s', self.recvFrame.to_Log)
                        pass
                    elif rtu_query.request.function_code != self.recvFrame.function_code:
                        if self.recvFrame.function_code > 79: # Error-Response is valid
                            pass
                        else:
                            ##ToDo:
                            ##   if mqClient => publish  rx_chunk
                            #rx_raw = None   #ToDo: Add rx_chunk to Receive-Queue as a new Event
                            logger.debug('Serial response with wrong FC: %s', self.recvFrame.to_Log)
                if isinstance(rtu_query, RtuQuery):
                    if self.asciiFrame:
                        rtu_query.response.raw = self.recvFrame.raw_rtu
                    else:
                        rtu_query.response.raw = self.recvFrame.raw
                    # mark all as done
                    rtu_query.completed.set()
                    logger.debug('Serial-TCP resp: %s', self.recvFrame.to_Log)
                else:
                    logger.debug('Serial-MQTT resp: %s', self.recvFrame.to_Log)
                    if self.recvFrame.is_valid:
                        #Publish MQTT-Event
                        payload = {'value': self.recvFrame.pdu, 'slave_id': self.recvFrame.slave_address, 'fc': self.recvFrame.function_code}
                        if WrkCount == 1:
                            mqtt_publish_msg(self.session.mqClient, f"{MqTopic}{self.session.ix}/{self.recvFrame.slave_address}/{self.recvFrame.function_code}", json.dumps(payload))
                        elif WrkCount > 1:
                            thrLock.acquire()
                            mqtt_publish_msg(self.session.mqClient, f"{MqTopic}{self.session.ix}/{self.recvFrame.slave_address}/{self.recvFrame.function_code}", json.dumps(payload))
                            thrLock.release()
                        else:
                            pass
                self.rtu_queries_q.task_done()
            except serial.serialutil.SerialException as e:
                logger.critical('Serial device {0} error: {1}'.format(self.session.SerialName, e))
                if isinstance(rtu_query, RtuQuery):
                    rtu_query.response.raw = b''
                    rtu_query.completed.set()
                self.rtu_queries_q.task_done()

    def srv_engine_entry(self, session_data):
        """Server engine entry point (pass request to serial worker queries queue).

        :param session_data: server session data
        :type session_data: ModbusServer.SessionData
        """
        # init a serial exchange from session data
        rtu_query = RtuQuery()
        rtu_query.request.build(raw_pdu=session_data.request.pdu.raw,
                                slave_ad=session_data.request.mbap.unit_id)
        logger.debug('TCP recv: %s', bytes.hex(session_data.request.pdu.raw))
        try:
            # add a request in the serial worker queue, can raise queue.Full
            self.rtu_queries_q.put(rtu_query, block=False)
            # wait result
            rtu_query.completed.wait()
            # check receive frame status
            if rtu_query.response.is_valid:
                logger.debug('TCP resp: %s', bytes.hex(rtu_query.response.pdu))
                session_data.response.pdu.raw = rtu_query.response.pdu
                return
            # except status for slave failed to respond
            exp_status = EXP_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND
        except Full:
            # except status for overloaded gateway
            exp_status = EXP_GATEWAY_PATH_UNAVAILABLE
        # return modbus exception
        func_code = rtu_query.request.function_code
        session_data.response.pdu.build_except(func_code=func_code, exp_status=exp_status)
        logger.debug('TCP resp with failure: %s', exp_status)


def open_serial(Port, Bauds=19200, ByteSize=8, StopBits=1, Parity='N'):
    logger.info('Open serial port {0} at {1:d} bauds'.format(Port, Bauds))
    # databits=WorkerArray[i].bytesize, parity=WorkerArray[i].parity
    serPort = serial.Serial(port=Port, baudrate=Bauds,
            bytesize=ByteSize, stopbits=StopBits, parity=Parity)
    return serPort


def mqtt_publish_msg(client, topic, msg):
    #MqPub_Prop = props.Properties(PacketTypes.PUBLISH)
    #MqPub_Prop.UserProperty = ("Content-Type", "application/json")
    #MqPub_Prop.UserProperty = ("content-encoding", "UTF-8")
    result = client.publish(topic, payload=msg, properties=MqPub_Prop)   #Defaults: qos=0, retain=False
    status = result[0]
    if status != 0:
        logger.info(f"Failed to send message to topic {topic}")


def mqtt_on_subscribe(client, userdata, mid, granted_qos, properties=None):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))

def mqtt_on_message(client, userdata, msg):
    print(f"Received `{msg.payload.decode('utf-8')}` from topic: `{msg.topic}`")
    #print("message qos=",msg.qos)
    #print("message retain flag=",msg.retain)
    msgParts = msg.topic.split("/")
    if len(msgParts) < 4:
        return
    #msgParts[1]    #DeviceID
    #msgParts[2]    #SlaveID
    #msgParts[3]    #FC
    DevID = int(msgParts[1])
    if DevID >= WrkCount or DevID < 0:
        logger.info("Skip message for unknown device : {0:d}".format(DevID))
        return
    jsonObj = json.loads(str(msg.payload.decode('utf-8')))
    Adr = jsonObj.get('Adr', None)  #jsonObj["Adr"]
    Value = jsonObj.get('Value', None)
    if Adr and Value:
        rtuFrame = ModbusRTUFrame()
        rtuFrame.buildFull(msgParts[2], msgParts[3], Adr, Value)
        WorkerArray[DevID].MqQueue.put(rtuFrame, block=False, timeout=None)


def connect_mqtt(broker, port, client_id, usr, pwd, proto, Keepalive=60):
    #Signature for MQTT v3.1 and v3.1.1 is:
    #def on_connect(client, userdata, flags, rc):
    #MQTT v5.0:
    def on_connect(client, userdata, flags, rc, properties=None):
        if rc == 0:
            print("Connected to MQTT Broker!")
            logger.info("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code {0:d}\n".format(rc))
            logger.error("Failed to connect to MQTT Broker, return code {0:d}\n".format(rc))
    mqProto = None
    if "v311" in proto:
        mqProto=mqtt_client.MQTTv311
    elif "v3" in proto:
        mqProto=mqtt_client.MQTTv31
    else:
        mqProto=mqtt_client.MQTTv5
    #client.connect_async(broker, port, keepalive=60, bind_address="")
    if "v3" in proto:
        MQclient = mqtt_client.Client(client_id=client_id, clean_session=True, protocol=mqProto, transport="tcp")
    else:
        from paho.mqtt.properties import Properties
        from paho.mqtt.packettypes import PacketTypes
        mqProp=Properties(PacketTypes.CONNECT)
        mqProp.SessionExpiryInterval=30*60*60 # in seconds
        #
        #User-Properties for publishing:
        global MqPub_Prop
        MqPub_Prop = Properties(PacketTypes.PUBLISH)
        MqPub_Prop.UserProperty = ("Content-Type", "application/json")
        MqPub_Prop.UserProperty = ("content-encoding", "UTF-8")
        #
        MQclient = mqtt_client.Client(client_id=client_id, clean_session=None, protocol=mqProto, transport="tcp")
    MQclient.on_message = mqtt_on_message
    if usr:
        MQclient.username_pw_set(usr, pwd)
    MQclient.on_connect = on_connect
    MQclient.on_disconnect = mqtt_on_disconnect
    MQclient.on_subscribe = mqtt_on_subscribe
    if "v3" in proto:
        MQclient.connect(broker, port)
    else:
        MQclient.connect(broker, port, clean_start=mqtt_client.MQTT_CLEAN_START_FIRST_ONLY, properties=mqProp, keepalive=Keepalive)
    return MQclient

def mqtt_on_disconnect(client, userdata, rc):
    logger.info("Disconnected with result code: {0:d}".format(rc))
    reconnect_count, reconnect_delay = 0, FIRST_RECONNECT_DELAY
    while reconnect_count < MAX_RECONNECT_COUNT:
        logger.info("Reconnecting in {0:d} seconds...".format(reconnect_delay))
        time.sleep(reconnect_delay)

        try:
            client.reconnect()
            logger.info("Reconnected successfully!")
            return
        except Exception as err:
            logger.error("%s. Reconnect failed. Retrying...", err)

        reconnect_delay *= RECONNECT_RATE
        reconnect_delay = min(reconnect_delay, MAX_RECONNECT_DELAY)
        reconnect_count += 1
    logger.info("Reconnect failed after {0:d} attempts. Exiting...".format(reconnect_count))


if __name__ == '__main__':
    # parse args
    parser = argparse.ArgumentParser()
    parser.add_argument('device', type=str, help='serial device (like /dev/ttyUSB0)')
    parser.add_argument('-L', '--listen', type=str, default='0.0.0.0', help='listen-address (default: 0.0.0.0)')
    parser.add_argument('-p', '--port', type=int, default=502, help='TCP port (default: 502)')
    parser.add_argument('-b', '--baudrate', type=str, default='19200', help='serial rate (default is 19200)')
    parser.add_argument('-c', '--bytesize', type=str, default='8', help='serial bytesize (default is 8)')
    parser.add_argument('-P', '--parity', type=str, default='N', help='serial parity N,O,E (default is N)')
    parser.add_argument('-f', '--stopbits', type=str, default='1', help='serial stopbits (default is 1)')
    parser.add_argument('-F', '--frametype', type=str, default='RTU', help='Frametype RTU, ASCII (default: RTU)')
    parser.add_argument('-t', '--timeout', type=float, default=1.0, help='timeout delay (default is 1.0 s)')
    parser.add_argument('-w', '--waittime', type=str, default="0", help='serial waittime delay (default is 0ms)')
    parser.add_argument('-e', '--eof', type=float, default=0.05, help='end of frame delay (default is 0.05 s)')
    parser.add_argument('-M', '--mqtthost', type=str, default=None, help='mqtthost (default: None)')
    parser.add_argument('-q', '--mqttport', type=int, default=1883, help='TCP mqtt-port (default: 1883)')
    parser.add_argument('-T', '--mqtttopic', type=str, default='modbus', help='Topic (default: modbus)')
    parser.add_argument('-i', '--mqttclientid', type=str, default='pymodbus_', help='Client-ID prefix (default: pymodbus_)')
    parser.add_argument('-u', '--mqttuser', type=str, default=None, help='mqtt Username (default: None)')
    parser.add_argument('-s', '--mqttpwd', type=str, default=None, help='mqtt Password (default: None)')
    parser.add_argument('-v', '--mqttversion', type=str, default='MQTTv5', help='mqtt Version MQTTv5, MQTTv311 (default: MQTTv5)')
    #MQTT - Certificate-Authentication is not implemented.
    #       for an Example e.g. see online:
    #                     https://www.emqx.com/en/blog/how-to-use-mqtt-in-python
    #                     https://pypi.org/project/paho-mqtt/
    parser.add_argument('-g', '--log', type=str, default='INFO', help='Loglevel (default: INFO)')
    parser.add_argument('-d', '--debug', action='store_true', help='set debug mode')
    args = parser.parse_args()
    args.device = os.environ.get('SERIAL_DEVICE', args.device)
    args.listen = os.environ.get('LISTEN_ADDRESS', args.listen)
    args.port = int(os.environ.get('PORT', args.port))
    args.baudrate = os.environ.get('SERIAL_BAUDRATE', args.baudrate)
    args.bytesize = os.environ.get('SERIAL_BYTESIZE', args.bytesize)
    args.parity = os.environ.get('SERIAL_PARITY', args.parity)
    args.stopbits = os.environ.get('SERIAL_STOPBITS', args.stopbits)
    args.frametype = os.environ.get('SERIAL_FRAMETYPE', args.frametype)
    args.timeout = float(os.environ.get('SERIAL_TIMEOUT', args.timeout))
    args.waittime = os.environ.get('SERIAL_WAITTIME', args.waittime)
    args.eof = os.environ.get('SERIAL_EOF', args.eof)
    args.mqtthost = os.environ.get('MQTT_HOST', args.mqtthost)
    args.mqttport = int(os.environ.get('MQTT_PORT', args.mqttport))
    args.mqtttopic = os.environ.get('MQTT_TOPIC', args.mqtttopic)
    args.mqttclientid = os.environ.get('MQTT_CLIENTID', args.mqttclientid)
    args.mqttuser = os.environ.get('MQTT_USER', args.mqttuser)
    args.mqttpwd = os.environ.get('MQTT_PWD', args.mqttpwd)
    args.mqttversion = os.environ.get('MQTT_VERSION', args.mqttversion)
    args.log = os.environ.get('LOG_LEVEL', args.log)
    args.debug = os.environ.get('DEBUG', args.debug)
    # init logging
    LogLevel = getattr(logging, args.log.upper(), None)
    if args.debug:
        LogLevel = logging.DEBUG
    logging.basicConfig(level=LogLevel)
    logger = logging.getLogger(__name__)
    # create console handler
    ch = logging.StreamHandler()
    ch.setLevel(LogLevel)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    ch.setFormatter(formatter)
    logger.addHandler(ch)
    #
    # Split Serial-Connections to an Array
    #
    SerialDevices = args.device.split(";")
    SerialBaudrates = args.baudrate.split(";")
    SerialBytesizes = args.bytesize.split(";")
    SerialParities = args.parity.split(";")
    SerialStopbits = args.stopbits.split(";")
    SerialFrameTypes = args.frametype.split(";")
    SerialWaitTimes = args.waittime.split(";")
    WrkCount = len(SerialDevices)
    baudRate = "19200"
    byteSize = "8"
    Parity = "N"
    stopBits = "1"
    frameType = "RTU"
    wTime = 0
    for i in range(len(SerialDevices)):
        if i < len(SerialBaudrates):
            baudRate = SerialBaudrates[i]
        else:
            baudRate = SerialBaudrates[0]
        if i < len(SerialBytesizes):
            byteSize = SerialBytesizes[i]
        else:
            byteSize = SerialBytesizes[0]
        if i < len(SerialParities):
            Parity = SerialParities[i]
        else:
            Parity = SerialParities[0]
        if i < len(SerialStopbits):
            stopBits = SerialStopbits[i]
        else:
            stopBits = SerialStopbits[0]
        if i < len(SerialFrameTypes):
            frameType = SerialFrameTypes[i]
        else:
            frameType = SerialFrameTypes[0]
        WorkerArray.append(WorkerData(baudRate, byteSize, Parity, stopBits, frameType))
        WorkerArray[i].ix = i
        WorkerArray[i].SerialName = SerialDevices[i]
    try:
        for i in range(len(SerialDevices)):
            WorkerArray[i].MqQueue = Queue(maxsize=256)
        # init serial port
        for i in range(len(SerialDevices)):
            # init serial worker
            if i < len(SerialWaitTimes):
                wTime = float(SerialWaitTimes[i])
            else:
                wTime = float(SerialWaitTimes[0])
            WorkerArray[i].serial_worker = ModbusSerialWorker(WorkerArray[i], args.timeout, wTime, args.eof)
            # start modbus server with custom engine
            logger.info('Start modbus server ({0}, {1:d} for serial device: {2}'.format(args.listen, args.port+i, SerialDevices[i]))
            WorkerArray[i].srv = ModbusServer(host=args.listen, port=args.port+i,
                           no_block=True, ext_engine=WorkerArray[i].serial_worker.srv_engine_entry)
        # MQTT Connect
        mqClient = None
        if args.mqtthost and args.mqtthost != "":
            MqTopic = args.mqtttopic
            if MqTopic[-1] != "/":
                MqTopic = MqTopic + "/"
            mqttClientID = f'{args.mqttclientid}-{random.randint(0, 1000)}'
            mqClient = connect_mqtt(args.mqtthost, args.mqttport, mqttClientID, args.mqttuser, args.mqttpwd, args.mqttversion, 60)
            logger.info('Connected to MQTT-Host ({0}, {1:d}) with client-id: {2}'.format(args.mqtthost, args.mqttport, mqttClientID))
            mqClient.loop_start()     # ToDo: implement   mqClient.loop_stop() for gracefull shutdown
            mqClient.subscribe(MqTopic, qos=0)

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        #evLoops = []
        for i in range(len(SerialDevices)):
            #evLoops.append(asyncio.new_event_loop())
            WorkerArray[i].mqClient = mqClient
            WorkerArray[i].srv.start()
            # start serial worker loop
            logger.debug('Start serial worker: ' + WorkerArray[i].SerialName)
            WorkerArray[i].serial_worker.run()
        logger.info("ModbusGW is online.")
        loop.run_forever()
        logger.info("ModbusGW is shutdown.")
    except ModbusServer.Error as e:
        logger.critical('Modbus server error: %r', e)
        logger.info("ModbusGW is shutdown.")
        sys.exit(1)
