'''
HITACHI_LDS360.py

A library for driving the HITACHI_LG360 robot lidar

Bought from AliExpress

See also https://github.com/synthiam/Behavior-Control-Hitachi-LG-LDS-Lidar

The sensor wiring:-

Motor
Red     5v
Black   connected to orange on the 6 pin connector

6 pin connector
Red     5v
Black   Gnd
Green   RX
Brown   TX
blue    not used
orange  -> motor gnd

Baud rate 230400

Commands

b'b'    Start,  also starts the motor if motor gnd is connected as above
b'e'    Stop,   also stops the motor if motor gnd is connected as above

Data format

see LDS_Basic_Specification.pdf


The data capture code runs in a seperate thread and captures the individual packets (42 bytes), extracts the angle and distance data which is cached till the user requests it.

'''
import serial
from serial import SerialException
import time
import threading
import math

SERIAL_PORT="COM3"
DEBUG=True

PACKET_SIZE=42      # a byte array 60x42 byte entries
NUM_PACKETS=60      # each packet contains 6 degrees this gives 360 degrees
HEADER_TIMEOUT=30   # time out syncing to datastream
THREAD_TIMEOUT=10.0 # timeout when threads are terminated
BAUD_RATE=230400    # do not change

# according to the spec the detection ranges are
MIN_RANGE = 120	 # mm
MAX_RANGE = 3500 # mm
DIST_SCALE =  10 # SPEC doesn't actually say what the units of the stored distance value are

fa = bytearray([0xfa])
a0=bytearray([0xa0])
begin=b'b'
end=b'e'

class SyncTimeout(Exception):
    """Timeout waiting for sync to header"""

class ThreadStopTimeout(Exception):
    """Timeout waiting for thread to finish"""

class HITACHI_LDS360:

    def __init__(self,port=SERIAL_PORT,baudrate=BAUD_RATE,header_timeout=HEADER_TIMEOUT,debug=DEBUG):
        self.header_timeout=header_timeout
        self.debug=debug
        self.lock = threading.RLock()
        self.port=port
        self.baudrate=baudrate
        self.conn=None              # serial connection see start() and stop()
        #self.running=False          # flag used to exit threads

        self.dataGathererThread = threading.Thread(target=self.dataGatherer)
        self.parserThread = threading.Thread(target=self.parser)

        # the data
        self.dataAvailable=False
        self.packet_list=[None]*NUM_PACKETS     # 60 packets of 6 degrees, each packet is a bytearray
        self.dist=[None]*360                    # parsed dist[ang]=value

    def __del__(self):
        # make sure serial port is freed
        self.stop()
        if self.conn is None:
            return
        self.conn.close()

    def isRunning(self):
        # if any of these threads are not alive
        # we should stop everything

        return self.dataGathererThread.is_alive() and self.parserThread.is_alive()


    def getSerialData(self,size=1):
        if self.conn is None:
            return None

        if not self.isRunning():
            return None

        try:
            return self.conn.read(size)
        except SerialException:
            raise

    def readNextPacket(self):
        """
        packets begin with 0xfa followed by 0xa0+seqNo

        seqNo is 0..59 (60 packets) representing 6 degree increments

        adds packet to packet_list[seqNo]

        :return: nothing
        """
        start=time.time()

        ch=b'0'

        self.packet=None

        while(time.time()-start)<self.header_timeout:

            if not self.isRunning():
                return

            # sync to 0xfa
            while ch!=fa:
                if (time.time() - start) > self.header_timeout:
                    raise SyncTimeout

                ch=self.getSerialData(1) # returns None if not running

                if ch is None or len(ch)==0:
                    return

            self.dataAvailable=True # so that calling thread can tell when to access data

            remainder=self.getSerialData(PACKET_SIZE-1) # returns None if not running
            if remainder is None or len(remainder)==0:
                return

            packet=ch+remainder

            seqNo=packet[1] - 0xa0

            # a valid packet has a sequence number 0..59
            if 0<seqNo<60:
                with self.lock:
                    self.packet_list[seqNo]=packet

    def crcCheck(self,packet):
        """
        sum all the bytes except the last two
        """
        crc=0
        for b in range(40):
            crc=crc+packet[b]
        crc =0xFF-crc
        if (crc & 0xFF != packet[40]) or (int(crc/256) != packet[41]):
           return False # CRC FAILED
        return True
    
    def processPacket(self,packet):
        """
        each packet (42 bytes) contains data for 6 angles
        
        See LDS_Basic_Specification for serial data format.
        
        The data consists of an intensity value and a range value e.g.
        
        packet[4:5] intensity (LSB,MSB)
        packet[6:7] distance (LSB,MSB)
        packet[8:9] reserved (LSB,MSB)
        
        populates self.dist[] with 360 values (eventually)

        :param packet:
        :return: Nothing, populates self.dist[] with 360 values
        """
        if packet is None:
            return

        degree=(packet[1]-0xa0)*6

        # I'm not checking the CRC as I want the code to
        # handle this processing quickly
        # if not crcCheck(packet):
        #    return
        
        # check if this packet looks correct
        # the start degree will be 0,6,12..354
        if 0<=degree<=354:
            # ok lets assume it's correct and grab the distance value
            # ignoring intensity value
            for ang in range(6):
                offset=ang*6
                self.dist[degree+ang]=packet[offset]+packet[offset+1]*256 # scaled later

    def parser(self):
        """
        There are 60 packets in the packet list

        :return: Nothing
        """
        if self.debug:
            print(f"parser starting thread Id {threading.get_ident()}")

        t = threading.current_thread()
        while getattr(t, "do_run", True):
            
            # There are 60 packets each 6 degrees apart 0,6,12,..348,354 which is the start angle
            # and containing the distances for 6 degrees from the start angle
            for pktNo in range(NUM_PACKETS):
                with self.lock:
                    self.processPacket(self.packet_list[pktNo])

        if self.debug:
            print("Parser thread exit")

    def dataGatherer(self):
        """
        This is a thread loop it's sole job is to capture packets from the serial port.

        It can be stopped by setting self.running=False in self.stop()

        :return: Nothing
        """
        if self.debug:
            print(f"Data gatherer starting thread Id {threading.get_ident()}")

        try:
            t = threading.current_thread()
            while getattr(t, "do_run", True):
                self.readNextPacket()

            if self.debug:
                print("Data gatherer thread exit")

        except Exception as e:
            print(f"EXCEPTION in dataGatherer: {e}")
            self.stop()

    def start(self):
        """
        Sends a start command to the LIDAR. If wired correctly this
        should also start the motor and datastream

        The code waits till it see the header bytes 0xfa,0xa0 which
        correspond to the zero degree data block.

        The code throws away the remaining record

        Raises a HeaderTimeout exception if the header is not found

        If a header is found starts the data gatherer in a seperate thread

        :return: nothing
        """
        print("Start called")

        try:
            self.conn=serial.Serial(self.port,self.baudrate)
        except SerialException:
            raise

        self.conn.write(begin)
        if self.debug:
            print("Threads starting")

        self.dataGathererThread.start()
        self.parserThread.start()
        # flags used to stop the threads
        self.dataGathererThread.do_run=True
        self.parserThread.do_run=True

    def stop(self):
        """
        terminate the data gatherer and turn off the LIDAR
        :return: nothing
        """
        print("Stop called")

        if self.conn is None:
            # LIDAR has not been started
            return

        self.conn.write(end)

        if self.debug:
            print("Waiting for threads to stop")
            print(f"Thread active count={threading.active_count()}")

        self.dataGathererThread.do_run=False
        self.parserThread.do_run=False

        # wait till child threads have finished
        # self.num_threads is the number of threads before
        # we start our own
        print("Waiting for parser thread to stop")
        self.parserThread.join(THREAD_TIMEOUT)
        self.dataGathererThread.join(THREAD_TIMEOUT)

        print(f"parserThread alive? {self.parserThread.is_alive()}")
        print(f"dataGatherThread alive? {self.dataGathererThread.is_alive()}")


        # must be done last
        self.conn.close()
        self.conn=None

    def dataIsAvailable(self):
        # set True when the first packet is found
        return self.dataAvailable

    def getAngleDist(self,angle):
        with self.lock:
            # according to github this is the value but what units (assume mm)?
            dist=self.dist[angle]
            if dist is None 
                return None # invalid/missing distance at this angle

            dist=dist/DIST_SCALE # should be mm
            
			dist<MIN_RANGE or dist>MAX_RANGE:
				return None
            return dist/

    def getAnglePoint(self,angle):
        dist=self.getAngleDist(angle)
        if dist is None:
            return (None,None)
        X=dist * math.cos(math.radians(angle))
        Y=dist * math.sin(math.radians(angle))
        return (X,Y)

    def getAnglePoints(self):
        """
        Returns a list of available points

        Sometimes the LIDAR might get out of sync and miss an angle
        so we return the angle associated with the coordinates
        in case the caller wants to store them in a list indexed by angle

        :return: list of tuples (ang,x,y)
        """
        points=[]
        for angle in range(360):
            x,y=self.getAnglePoint(angle)
            if (x,y)!=(None,None):
                points.append((ang,x,y))
        return points

    def getPoints(self):
        """
        Returns a list of available points

        This may be less than 360 degrees.

        :return: list of tuples (x,y)
        """
        points = []
        for angle in range(360):
            x, y = self.getAnglePoint(angle)
            if (x, y) != (None, None):
                points.append((x, y))
        return points

	def getAllAnglePoints(self):
        """
        Returns a list of available points with the angle so that
		averaging of X,Y at given angle can take place

        This may be less than 360 degrees.

        :return: list of tuples (angle,x,y)
        """
        points = []
        for angle in range(360):
            x, y = self.getAnglePoint(angle)
            if (x, y) != (None, None):
                points.append((angle,x, y))
        return points


if __name__=="__main__":
    # simple syntax check
    LG360=HITACHI_LG360()