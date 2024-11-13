'''
HITACHI_LDS360.py

A library for driving the HITACHI_LG360 robot lidar bought from AliExpress

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

see https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver/blob/master/src/hlds_laser_publisher.cpp

A full 360 degree sweep consists of 60*42=2520 bytes which, in turn, consists of 60 blocks each of
42 bytes where each block contains the info for 6 degrees (60x6=360)

The first block representing zero degrees has a header 0xfa,0xa0. The header for the next block
is 0xfa,0xa1 which is 6 degrees then 0xfa,0xa2 which is 12 degrees and so on.

block format.

byte    value
0       0xfa
1       0xa0 + (6 * block number)   the angle index for this block, each step represents 6 degrees
2:3     rpm                         little endian, motor speed (rpm)

the following consists of 6 sets of 4 bytes each corresponding to base angle +0..5 degrees

at base angle + 0
4:5     intensity 1                 little endian laser intensity at base ang
6:7     dist1                       little endian dist at base angle divide by 1000 valid range is 120-3500 mm

at base angle +1
8:11    intensity 2                
12:13   dist2     
                  
at base angle +2
14:17   intensity 3                 
18:19   dist3 

at base angle +3
20:23   intensity 4
24:25   dist4                       

at base angle +4
26:29   intensity 5
30:31   dist5                      

at base angle +5
32:35  intensity 6
36:37  dist6


The data capture code runs in a seperate thread and is double buffered
The data convertor runs in a seperate thread

Access is controlled with thread locking

'''
import serial
import time
import threading
import math
import platform

SERIAL_PORT=None

if platform.system()=="Windows":
    SERIAL_PORT="COM3"

if platform.system()=="Linux":
    SERIAL_PORT="/dev/ttyUSB0"

if SERIAL_PORT is None:
    print("Unable to determine the serial port")
    exit(0)

DEBUG=True

PACKET_SIZE=42      # a byte array 60x42 byte entries
NUM_PACKETS=60      # each packet contains 6 degrees this gives 360 degrees
DATA_TIMEOUT=5
SYNC_TIMEOUT=10      # time out syncing to datastream (first byte of packet header s/b 0xFA)
THREAD_TIMEOUT=10.0 # timeout when threads are terminated
BAUD_RATE=230400    # do not change

# according to the spec the detection ranges are
MIN_RANGE=120
MAX_RANGE=3500

fa=bytearray([0xfa])
a0=bytearray([0xa0])
begin=b'b'
end=b'e'

class SerialException(Exception):
    """unable to connect to the serial port"""
class SerialTimeout(Exception):
    """Timeout waiting for serial data"""

class SyncTimeout(Exception):
    """Timeout waiting for sync to header"""

class ThreadStopTimeout(Exception):
    """Timeout waiting for thread to finish"""

class NotStarted(Exception):
    """Data gatherer thread has not been started"""

class HITACHI_LDS360:

    def __init__(self,port=SERIAL_PORT,baudrate=BAUD_RATE,debug=DEBUG):
        self.debug=debug
        self.lock = threading.RLock()   # to control access to raw_data
        self.port=port
        self.baudrate=baudrate
        self.conn=None                  # serial connection see start() and stop()

        self.dataGathererThread = threading.Thread(target=self.dataGatherer)

        self.dataAvailable=False # indicates raw_data for a full 360 has been captured
        self.raw_data=bytearray([0]*NUM_PACKETS*PACKET_SIZE)

    def __del__(self):
        # make sure serial port is freed
        self.stop()
        if self.conn is None:
            return
        self.conn.close()

    def isRunning(self):
        # if any of these threads are not alive
        # we should stop everything

        return self.dataGathererThread.is_alive()

    def getSerialData(self,size=1):
        if self.conn is None:
            return None

        if not self.isRunning():
            raise NotStarted

        start=time.time()
        while not self.conn.in_waiting:
            if (time.time()-start)>DATA_TIMEOUT:
                raise SerialTimeout

        try:
            return self.conn.read(size)
        except SerialException:
            raise

    def readNextPacket(self):
        """
        packets are 42 bytes long begin with 0xfa followed by 0xa0+seqNo then angle info

        seqNo is 0..59 (60 packets) representing 6 degree increments

        The received data is not processed in this thread but stored in an array for later.

        We sync to the first seqNo and grab all packets in one go

        :return: nothing
        """
        start=time.time()

        while not self.conn.in_waiting:
            if (time.time() - start) >= DATA_TIMEOUT:
                raise SerialTimeout

        # it's easier to wait for the start of a new scan because
        # the first two bytes will always be 0xfa oxa0

        inSync=False

        ch=bytearray([0])

        while not inSync:

            # look for leading byte of 0xfa
            while ch!=fa:
                if (time.time() - start) > SYNC_TIMEOUT:
                    raise SyncTimeout
                ch=self.getSerialData(1)

            # followed by 0xa0 (angle index zero)
            ch=self.getSerialData(1)

            if ch==a0:
                inSync=True

        data=self.getSerialData(PACKET_SIZE*NUM_PACKETS - 2)

        with self.lock:
            self.raw_data=fa+a0+data # retain divisibility re PACKET_SIZE
            self.dataAvailable=True

    def dataGatherer(self):
        """
        This is a thread loop it's sole job is to capture packets from the serial port

        It can be stopped by setting self.dataGathererThread.do_run=False

        :return: Nothing
        """
        if self.debug:
            print(f"Data gatherer starting thread Id {threading.get_ident()}")

        try:
            t = threading.current_thread()
            self.num_packets_read=0
            while getattr(t, "do_run", True):
                self.readNextPacket()
                self.num_packets_read+=1
                if self.num_packets_read==60:
                    self.dataAvailable=True
                self.num_packets_read=0

            if self.debug:
                print("Data gatherer thread exit")

        except Exception as e:
            print(f"Exception in dataGatherer: {e}")
            self.stop()

    def getAngleData(self,angle):
        angleIdx = angle // 6
        angle_offset = angle % 6

        packet_start=angleIdx*PACKET_SIZE
        #rdmv=memoryview(self.raw_data)
        with self.lock:
            packet=self.raw_data[packet_start:packet_start+PACKET_SIZE]

        packet_offset = angle_offset * 6 + 4

        # intensity and distance
        intensity=int.from_bytes(packet[packet_offset:packet_offset+1],"little")
        distance=int.from_bytes(packet[packet_offset + 2:packet_offset + 3],"little")
        return intensity,distance

    def start(self):
        """
        Sends a start command to the LIDAR. If wired correctly this
        should also start the motor and datastream

        The code waits till it see the header bytes 0xfa,0xa0 which
        correspond to the zero degree data block.

        The code throws away the remaining record

        Raises a HeaderTimeout exception if the header is not found (in time)

        :return: nothing
        """
        print("Start called")

        try:
            self.conn=serial.Serial(self.port,self.baudrate)
        except SerialException:
            raise

        self.conn.write(begin)  # starts the lidar
        if self.debug:
            print("waiting for LIDAR serial data")

        start=time.time()
        while self.conn.in_waiting==0:
            if (time.time()-start)>DATA_TIMEOUT:
                self.stop()
                raise SerialTimeout

        if self.debug:
            print("Starting data gatherer thread")

        self.dataGathererThread.do_run=True  # flag used to stop the threads nicely
        self.dataGathererThread.start()

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

        if threading.active_count()>1:
            if self.debug:
                print("Waiting for threads to stop")
                print(f"Thread active count={threading.active_count()}")

            self.dataGathererThread.do_run=False

            # wait till child threads have finished
            self.dataGathererThread.join(THREAD_TIMEOUT)

        # must be done last
        self.conn.close()
        self.conn=None

    def dataIsAvailable(self):
        # set True when the first packet is found
        return self.dataAvailable

    def getIntensitiesAndDistances(self):
        distance = [-1] * 360
        intensity = [-1] * 360
        for angle in range(360):
            intensity[angle],distance[angle]=self.getAngleData(angle)
        return intensity,distance

    def getDistancePoints(self):
        """
        Returns a list of 360 distance points

        :return: list of tuples (ang,x,y)
        """
        points=[]
        i, d = self.getIntensitiesAndDistances() # not using intensity value
        for angle in range(360):
            X=d[angle] * math.cos(math.radians(angle))
            Y=d[angle] * math.sin(math.radians(angle))
            points.append((X,Y))
        return points

    def getIntensityPoints(self):
        """
        Returns a list of 360 distance points

        :return: list of tuples (ang,x,y)
        """
        points=[]
        i, d = self.getIntensitiesAndDistances() # not using intensity value
        for angle in range(360):
            X=i[angle] * math.cos(math.radians(angle))
            Y=i[angle] * math.sin(math.radians(angle))
            points.append((X,Y))
        return points

if __name__=="__main__":
    # simple syntax check
    LDS=HITACHI_LDS360()
    LDS.start()

    while not LDS.dataIsAvailable():
        pass

    intensity,distance=LDS.getIntensitiesAndDistances()

    for ang in range (360):
        print(f"ang {ang} intensity {intensity[ang]} distance {distance[ang]}")

    LDS.stop()

