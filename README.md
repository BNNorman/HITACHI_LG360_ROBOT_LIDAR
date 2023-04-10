
# BEWARE

I am still refining this - feel free to contribute since the packet serial protocol has been gleaned from two different sources which disagree on which two byte provide the laser range.

# HITACHI_LG_LDS360_ROBOT_LIDAR
An investigation of the Hitachi LDS360 Turtlebot Robot Lidar

![image](https://user-images.githubusercontent.com/15849181/229080401-91c47140-bc30-4980-9ec1-eadb665c3e99.png)


I thought it would be fun to play around with one of these. There are many different devices on AliExpress but I chose this one because the description said 'brand new' and the 'laser safety class 1' whereas others just indicated they used IR. 

https://www.aliexpress.com/item/1005004738189008.html?spm=a2g0o.order_list.order_list_main.5.18051802BNCZEC

The sensor transmits 60x42 byte packets one for every 6th degree of rotation. Each packet contains the distance data for the six angles and some extra fields which I have not found out about.

## wiring

|Motor|comments|
|-----|--------|
|Red|5v|
|black| connect to the sensor orange wire so that the motor starts when the sensor starts

|Sensor|comments|
|------|--------|
|Red|5v|
|black| Gnd|
|Green|RX|
|Brown|TX|
|Blue| ?|
|Orange|connect to motor Black|

The device transmits data at 230400 baud.

note: to start the sensor using a serial monitor press **b** (begin) the motor should start if wired as above. To stop the sensor press **e** (end).

## Serial Packet structure

The following information was obtained by reading code at https://github.com/synthiam/Behavior-Control-Hitachi-LG-LDS-Lidar. Some of the fields were not identified in that code and are marked with a ?.

| Byte | Comments |
|------|----------|
| 0    | 0xfa the start byte    |
| 1    | 0xa0+base angle (0..59)|
| 2    |?|
| 3    |?|
| 4    | motor rpm LSB |
| 5    | motor rpm MSB |
| 6   | intensity for base angle LSB      |
| 7   | intensity for base angle MSB    |
| 8   | base angle LSB of laser range |
| 9   | base angle MSB of laser range |
| 10|?|
| 11|?|
| 12| intensity for base angle +1  LSB  |
| 13| intensity for base angle +1 MSB  |
| 14| base angle +1 LSB of laser reading|
| 15| base angle +1 MSB of laser reading|
| 16| ?|
| 17 ?|
| 18|intensity for base angle +2  LSB  |
| 19|intensity for base angle +2  MSB  |
| 20 base angle +2 LSB of laser reading|
| 21 base angle +2 MSB of laser reading|
| 22| ?|
| 23| ?|
| 24| intensity for base angle +3  LSB |
| 25| intensity for base angle +3  MSB |
| 26| base angle +3 LSB of laser reading|
| 27| base angle +3 MSB of laser reading|
| 28| ?|
| 29| ?|
| 30 intensity for base angle +4  LSB |
| 31|intensity for base angle +4  MSB |
| 32 base angle +4 LSB of laser reading|
| 33| base angle +4 MSB of laser reading|
| 34| ?|
| 35| ?|
| 36| intensity for base angle +5  LSB|
| 37| intensity for base angle +5  MSB|
| 38| base angle +5 LSB of laser reading|
| 39| base angle +5 MSB of laser reading|
| 40| ?|
| 41| ?|


## Driver Software

HITACHI_LDS360_LIDAR.py defines the HITACHI_LDS360 class used to control the LIDAR.

Because of the high baud rate the driver uses two threads. One called **dataGatherer** is solely responsible for capturing the packets of data, maintaining sync with the data stream and storing the packets in a 60 element list. A second thread called **parser** scans the 60 captured packets and stores the distances in a 360 element list.

If you try to capture all 60 packets at once, 2520 bytes, I found that the risk of losing sync with the data stream increased. Regaining sync on 42 byte reads is faster.

Thread locking is used to prevent accessing the lists before they have been updated.

## API

from HITACHI_LDS360_LIDAR import HITACHI_LDS360

| code | comment |
|------|---------|
|lds360=HITACHI_LDS360(<port>)| header timeout defaults to 30s |
|lds360=HITACHI_LDS360(<port>,header_timeout) | set your own header timeout|
|lds360=HITACHI_LDS360(<port>,header_timeout,debug) | turn on a lot of debug messages|
|lds360.start()| opens the serial port,starts the motor, there will be a slight delay till the datastream begings (header_timeout)|
|lds360.stop()| switches off the motor and closes the serial port|
|lds360.dataIsAvailable()| True if the datastream has started - there can be a short delay|
|lds360.getAngleDist(angle)| return the distance of an object at the given angle|
|lds360.getAnglePoint(angle)| returns a tuple (x,y) for the point at the given angle|
|lds360.getAllAnglePoint()| returns a list of upto 360 of tuples (x,y) for the point at the given angle. Note that there may be less than 360 values if serial sync is lost but this should improve as the packets are picked up and sync is re-established.|



