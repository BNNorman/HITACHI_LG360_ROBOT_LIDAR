
# FAULTY UNIT

It seems that the unit I bought from Aliexpress is faulty. There is not data output for the range 100-200 degrees and, when placing objects close to the LIDAR the distance readings are non-sensible (range too long).

I and trying to find a schematic, all my roads have dead ends, for this as it uses an STM32F205 mpu. It would be good to reprogram this with the Arduino IDE to find out if the fault is with the laser module (likely) or missed interrupts by the MCU - unlikely because it generates the correct serial data packets.



# HITACHI_LG_LDS360_ROBOT_LIDAR
An investigation of the Hitachi LDS360 Turtlebot Robot Lidar

![image](https://user-images.githubusercontent.com/15849181/229080401-91c47140-bc30-4980-9ec1-eadb665c3e99.png)


I thought it would be fun to play around with one of these. There are many different devices on AliExpress but I chose this one because the description said 'brand new' and the 'laser safety class 1' whereas others just indicated they used IR. 

https://www.aliexpress.com/item/1005004738189008.html?spm=a2g0o.order_list.order_list_main.5.18051802BNCZEC

The sensor transmits 60 x 42 byte packets one for every 6th degree of rotation. Each packet contains the distance data for the six angles in between.
## wiring

|Motor|comments|
|-----|--------|
|Red|5v|
|black| connect to the sensor orange wire so that the motor starts when the sensor starts. This is a pwm signal to control the motor speed

|Sensor|comments|
|------|--------|
|Red|5v|
|black| Gnd|
|Green|RX|
|Brown|TX|
|Blue| ?|
|Orange|connect to motor Black|

The device transmits serial data at 230400 baud.

note: to start the sensor using a serial monitor send **b** (begin) the motor should start if wired as above. To stop the sensor send **e** (end).

## Serial Packet structure

See LDS_BAsic_Specification.pdf


## Driver Software

HITACHI_LDS360_LIDAR.py defines the HITACHI_LDS360 class used to control the LIDAR.

Because of the high baud rate the driver uses two threads. One called **dataGatherer** which is solely responsible for capturing the packets of data, maintaining sync with the data stream and storing the packets in a 60 element list. A second thread called **parser** scans the 60 captured packets and stores the distances in a 360 element list.

If you try to capture all 60 packets at once, 2520 bytes, I found that the risk of losing sync with the data stream increased. Regaining sync on 42 byte reads is faster.

Thread locking is used to prevent accessing the stored lists whilst they are being updated.

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



