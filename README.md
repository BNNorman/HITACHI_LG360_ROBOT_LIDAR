
# Update Dec 2024#

Re-written, hopefully, to improve the speed by using pyqtgraph, numpy, memoryviews and pyqtgraph.

Currently the display is fast enough to track an object moving in the room.

Tested on a RasPi 4 and a Dell Studio 1558 running Linuxmint.

# HITACHI_LG_LDS360_ROBOT_LIDAR

An investigation of the Hitachi LDS360 Turtlebot Robot Lidar

![image](https://user-images.githubusercontent.com/15849181/229080401-91c47140-bc30-4980-9ec1-eadb665c3e99.png)

I thought it would be fun to play around with one of these. There are many different devices on AliExpress but I chose this one because the description said 'brand new' and the 'laser safety class 1' whereas others just indicated they used IR. 

https://www.aliexpress.com/item/1005004738189008.html?spm=a2g0o.order_list.order_list_main.5.18051802BNCZEC

The sensor transmits, in one full rotation, a 60 x 42 byte stream of data. Each 42 byte packet contains the distance data for the six angles starting at the base angle for each packet. The base angle of each packet increments from 0 to 354. See HITACHI_LDS360_LIDAR.py for a more detailed view of the packets

I actually found the readings to be more stable by reading 60 consecutive packets rather than one packet at a time. It may be possible, using C, to read each packet individually without missing any but python on my RasPi4 and Dell Studio 1558 tended to miss some packets. 

## screenshot

This screenshot shows the device working in my rectangular living room. The pink markings are the distance readings whilst the green are the intensity readings (scaled down)

![IMG_20241215_163843](https://github.com/user-attachments/assets/d673b2e5-67af-4444-bbc9-c18ab2af9910)



## wiring

|Motor| comments                                                                                                                           |
|-----|------------------------------------------------------------------------------------------------------------------------------------|
|Red| 5v - link to sensor 5V                                                                                                             |
|black| connect to the sensor orange wire so that the motor starts when the sensor starts. This is a pwm signal to control the motor speed 

|Sensor| comments               |
|------|------------------------|
|Red| 5v link to Motor 5v    |
|black| Gnd                    |
|Green| RX                     |
|Brown| TX                     |
|Blue| ?                      |
|Orange| connect to motor Black |

The device transmits serial data at 230400 baud.

note: to start the sensor using a serial monitor send **b** (begin) the motor should start if wired as above. To stop the sensor send **e** (end).

## Serial Packet structure

See LDS_Basic_Specification.pdf


## Driver Software

HITACHI_LDS360_LIDAR.py defines the HITACHI_LDS360 class used to control the LIDAR.

Because of the high baud rate the driver uses a background thread which is solely responsible for capturing the packets of data. 

All packets begin with 0xFA. When the driver sees that it reads the next 50 packets in one pass. It then uses a callback to let the user know a packet has been read

After a complete pass the driver scans the packets, extracts the distance and intensity values then stores them in a lists of 360 values.

The Lidar runs at 300 rpm which means 1 revolution takes 60/300s (0.2s). The baud rate is 230400 which is approximately 230400/9 (25,600) bytes per second. A single pass of data would require 2520 bytes (60*42) and could be read in 2520/25,600 (0.1s). 

The driver has been re-written using numpy and memoryviews to try to make the data processing as fast as possible.

Considering that the lidar isn't expected to be moving at the speed of light the processing speed is adequate.

The driver inputs a full 60 consecutive 42byte packets then populates two lists: one for distance and the other for intensity. These lists have 360 entries, one per degree. When the 60 packets have been processed the drive uses a callback to notify the caller that new data is available. The caller can then process the distances however they like.

The PyQtGraphScatter.py program uses the data to update a scatter diagram once every 0.1s, if new data has been notified.

## API


NEEDS WORK

from HITACHI_LDS360_LIDAR import HITACHI_LDS360

| code | comment |
|------|---------|
|lds360=HITACHI_LDS360(port,baudrate=230400,debug=True)| serial port depends on platform |
|lds360.start()| opens the serial port,starts the motor, there will be a slight delay till the datastream begins (header_timeout)|
|lds360.stop()| switches off the motor , terminates the background task and closes the serial port|
|lds360.dataIsAvailable()| True if the raw data array has been filled with 60 packets of data|
|lds360.getAngleData(angle)| return the (intensity,distance) values at the given angle|
|lds360.getIntensitiesAndDistances()| returns a tuple of two arrays (intensity,distance). Each array has 360 entries|
|lds360.getDistances()| returns a list of 360 corresponding tuples (x,y) for available data points|
|lds360.getIntensities()| returns a list of 360 corresponding tuples (x,y) for available data points|


## PyQtGraphScatter.py
This program uses PyQtGraph to produce a scatter diagram of the distance points like the one below. This is a view of my livingroom (rectangular). The scatter diagram clearly shows where my stairs are located at the X=0 position.




