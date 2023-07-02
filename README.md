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

Because of the high baud rate the driver uses a background thread which is solely responsible for capturing the raw packets of data. It synchronises to the first packet and reads all 60 in one go.

Thread locking is used to control accessing the stored raw_data.

## API

from HITACHI_LDS360_LIDAR import HITACHI_LDS360

| code | comment |
|------|---------|
|lds360=HITACHI_LDS360(port,baudrate=230400,debug=True)| serial port depends on platform |
|lds360.start()| opens the serial port,starts the motor, there will be a slight delay till the datastream begins (header_timeout)|
|lds360.stop()| switches off the motor , terminates the background task and closes the serial port|
|lds360.dataIsAvailable()| True if the raw data array has been filled with 60 packets of data|
|lds360.getAngleData(angle)| return the (intensity,distance) values at the given angle|
|lds360.getIntensitiesAndDistances()| returns a tuple of two arrays (intensity,distance)|
|lds360.getDistancePoints()| returns a list of 360 corresponding tuples (x,y) for available data points|
|lds360.getIntensityPoints()| returns a list of 360 corresponding tuples (x,y) for available data points|


## Plotter
The plotter program uses matplotlib pyplot to produce a scatter diagram of the distance points like this

![image](https://github.com/BNNorman/HITACHI_LG_LDS360_ROBOT_LIDAR/assets/15849181/e28e51a9-6854-47ee-90cf-edfc6e4a23fe)


