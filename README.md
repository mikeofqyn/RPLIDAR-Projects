# Projects based on RPLIDAR A1 360º Laser Range Scanner

**Content**

1. **Software**
* Arduino versions
* Arduino sketches
* Project libraries
* RPLIDAR Product Libraries - RPLIDAR Driver Library
* Processing utilities (V 3.5.4)
2. **Hardware**
* RPLIDAR A1 (A1M8 MODEL) 
* MCP4725 12-bit DAC 
* Driver circuit
* Base station with servo 
* Notes on hardware setup
3. **Software usage notes and reference**

----
&nbsp;<p>


# 1. Software

## Arduino versions

The project have been compiled and tested with Arduino IDE 1.8.15 using the following cores:

| Board   | IDE board selection | Core version | Board manager |
| -------- | ------------- | ------------ | --------------|
| ESP801 or NodeMCU | ESP8266 boards (3.0.2)/Generic ESP8266 Module | 3.0.2 | http://arduino.esp8266.com/stable/package_esp8266com_index.json |
| ESP32             | ESP32 Arduino/ESP32 Dev Module | 1.0.6 | https://dl.espressif.com/dl/package_esp32_index.json |

## Arduino sketches

Download to: **&lt;ArduinoFolder&gt;\Projects\RPLIDAR\_Projects**

  
| Project directory | Date | Description |
|-------------------|-------|------------|
| RPLIDAR\_V3\_WIFI\_DRIVER | 05/21/2022 | Read RPLIDAR point data from through serial interface and send them via UDP broadcast. |
| RPLIDAR\_WIFI\_BASE\_SERVO | 05/29/2022 | Base station. Receives point data sent by the WiFi driver and keeps a table with the detected points (see library **RPLIDAR\_utils**). Also tries to identify points of interest (things that move) and directs a laser pointer towards the point using a servo motor. |
| CALIBTRATE\_SERVO | 05/26/2022 | Utility to calibrate a servo motor. Allows fine tuning of servo parameters and zero position. The zero degrees direction of the RPLIDAR must align with the 90 degrees direction of the servo. |
| RPLIDAR\_MCP4725\_Oscilloscope | 03/21/2022 | First attempt to interpret RPLIDAR A1 data. It reads directly from the serial port and tries to plot the data on an XY oscilloscope through 2 I2C DACs (MCP4725). It has been tested that and works, but, the result is not very useful. However it serves as an example of the use of RPLIDAR and 2 DACs of this type. |
| RPLIDAR\_WIFI\_BASE\_STATTION\_V2B | 05/03/2022 | Like RPLIDAR\_WIFI\_BASE\_SERVO but without the servo. The version with servo is derived from this one, in fact. |
  
## Project libraries

Download to: **&lt;ArduinoFolder&gt;\Projects\libraries\RPLIDAR\_utils**

|File | Date | Description |
|-------------------|-------|------------|
| ```RplidarData.h``` | 05/29/2022 | Data structures to hold, update and query the database of points detected by the RPLIDAR. Used by base station programs. |
| ```RPLIDAR_utils.h``` | 05/03/2022 | UDP connection between the driver and the base station(s). The driver communicates by broadcast address. The network is hardcoded in the program. I use a dedicated TP-LINK AP for testing, |
  

## RPLIDAR Product Libraries - RPLIDAR Driver Library
Download to: **&lt;ArduinoFolder&gt;\Projects\libraries\rplidar\_arduino**

**Source:**  [**https://**](https://github.com/robopeak/rplidar_arduino)[**github**](https://github.com/robopeak/rplidar_arduino)[**.com/robopeak/rplidar\_arduino**](https://github.com/robopeak/rplidar_arduino)

This library provides communications with the RPLIDAR through a serial object. Version used is 2. 


## MCP4725 12-bit DAC 
Download to: **&lt;ArduinoFolder&gt;\Projects\libraries\MCP4725**

** Source** https://github.com/RobTillaart/MCP4725

Rob Tillaart's library for the MCP4725 I2C DAC. Version tested is 0.3.3.


## Processing utilities (V 3.5.4)

Download to: **&lt;processing folder&gt;\Projects\RPLIDAR\_Projects**

Utilities to visualize the points detected by the RPLIDAR. See **RPLIDAR_RADAR.png**

**WARNING:** These programs use a data structure like the one in **RplidarData.h** (the latter is actually derived from these java programs), but they are not necessarily updated with every version, so the algorithms may differ. Be especially careful with the UDP packet structure.

In all of them, use &#39;h&#39; or &#39;?&#39; for help on options (clicking before on the graphic display).

The colors of the dots indicate the number of times they have been seen (the RPLIDAR does not detect all the dots on every turn).

  
| Project | Date | Comments |
| --- | --- | --- |
| RPLIDAR\_SERIAL\_PLOT\_POI | 05/06/2022 | Connect to a base station through a serial port to display the points of interest (POI) detected by the RPLIDAR\_data library algorithm. The code must be updated with the COM port assigned to the USB adapter. |
| RPLIDAR\_UDP\_ANIMATED | 05/22/2022 | Visualize data points with animation |
| RPLIDAR\_UDP\_RADAR | 05/22/2022 | Radar display, just for fun |
| RPLIDAR\_UDP\_RADAR\_INPROGRESS | 04/17/2022 | Radar display, with modifications |
| RPLIDAR\_UDP\_RADAR\_INPROGRESS\_2 | 05/22/2022 | Display showing moving dots (in blue) |
| RPLIDAR\_UDP\_RADAR\_INPROGRESS\_3 | 05/22/2022 | Similar to v2, also tries (without much success) to display the detected POIs. |

# 2. Hardware

## RPLIDAR A1 (A1M8 MODEL)

[https://www.slamtec.com/en/Lidar/A1](https://www.slamtec.com/en/Lidar/A1)

There are at least three versions of the device, one (rev 1.0) with a datasheet dated 2016, another (rev 3.0) from 2020. I have not been able to access version 2.0 datasheet. Different versions have slightli different connections. The project is done with the 2020 one, but it should work with others.

|  Version  | Datasheet            |
| ------- | --------------- |
|2020  |    https://bucketdownload.slamtec.com/d1e428e7efbdcd65a8ea111061794fb8d4ccd3a0/LD108\_SLAMTEC\_rplidar\_datasheet\_A1M8\_v3.0\_en.pdf |
| 2016  | https://bucket-download.slamtec.com/e680b4e2d99c4349c019553820904f28c7e6ec32/LM108\_SLAMTEC\_rplidarkit\_usermaunal\_A1M8\_v1.0\_en.pdf |

The RPLIDAR accepts a motor speed PWM control signal. I have set it to 5V DC fixed, maximum speed. Important, communication with RPLIDAR does not work if the 5V input to the device&#39;s serial interface is not provided by the microcontroller but instead is taken from an external power supply.

## Driver circuit

The driver circuit is very simple. It is a breakout board with the pins that the RPLIDAR needs, in the same order. See **ESP32_RPLIDAR_DRV_V0.2.png**.

The RPLIDAR communicates with the ESP32 through a serial port. I've used an ESP32 mainly because it support serveral serial ports and is faster than an ESP8266.

Please note that due a wiring error in the prototype, the second serial port's pin assignments are not the default ones and thus must be passed to the serial port ``begin()`` method.

## Base station with servo

The circuit used in the project uses an ESP01 microcontroller (Generic ESP8266 Arduino core). The circuit assumes a 5V input and provides 3.2 V for the ESP01 through an LM1117 regulator. An NodeMCU or ESP32 boards should work as well. It also provides a socket for an USB to Serial adapter. **Warning** : use a 3.3 V capable adaptor. The wiring might differ depending on the adaptor used.

See **ESP-01_RPLIDAR_UDP_SERVO_BASE_STATTION.png**

Some models of servo motor might need tweaking the parameters that control position. These are defined in the program as `MIN_SERVO_US` and `MAX_SERVO_US`. See the servo library ```attach()``` method:
  
 **```Servo.attach(pin, min, max)```**
*  **```min```**: the pulse width, in microseconds, corresponding to the minimum (0 degree) angle on the servo (defaults to 544)
*  **```max```**: the pulse width, in microseconds, corresponding to the maximum (180 degree) angle on the servo (defaults to 2400)

Use the **CALIBRATE\_SERVO.ino** sketch to find the correct values for your servo.

## Notes on hardware setup

The software assumes that the servo is mounted underneath the RPLIDAR, with the servo&#39;s axis aligned with the RPLIDAR&#39;s spindle. The servo&#39;s 90% position must be aligned with the RPLIDAR&#39;s x-axis (line passing through the rotation axis and the point in the middle of the space between the longest legs).

(See: **ALIGNMENT_SCHEMA.PNG**)
(See: **MOUNTED_PROTOYPE.jpg**)

This disposition may cause interference between the WiFi modules of the base station and the driver. The interference might be reduced by placing a piece of wire mesh between the devices.

And improved design might consider the relative position of the RPLIDAR&#39;s spindle and the servo&#39;s axis (3 numbers: distance, azimuth, and relative rotation of the axes) and then resolve the resulted triangle for the adjusted servo&#39;s orientation.

# 3. Software usage notes and reference

The include file **RPLIDAR\_utils.h** contains the WiFi parameters and the ```RPLIDAR_Packet``` message structure used in UDP communications between the driver and the base station.

It also includes logging functions with several log levels and some wrappers for ```Serial.print()``` and ``Serial.println()`` to facilitate redirecting or disabling console output.
  
**RplidarData.h** is used only by the base station. The user must declare a LidarData object and then call the ``set()`` method for every packet received form the driver.

The ``loop()`` method **must** be called frequently, preferably with each invocation of the sketch&#39;s loop() function. It performs the LidarData structure housekeeping tasks

Then the ``get()`` and ``getPointOfInterest()`` methods can be used to retrieve the distance information for a given angle and the data of the most recently moved point. Both functions use a ``DistanceData`` object (also declared in **RplidarData.h**) to hold the returned data.
 
 The header file also contains parameters configuring angular resolution, persistence of detected points and motion detection criteria. Too much angular resolution will make the ``LidarData`` structure too big to fit in memory.

Points further than ``DEFAULT_MAX_POI_DIST`` (by default  5 meters) are not candidates to be considered as points of interest. This may be changed with the setMaxPOIDist() method.

```c++
void loop()
```   
 Perform LidarData structure housekeepking. Call whenever the calling program is idle. 
 
```c++
void setMaxPOIDist(float max)
``` 
 Change max distance at which POIs are looked for.
 
```c++
bool set(float angle, float distance, int quality,  long sequence)
```
Set the position data for a given angle. Returns true if the point has moved.

```c++
bool get(float angle, DistanceData&; d, bool erase)
```
Get position data for a given angle. Returns true if there is valid data recorded. Optionally, erase (invalidate) the data.

```c++
bool getPointOfInterest(DistanceData& dd)
```
Get point data for the point of interest (most recently moved point with certain conditions). Returns true if there is a point of interest, false otherwise.

```c++
float getLoss()
```
Get percentage of UDP packets lost.

```c++
long getPackets()
```
Get number of UDP packets received.