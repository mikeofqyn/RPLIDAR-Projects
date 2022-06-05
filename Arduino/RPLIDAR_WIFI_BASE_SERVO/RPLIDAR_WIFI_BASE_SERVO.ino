#include <RPLidar.h>    // Robopeak's library for SLAMTEC RPLIDAR A1 
#include <Servo.h>
  
/****************************************************************************** 
 * Receive RPLIDAR A1 data through UDP and display it 
 * 
 * See sketch RPLIDAR_WIFI_DRIVER
 * 
 * 20 Mar 2022 by Miguel de Reyna (https://github.com/mikeofqyn)
 * 
 * 
 *
 */

// *****************************************************************************
// RPLIDAR AND AUX LIBS
// *****************************************************************************


#define RPLIDAR_LOG_LEVEL 0

#include <RPLIDAR_utils.h>

RPLIDAR_Packet messageData;

#include <RplidarData.h>

LidarData lidarData;


// *****************************************************************************
// SERVO
// *****************************************************************************

// Use CALIBRATE_SERVO.ino to calibrate servo params

const int MIN_SERVO_US =  400;
const int MAX_SERVO_US = 2400;
const int SERVO_PIN = 2;
const int ANGLE_CORRECTION = -8;  // Mounting angle error. 

Servo pointerServo;  // create servo object to control a servo

// *************************************************************************************
// SET UP
// *************************************************************************************
void setup()
{
  rplidar_set_log_level(RPLIDAR_LOG_LEVEL);
  //
  // Servo setup. Put Servo pointing left while configuring
  //
  pointerServo.attach(SERVO_PIN, MIN_SERVO_US, MAX_SERVO_US);  // attaches the servo on GIO2 to the servo object
  pointerServo.write(0); 

  //
  // This (hopefully) will help connecting to wifi when RPLIDAR and base are switched on
  // simultaneously
  delay(3000);
  
  //
  // initialize serial console if available
  //
  r_serial_begin(115200);
  r_serial_println("\n\n");
  delay(400);
  r_serial_println("\n\n\n\n\n\n\n\n");
  r_serial_println("Initializing RPLIDAR base stattion\n");

  
  // 
  // WiFi
  //
  // If BASE_STATION_ADDRESS is not defined, use DHCP
#ifdef BASE_STATION_ADDRESS  
  r_serial_println("");
  r_serial_println("Configuring network (static)");
  base_station_IP.fromString(BASE_STATION_ADDRESS);
  gateway.fromString(GATEWAY_ADDRESS);
  subnet.fromString(SUBNET_MASK);
  primaryDNS.fromString(DNS_1_ADDRESS);   //optional - not used in this example
  secondaryDNS.fromString(DNS_2_ADDRESS); //optional - not used in this example
  r_serial_println("Address: " BASE_STATION_ADDRESS ". Gateway: " GATEWAY_ADDRESS " Mask: " SUBNET_MASK);
  // Configures static IP address
  if (!WiFi.config(base_station_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    r_serial_println("STA Failed to configure. Please reset");
    while(1);
  }
#endif  
  //
  // Put Servo pointing rignt while conecting
  //
  pointerServo.write(180); 
  //
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  r_serial_print("Connecting to " WIFI_SSID " ");
  // Loop continuously while WiFi is not connected
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    r_serial_print(".");
  }
  r_serial_print("  IP address: ");
  r_serial_println(WiFi.localIP().toString());
  // Establish UDP socket and begin listening to it
  if (!UDP.begin(UDP_PORT)) {
    r_serial_print("Could not bind to UDP port "); r_serial_print(UDP_PORT); r_serial_println(". Please reset");
    while(1);
  }
  r_serial_print("Listening on UDP port "); r_serial_println(UDP_PORT);
  r_serial_println();

  // Signal connection successful
  pointerServo.write(90); 
  delay(1000);
  pointerServo.write(0); 
}

bool find_point_of_interest(float &angle);
void point_servo(float angle);

// *************************************************************************************
// LOOP
// *************************************************************************************
void loop()
{
  //
  // Wait for packet
  //
  int packetSize = UDP.parsePacket();
  if (!packetSize) {
    lidarData.loop();
    return;
  }
  // r_serial_print("Received packet! Size: ");  r_serial_println(packetSize); 
  int len = UDP.read((char *)&messageData, sizeof(messageData));
  if (len != sizeof(messageData)) {
    r_serial_print("ERROR: Invalid message size. Expected "); 
    r_serial_print(sizeof(messageData)); r_serial_print(" Received"); r_serial_println(len);
    return;
  }
  if (messageData.magicNumber != MAGIC_NUMBER) {
    r_serial_print("ERROR: Invalid magic number. Expected "); 
    r_serial_print(sizeof(MAGIC_NUMBER)); r_serial_print(" Received"); r_serial_println(messageData.magicNumber);
    return;
  }
  //
  // Update point data
  // 
  if (lidarData.set(messageData.angle, messageData.distance, messageData.quality, messageData.seqnNumber)) {
    // Log movement
    r_serial_print("MOV "); r_sep(); r_serial_print(messageData.angle); r_sep(); r_serial_println(messageData.distance);
    // If point has moved from previous location, determine if there is a new point of interest and turn the pointer towards it
    float poi_angle;
    if (find_point_of_interest(poi_angle)) {
      r_serial_print("POI "); r_sep(); r_serial_print(messageData.angle); r_sep(); r_serial_println(messageData.distance);
      point_servo(poi_angle);
      //
      // To do: compute for a pointer displaced from the lidar and adjust 0 degrees to a convenient position
    }
  } 
  //
  // Log
  //
  rplidar_log_data(messageData);
}

bool find_point_of_interest(float &angle) {
  DistanceData dd;
  if(lidarData.getPointOfInterest(dd)) {
    angle = dd.angle;
    return true;
  }
  return false;
}

void point_servo(float angle) {
      // RPLIDAR 0 degrees pointing forwards, growing clockwise 0 to 360
      // SERVO 0 degrees at right, growing to 180 (max) at left  forward 90 degrees
      // Correct for Small mounting misalignment between RPLIDAR and SERVO
      // -----------------------------------------------------
      // Conversion:
      //   Flip clockwise to conuterclockwiser: substract angle from 360
      //   Put 0 degrees at center: add 90
      //   Modulo 360
      //   Add ANGLE_CORRECTION for mounting misalignment
      //   Clip amgles outside 0 - 180
      int iangle = round(angle);
      iangle = ( ( (360-iangle) + 90) %360) + ANGLE_CORRECTION;
      r_serial_print("PN1 "); r_sep(); r_serial_println(iangle);
      if (iangle > 180) iangle = 180;
      if (iangle <   0) iangle = 0;
      pointerServo.write(iangle); 
      r_serial_print("PNT "); r_sep(); r_serial_println(iangle);
}
