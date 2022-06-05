#include <RPLidar.h>    // Robopeak's library for SLAMTEC RPLIDAR A1 
  
/****************************************************************************** 
 * Receive RPLIDAR A1 data through UDP and display it 
 * 
 * See sketch RPLIDAR_WIFI_DRIVER
 * 
 * 20 Mar 2022 by Miguel de Reyna (https://github.com/mikeofqyn)
 * 
 * 
 * Uses Rob Tillaart's library to convert X-Y values to analog voltages
 *    https://github.com/RobTillaart/MCP4725
 * 
 * Test data and some snippets taken from 
 *    MCP4725 Example Waveform Sketch by  Joel Bartlett (SparkFun Electronics)
 *    Sept. 11, 2014 https://github.com/sparkfun/MCP4725_Breakout
 *    
 *
 */

// *****************************************************************************
// LIMITS
// *****************************************************************************

const unsigned int MAX_DAC_VALUE = 4096;  // MCP4725 is a 12-bit DAC 
const int MIN_QUALITY = 0;       // Points below this quality level are ingnored

#define RPLIDAR_LOG_LEVEL 1

#include <RPLIDAR_utils.h>

RPLIDAR_Packet messageData;

#include <RplidarData.h>

LidarData lidarData;

// *************************************************************************************
// SET UP
// *************************************************************************************
void setup()
{
  rplidar_set_log_level(RPLIDAR_LOG_LEVEL);
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
}

bool find_point_of_interest();

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
    // If point has moved from previous location, determine if there is a new point of interest
    find_point_of_interest();
    r_serial_print("MOV "); r_sep(); r_serial_print(messageData.angle); r_sep(); r_serial_println(messageData.distance);
  } 
  //
  // Log
  //
  rplidar_log_data(messageData);
}

bool find_point_of_interest() {
  DistanceData dd;
  if(lidarData.getPointOfInterest(dd)) {
    r_serial_print("POI "); r_sep(); r_serial_print(dd.angle); r_sep(); r_serial_println(dd.distance);
    return true;
  }
  return false;
}
