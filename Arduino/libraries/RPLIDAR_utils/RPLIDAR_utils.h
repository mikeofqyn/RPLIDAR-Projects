#ifndef __RPLIDAR_UTILS_H
#define __RPLIDAR_UTILS_H

#include <Arduino.h>

#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#include <WiFiUdp.h>


// *****************************************************************************
// CONFIGURATION 
// *****************************************************************************

//
#define MAGIC_NUMBER 13092783UL

// Set WiFi credentials
#define WIFI_SSID "TP-LINK_3028"
#define WIFI_PASS "05709724"
#define UDP_PORT 4211
#define GATEWAY_ADDRESS "172.22.39.1"
#define SUBNET_MASK "255.255.255.0"
#define DNS_1_ADDRESS "8.8.8.8"  // Google's DNS servers 
#define DNS_2_ADDRESS "8.8.4.4"
#define RPLIDAR_BROADCAST_ADDRESS "172.22.39.255"

// Use an address not managed by the router's DHCP server
// This is used only to configure base station network interface, packets are
// sent to the broadcast address allowing for serveral monitoring applications.
// If not defined (comment out), then tha base station is configured via DHCP
#define BASE_STATION_ADDRESS "172.22.39.90"  

IPAddress   gateway, subnet;
IPAddress   primaryDNS, secondaryDNS;
IPAddress   base_station_IP;

typedef enum station_id_e { base = 0, RPLIDAR_1 = 1 } station_id_t;

typedef enum packet_type_e { DEBUG_DATA = 0, RPLIDAR_DATA = 1 } packet_type_t;

//
// UDP object used to create socket
//
WiFiUDP UDP;

// *****************************************************************************
// DATA PACKET  
// *****************************************************************************

struct __attribute__((packed))  RPLIDAR_Packet
{
    uint32_t      magicNumber;
    packet_type_t packetType;
    station_id_t  fromId;
    uint32_t      fromIP;
    uint32_t      seqnNumber;
    uint32_t      onTimeMillis;
    float         distance;
    float         angle;
    bool          startBit;
    byte          quality;
};

void RPLIDAR_packet_init(struct RPLIDAR_Packet &pk, packet_type_t type, station_id_t f_Id, uint32_t f_IP) {
    pk.magicNumber     = MAGIC_NUMBER;
    pk.packetType      = type;
    pk.fromId          = f_Id;
    pk.fromIP          = f_IP;  
    pk.seqnNumber      = 0;
    pk.onTimeMillis    = 0;
    pk.distance        = 0.0;
    pk.angle           = 0.0;
    pk.startBit        = false;
    pk.quality         = 0;
}

// *************************************************************************************
// LOGGING
// *************************************************************************************
// 0 - NONE  - Don't log
// 1 - STATS - Statistics for each scan cycle (default)
// 2 - LIDAR - Log each point LIDAR data
// 3 - COORD - Log XY coordinates

#ifndef RPLIDAR_LOG_LEVEL 
#define RPLIDAR_LOG_LEVEL 1
#endif

static unsigned int RPLIDAR_log_level = RPLIDAR_LOG_LEVEL;

void rplidar_set_log_level(unsigned int ll) {
    RPLIDAR_log_level = ll;
}

#define r_serial_print(x)    Serial.print(x)
#define r_serial_println(x)  Serial.println(x)
#define r_serial_begin(x)    Serial.begin(x)
#define r_sep() Serial.print(" ");

unsigned long last_sequence_received = 1;
bool firstpacket = true;
unsigned long packets_lost = 0;


void rplidar_log_data(RPLIDAR_Packet& data, float percent_good = -1.0) {
    if (RPLIDAR_log_level == 0) {
        return;
    }
    static unsigned long startcycle = millis();
    if (RPLIDAR_log_level == 1) {  // Summary
        // Tally lost packets
        if (firstpacket) {
            firstpacket = false;
        }
        else {
            if (data.seqnNumber < last_sequence_received) {  // sender has reset count
                packets_lost = 0;
            }
            else {
                packets_lost += (data.seqnNumber - (1 + last_sequence_received));
            }
        }
        last_sequence_received = data.seqnNumber;
        static unsigned int n_data = 0, n_errors = 0;
        static float max_dist = 0.0, min_dist = 0.0;
        static float sumdist = 0.0, sumqual = 0.0;
        static unsigned long cycle_ms = 0;
        if (data.startBit) {
            cycle_ms = millis() - startcycle;
            float cycles_per_ms = 1.0 / (cycle_ms);
            float avg_dist = sumdist / n_data;
            float avg_qual = sumqual / n_data;
            r_serial_print("n_data: "); r_serial_print(n_data); r_serial_print("\t max: "); r_serial_print(max_dist); r_serial_print("\t min: "); r_serial_print(min_dist);
            r_serial_print("\t avg: "); r_serial_print(avg_dist); r_serial_print("\t avg_q: "); r_serial_print(avg_qual);
            r_serial_print("\t t(ms): "); r_serial_print(cycle_ms); r_serial_print("\tHz: "); r_serial_print(1.e3 * cycles_per_ms);
            if (percent_good < 0) { // Meaning receivinbg side
                r_serial_print("\t lost: "); r_serial_print(packets_lost); r_serial_print("\t ("); r_serial_print(100.0 * packets_lost / last_sequence_received); r_serial_println("%)");
            } else {
                r_serial_print("\t Good: "); r_serial_print(percent_good); r_serial_println("%)");
            }
            n_data = 0;
            sumdist = max_dist = min_dist = data.distance;
            sumqual = data.quality;
            startcycle = millis();
        }
        n_data++;
        if (data.distance > max_dist) max_dist = data.distance;
        if (data.distance < min_dist) min_dist = data.distance;
        sumdist += data.distance;
        sumqual += data.quality;
    } else  {
        // Detailed log 
        r_serial_print("N: "); r_serial_print(data.seqnNumber);
        r_serial_print("\tt: "); r_serial_print(data.onTimeMillis);
        r_serial_print("\tT: "); r_serial_print(data.packetType);
        r_serial_print("\tA: "); r_serial_print(data.angle);
        r_serial_print("\t D: "); r_serial_print(data.distance);
        r_serial_print("\t S: "); r_serial_print(data.startBit);
        r_serial_print("\t Q: "); r_serial_println(data.quality);
     }
}

#endif

