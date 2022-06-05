#include <Wire.h>       // Include the Wire library to talk I2C
#include <MCP4725.h>    // 16-bit I2C DAC
#include <RPLidar.h>    // Robopeak's library for SLAMTEC RPLIDAR A1 

/****************************************************************************** 
 * Handle 2 MCP4725 DACs (Sparkfun breakout boards) to draw XY coordinates from
 * an RPLIDAR A1 on an oscilloscope.
 * 
 * Tested on  Arduino Mega and  ESP32 borads, which both have more than one 
 * hardware serial ports 
 * 
 * 03 Mar 2022 by Miguel de Reyna (https://github.com/mikeofqyn)
 * 
 * Uses Rob Tillaart's library 
 *    https://github.com/RobTillaart/MCP4725
 * 
 * Test data and some snippets taken from 
 *    MCP4725 Example Waveform Sketch by  Joel Bartlett (SparkFun Electronics)
 *    Sept. 11, 2014 https://github.com/sparkfun/MCP4725_Breakout
 *
 * RPLidar library and code snippets Copyright (c) 2014, RoboPeak
 *    https://RoboPeak.com
 *    
 */

// *****************************************************************************
// LIMITS
// *****************************************************************************


const unsigned int MAX_DAC_VALUE = 4096;  // MCP4725 is a 12-bit DAC 

const float MAX_DIST_MM = 8000.0;    // Max distance mapped into the DAC's range

const int MIN_QUALITY = 2;       // Points below this quality level are ingnored



// *****************************************************************************
// MCP4725 DAC
// *****************************************************************************
// This is the I2C Address of the Sparkfun's MCP4725 breakout board, by default 
// (A0 pulled to GND). Please note that this breakout is for the MCP4725A0. 
// #define MCP4725_ADDR 0x60. For devices with A0 pulled HIGH, use 0x61
// ---
// The following addresses are the ones used by the chinese version I'm using.
// Using 2 devices needs to create a solder bridge to pull up A0 as well as 
// disabling the pullup resistors on one of them by cutting the traces, see 
//  
// https://learn.sparkfun.com/tutorials/mcp4725-digital-to-analog-converter-hookup-guide/all
// 
#define MCP4725_1_ADDR 0x62    // default
#define MCP4725_2_ADDR 0x63    // A0 pulled to Vcc

//
// Declare DACs
// 

MCP4725 MCP_1(MCP4725_1_ADDR);
MCP4725 MCP_2(MCP4725_2_ADDR);

// *****************************************************************************
// PWM CONTROL OF RPLIDAR'S MOTOR SPEED
// *****************************************************************************

// Motor speed (PWM pin). Min about 140, max 255 (100%). See file MOTOR_SPEED.txt
#define RPLIDAR_MOTOR_SPEED 150

// PWM pin controlling RPLIDAR's motor speed. Connect to RPLIDAR's MOTOCTRL signal 
#define RPLIDAR_MOTOR_PWM_PIN 3  // Valid for MEGA, UNO and ESP32

#ifdef ESP32   // EMULATE UNO'S BEHAVIOUR ON ESP32 (needs voltage level shifter)
  const int freq = 1000;
  const int ledChannel = 0;
  const int resolution = 8;
  void init_pwm() { 
      ledcSetup(ledChannel, freq, resolution);
      ledcAttachPin(RPLIDAR_MOTOR_PWM_PIN, ledChannel);
  }
  void set_speed(int dutyCycle) {
      ledcWrite(ledChannel, dutyCycle);
  }
#else
  void init_pwm() { 
      pinMode(RPLIDAR_MOTOR_PWM_PIN, OUTPUT);  
  }
  void set_speed(int dutyCycle) {
      digitalWrite(RPLIDAR_MOTOR_PWM_PIN, dutyCycle);
  }
#endif


// *****************************************************************************
// RPLIDAR A1
// *****************************************************************************
//  
// Create a driver instance 
//    WARNING   - RPLIDAR's motor uses 5V power and up to 5V control signal. 
//    REMEMBER  - Connect the grounds of the ESP32 and the motor's power supply
//    IMPORTANT - The RPLIDAR's *SIGNAL* Vin must also be connected to 5V for
//                the serial connection to work
//
RPLidar lidar;

// If the board has a second hardware serial interface to connect 
// the RPLIDAR to, then 'Serial' is used for debugging and logging
#if defined(ESP32)
  #include <HardwareSerial.h>
  HardwareSerial SerialTwo(2);  // ESP32: UART(2) pins 16-RX, 17-TX
  #define RPLIDAR_SERIAL  SerialTwo
#elif defined(HAVE_HWSERIAL1) // MEGA:  19-RX, 18-TX
  #define RPLIDAR_SERIAL  Serial1
#endif

#ifdef  RPLIDAR_SERIAL   
#define RPL_HWSERIAL    RPLIDAR_SERIAL
#define r_serial_print(x)    Serial.print(x)
#define r_serial_println(x)  Serial.println(x)
#define r_serial_begin(x)    Serial.begin(x)
#define r_sep() Serial.print(" ");
#else
#define RPL_HWSERIAL    Serial
#define r_serial_print(x)    delay(1)
#define r_serial_println(x)  delay(1)
#define r_serial_begin(x)    delay(1)
#define r_sep() delay(1)
#endif

//
// 0 - NONE  - Don't log
// 1 - STATS - Statistics for each scan cycle
// 2 - LIDAR - Log each point LIDAR data
// 3 - COORD - Log XY coordinates
// 
#define LOG_DATA 1  

// *************************************************************************************
// LOGGING
// *************************************************************************************
#if (LOG_DATA > 0)

unsigned long startcycle = millis();

void log_data(float angle, float distance, byte quality, bool startBit) {

#if (LOG_DATA == 1) // STATS
  static unsigned int n_data = 0, n_errors = 0;
  static float max_dist = 0.0, min_dist = 0.0;
  static float sumdist = 0.0, sumqual = 0.0;
  static unsigned long cycle_ms = 0;
  if (startBit) {
    cycle_ms = millis() - startcycle;
    float cycles_per_ms = 1.0/(cycle_ms);
    float avg_dist = sumdist / n_data;
    float avg_qual = sumqual / n_data;
    r_serial_print("n_data: ");r_serial_print(n_data);r_serial_print("\t max: ");r_serial_print(max_dist);r_serial_print("\t min: ");r_serial_print(min_dist);
    r_serial_print("\t avg: ");r_serial_print(avg_dist);r_serial_print("\t avg_q: ");r_serial_print(avg_qual);
    r_serial_print("\t t(ms): ");r_serial_print(cycle_ms);r_serial_print("\tHz: ");r_serial_println(1.e3 * cycles_per_ms);
    n_data = 0;
    sumdist = max_dist = min_dist = distance;
    sumqual = quality;
    startcycle = millis();
  }
  n_data++;
  if (distance > max_dist) max_dist = distance;
  if (distance < min_dist) min_dist = distance;
  sumdist += distance;
  sumqual += quality;
#else  // FULL
  r_serial_print("Angle: ");r_serial_print(angle);r_serial_print("\t Dist: ");r_serial_print(distance);
  r_serial_print("\t Start: ");r_serial_print(startBit);r_serial_print("\t Quality: ");r_serial_println(quality);
#endif
}
#endif  // LOG_DATA > 0


// *************************************************************************************
// SET UP
// *************************************************************************************
void setup()
{
  //
  // initialize serial console if available
  //
  r_serial_begin(115200);

  //
  // I2C interface & DACs
  //
  Wire.begin();  
  MCP_1.begin(); // Uno, Mega, etc. use default I2C 
  MCP_2.begin();
  Wire.setClock(400000);
  //
  // ESP32 needs SDA and SCL pins  (NOT TESTED)
  // MCP_1.begin(27, 26);  //  First one is SDA (data), second one is clock (SCL)
  // MCP_2.begin(27, 26);  //  First one is SDA (data), second one is clock (SCL)
  // Wire.setClock(3400000);

  //  
  // RPLidar
  //
  lidar.begin(RPL_HWSERIAL);
  init_pwm();

}


// *************************************************************************************
// LOOP
// *************************************************************************************
void loop()
{
  //
  // Get next point
  //
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement

    //
    // Log
    //
#if (LOG_DATA == 2) || (LOG_DATA == 1)
    log_data(angle, distance, quality, startBit);
#endif

    //
    // Compute XY coordinates and set analog values
    //
    //if (quality >= MIN_QUALITY) {
    if (0) { /////////////////////////////////////////////////////////////
      static const float degtorad = TWO_PI / 360.0;
      float anglerad = angle * degtorad;
      float dist = min(distance, MAX_DIST_MM);
      
      float x = dist * cos(anglerad);
      float y = dist * sin(anglerad);
  
      x = map (x,  -MAX_DIST_MM, MAX_DIST_MM, 0, MAX_DAC_VALUE);
      y = map (y,  -MAX_DIST_MM, MAX_DIST_MM, 0, MAX_DAC_VALUE);

#if (LOG_DATA == 3)
    r_serial_print(startBit? "  ": "* "); r_serial_print("(x,y) "); r_serial_print(x); r_serial_print(",");r_serial_print(y); r_serial_print("  Q_");r_serial_println(quality);
#endif

      MCP_1.setValue(x);  
      MCP_2.setValue(y);
    }
    
  } else {    
    //
    // LIDAR NOT READY.
    //
#if (LOG_DATA > 0)    
    r_serial_println("Waiting for RPLIDAR");
#endif
    set_speed(0); //stop the rplidar motor
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // detected...
       lidar.startScan();
       // start motor rotating at configured speed
       set_speed(RPLIDAR_MOTOR_SPEED);  
       delay(1000);
    }
  }  
}
