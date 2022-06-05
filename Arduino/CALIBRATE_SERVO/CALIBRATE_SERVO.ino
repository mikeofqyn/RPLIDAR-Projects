#include <Servo.h>
Servo myservo;

const int DEFAULT_MIN =  400;
const int DEFAULT_MAX = 2400;
const int SERVO_PIN = 2;

void setup()
{
  int min_us = DEFAULT_MIN; 
  int max_us = DEFAULT_MAX;
  int temp;
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n\nServo tester.\n\n");
  
  Serial.println("Enter calibration parameters for servo...\n");
  Serial.print("Min microseconds ("); Serial.print(min_us); Serial.print(") :");
  if (wait_integer(temp)) min_us = temp;
  Serial.println();
  Serial.print("Max microseconds ("); Serial.print(max_us); Serial.print(") :");
  if (wait_integer(temp)) max_us = temp;
  Serial.print("\nMin: "); Serial.print(min_us);Serial.print(", Max: ");Serial.println(max_us);
  

  myservo.attach(SERVO_PIN, min_us, max_us);

  Serial.print("\n\nTurning from 0 to 180\n");  
  for(int pos = 0; pos <= 180; pos += 1) {
    myservo.write(pos);
    delay(10);
  }
  Serial.print("\n\n180 degrees:");
  myservo.write(180);
  delay(1500);
  Serial.print("\n\n90 degrees:");
  myservo.write(90);
  delay(1500);
  Serial.print("\n\n0 degrees:");
  myservo.write(0);
  Serial.println("servo calibrated");
  delay(1500);
  Serial.println("\n\n");
  Serial.println("Waiting for input. Write position in degrees");
}

void loop() {
  int pos;
  if (get_integer(pos)) {
    myservo.write(pos);
  }
}

bool wait_integer(int &val) {
  unsigned long ms = millis();
  while ( (millis() - ms) < 15000) {
    if (Serial.available()) {
      return get_integer(val);
    }
  }
  return false;
}

bool get_integer(int &val) {
  Serial.flush();
  flush_input();
  if (!Serial.available()) {
    return false;
  }
  val = Serial.parseInt();
  Serial.print(">");
  Serial.println(val);
  return true;
}

void flush_input() {
  if (Serial.available()) { // skip non-numeric 
    char c = Serial.peek(); 
    while ( (c<'0' || c>'9') && (c != '-') ) {
      if (c<0) {
        return; 
      }
      char cc = Serial.read();
      if (Serial.available()) c = Serial.peek(); else return;
    }
  }
}



  
