import java.net.*;
import java.io.*;
import java.util.Arrays;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import processing.serial.*;

final float mmPerPixel = 10.0;
final float maxDistMm = 15000.0;  // 12m

final int DIVISIONS_PER_DEGREE = 4; // Number of divisions per degree = size of points array
final float MOVED_FACTOR = 0.05; // consider the point has moved if distance changes more than this factor (5% - 5cm at 1 m)
final long MS_TO_FORGET = 2000;  // Miliseconds to forget a point if not seen 

float zoomFactor = 1.0; 
boolean labels_on = false;  // show lables (slow)
boolean eraseData = false;  // erase data every cycle


PFont markerFont;  // Global font variable

// *************************************************************************************************    
// *************************************************************************************************    

// *************************************************************************************************    
float mm2pixels (float mm)  { 
  return  mm * zoomFactor / mmPerPixel;
}


// *************************************************************************************************    
float Xworld2window(float mm) { 
  return ( mm2pixels(mm) +  width ) / 2;
}
  
// *************************************************************************************************    
float Yworld2window(float mm) { 
  return ( height - mm2pixels(mm)) / 2;
}

// *************************************************************************************************    
void centeredArcPx(float pxradius, float angle1, float angle2) {
  float a_ini = TWO_PI - angle2; // flip y axis
  float a_fin = TWO_PI - angle1;
  arc(Xworld2window(0.0), Yworld2window(0.0), pxradius, pxradius, a_ini, a_fin);
}
void centeredArc(float mmradius, float angle1, float angle2) {
  float rpixels = mm2pixels(mmradius);
  centeredArcPx(rpixels,  angle1, angle2);
}

// *************************************************************************************************    
void drawGridSegment(float angle1, float angle2, boolean erase) {
  //
  // Clear the segment  
  if (erase) {
    fill(0,0,11);
    noStroke();
    centeredArc(maxDistMm, angle1, angle2);
  }
  
  //
  // draw the concentric distance lines
  stroke(0, 100, 0);
  strokeWeight(1);
  for (float r = 1000.0; r < maxDistMm; r += 1000.0) {
    noFill();
    centeredArc(r, angle1, angle2);
  } 
  //
  // Axes
  line(0, height/2, width, height/2);
  line(width/2, 0, width/2, height);

  if (!labels_on) return;
  //
  // Label the distance lines
  textFont(markerFont, 10);
  textAlign(LEFT);
  fill(40,50,255);
  for (float r = 1000.0; r < maxDistMm+1; r += 1000.0) {
    String label = String.format("%.2f", r/1000.0) + "m";
    text(label, Xworld2window(r), Yworld2window(80));
    text(label, Xworld2window(80), Yworld2window(r));
  }
  noFill();
}

void drawGrid() {
  clear();
  drawGridSegment(0.0, TWO_PI, true);
}


// *************************************************************************************************    
//
// Serial
//
Serial S;

// *************************************************************************************************    

void setup() {
  size(900, 900);
  background(0);  //107 GREY
  frameRate(30);
  markerFont = createFont("Arial", 16, true);  // Loading font

  try {
    S = new Serial(this, "COM5", 115200);
  } 
  catch (Exception e) {
    e.printStackTrace();
    exit();
    return;
  }
  delay(1000);
  S.clear();

  drawGrid();
  
}


// *************************************************************************************************    
// 
// loop and draw points
//

//---------------------------------------------------------------------------------------------------

float old_rads_1 = 0.0;
float old_rads_2 = 0.0;
float old_distance = -1.0;

void draw() {

  float angle;
  float distance;
  float rads_1, rads_2;
  
  while (S.available() > 0) {
      String inputstr = S.readStringUntil(10);  // until LF
      if (inputstr != null) {
        println(inputstr);
        String[] q = splitTokens(inputstr);
        // print("Found: ", q.length, " "); for (int i=0; i<q.length; i++) print("'"+q[i]+"' "); println();
        if (q.length >= 3) {
          if (q[0].equals("MOV") || q[0].equals("POI")) {
            angle = float(q[1]);
            distance = float(q[2]);
            rads_1 = (float) Math.toRadians(angle);
            rads_2 = (float) Math.toRadians(angle+(1.0/DIVISIONS_PER_DEGREE));
            if (q[0].equals("POI")) {
              strokeWeight(4);
              stroke(130, 90, 90);
              centeredArc (old_distance, old_rads_1, old_rads_2);
              stroke(255, 255, 140);
              centeredArc (distance, rads_1, rads_2);
              //println("> "+distance+" "+angle+"º - "+inputstr);
              old_rads_1 = rads_1;
              old_rads_2 = rads_2;
              old_distance = distance;
            } else if (q[0].equals("MOV")) {
              strokeWeight(3);
              stroke(40, 40, 99);
              centeredArc (distance, rads_1, rads_2);
            }
          }
        } else {
          println("Not enough tokens");
        }
      } else {
        // println("null");
      }
  }
}
        
// *************************************************************************************************    
// 
// Handle keyboard commands

void keyPressed() {
  switch(key) {
    case '+': 
      zoomFactor *= 1.1;
      println("Zomm+: "+zoomFactor*100.0+"%");
      clear();
      drawGrid();
      break;
    case '-': 
      zoomFactor /= 1.1;
      println("Zomm-: "+zoomFactor*100.0+"%");
      clear();
      drawGrid();
      break;
    case '=': 
      zoomFactor = 1.0;
      println("Zomm=: "+zoomFactor*100.0+"%");
      clear();
      drawGrid();
      break;
    case 'C':
    case 'c':
      println("Clear screen");
      clear();
      drawGrid();
      break;
    case 'l':
    case 'L':
      labels_on = !labels_on;
      println("Toggle show labels. Now: " + labels_on);
      drawGrid();
      break;
  }
}
