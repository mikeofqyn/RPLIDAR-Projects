import java.net.*;
import java.io.*;
import java.util.Arrays;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

final float mmPerPixel = 10.0;
final float maxDistMm = 15000.0;  // 12m

final int DIVISIONS_PER_DEGREE = 10; // Number of divisions per degree = size of points array
final float MM_TO_CONSIDER_MOVED = 50.0; // consider the point has moved if distance changes more than this
final long MS_TO_FORGET = 2500;  // Miliseconds to forget a point if not seen 

float zoomFactor = 1.0; 
boolean labels_on = false;  // show lables (slow)
boolean eraseData = false;  // erase data every cycle


PFont markerFont;  // Global font variable

class DistanceData {
  public float distance;
  public float max = 0.0;
  public float min = 0.0;
  public long times_seen = 0;
  public long last_seen = 0;
  public long last_moved;
}

class AngleData {
    // To do: keep average quality
    private float sum = 0.0;
    private long count = 0;
    private float maxd = 0;
    private float mind = 0;
    private long last_seen_ms = 0;
    private long last_moved_ms = 0;
    private final Object lock = new Object();

    public void addDist(float d) {
        synchronized (lock) {
            if (count > 0) {
              if (Math.abs(d - sum/count) > MM_TO_CONSIDER_MOVED) {
                last_moved_ms = millis();
                sum = 0;
                count = 0;
              } 
            }
            sum += d;
            if (count == 0) {
              maxd = d; 
              mind = d;
            } else if (d > maxd) {
                maxd = d;
            } else {
              if (d < mind) mind = d;
            }
            count++;
            last_seen_ms = millis();
        }
    }
    public boolean getData(DistanceData dd, boolean eraseit) {
        synchronized (lock) {
            dd.distance = sum/count;
            dd.max = maxd;
            dd.min = mind;
            dd.times_seen = count;
            dd.last_seen = last_seen_ms;
            dd.last_moved = last_moved_ms;
            boolean has_data = count > 0;
            if (eraseit) {
              this.erase();
            }
            return has_data;
        }
    }
    public boolean getData(DistanceData dd) {
      return getData(dd, false);
    }
    public void erase() {
        synchronized (lock) {
            count = 0;
            sum = 0.0;
            maxd = 0;
            mind = 0;
            last_seen_ms = 0;
            last_moved_ms = 0;
        }
    }
}

class LidarData {
  private AngleData[] data = new AngleData[360*DIVISIONS_PER_DEGREE];
  
  public volatile boolean  firstpacket = true;
  public volatile float    pctloss = 0.;
  public volatile long     packets = 0;
  public volatile long     firstseq = 0;
  public volatile long     lastseq = 0;
 
  public LidarData() {
    for(int i=0; i<data.length; i++) {
      data[i] = new AngleData();
    }
  }

  public void clear() {
    for(int i=0; i<data.length; i++) {
      data[i].erase();
      firstpacket = true;
      pctloss = 0;
      packets = 0;
      firstseq = 0;
      lastseq = 0;
    }
  }

  private int getIndex(float angle) {
    int iangle = (int) Math.round(angle*DIVISIONS_PER_DEGREE); 
    if (angle<0) iangle = 0;
    if (iangle>=360*DIVISIONS_PER_DEGREE) iangle = (360*DIVISIONS_PER_DEGREE)-1;
    return iangle;
  }

  public void set(float angle, float distance, int quality, long sequence) {
    packets++;
    if (firstpacket) {
      firstseq = sequence;
      firstpacket = false;
    }
    lastseq = sequence;
    int i = getIndex(angle);
    data[i].addDist(distance);
  }
  
  public boolean get(float angle, DistanceData d, boolean erase) {
    int i = getIndex(angle);
    return  data[i].getData(d, erase);
  }
  
  public boolean get(float angle, DistanceData d) {
    return get(angle, d, false);
  }
  
  public float  getLoss() {
    float packetssent = lastseq - firstseq; 
    float packetslost = packetssent - packets;
    if (packetssent == 0) return 0.0; 
    pctloss = 100.0 * packetslost / packetssent;
    return pctloss;
  }
  
  public long getPackets() {
    return packets;
  }
  
}

/** Shared structure
 *
 */
 
LidarData sharedData  = new LidarData();


// *************************************************************************************************    

/** Convert from little endian byte buffer to long
 * @param data (byte[4]) 
 * @return unsigned int (0 - 4,294,967,295)
 */
public static long toUInt(final byte[] data, int pos) {
    if (data == null || (pos+4 >= data.length))
        throw new IllegalArgumentException("!= 4 bytes");
    return (long)(
        (long)(data[3+pos] & 0xffL) << 24 |
        (long)(data[2+pos] & 0xffL) << 16 |
        (long)(data[1+pos] & 0xffL) << 8  |
        (long)(data[0+pos] & 0xffL)
         );
}


// *************************************************************************************************    

/** Receive data packets and update the shared database
 * executed throug thread()
 */

void LidarUDPListener() {  // Thread

  byte[] buf = new byte[256]; // packet buffer
  DatagramSocket socket = null;
  DatagramPacket packet = new DatagramPacket(buf, buf.length);
  
  long lastseq = 0;
  long firstseq = 0;
  boolean firstpacket = true;
  long packetsreceived = 0;
  long packetssent;
  long packetslost = 0;  
  long diff = 0;
  long count = 0;
  
  /* Ver sketch Arduino RPLIDAR_WIFI_DRIVER.ino: struct __attribute__((packed))  DataPacket
  
                              //   Start     bytes            Example  Little endian
  ----------------------------//   =====     =====      =============  =============  */
  long    magicNumber = 0;    //      0        4              1309278   00 C7 C7 AF
  int     packetType = 0;     //      4        4                    1   01 00 00 00 // DEBUG_DATA=0, RPLIDAR_DATA=1 
  int     fromId = 0;         //      8        4                    1   01 00 00 00
  String  SenderIP = null;    //     12        4        172.22.39.100   AC 16 27 64 // Network addresses 
  long    seqnNumber = 0;     //     16        4               620027   FA 75 09 00
  long    onTimeMillis = 0;   //     20        4               144167   27 33 02 00 
  float   distance = 0.0;     //     24        4               473.75   00 E0 EC 43
  float   angle = 0.0;        //     28        4               359,45   00 BA B3 43
  boolean startBit = false;   //     32        1                    1   01
  int     quality = 0;        //     33        1                   15   0F
  
  // Magic         Packet type   From Id       fromIP        Seq number    OnTime ms.    Distance      Angle         Start  Quality
  // AF,C7,C7,00 | 01,00,00,00 | 01,00,00,00 | AC,16,27,64 | FA,75,09,00 | 27,33,02,00 | 00,00,A0,40 | 00,00,A0,40 | 01    | 00 
  // AF,C7,C7,00 | 00,00,00,00 | 01,00,00,00 | AC,16,27,64 | 27,33,02,00 | 81,75,01,00 | 00,E0,EC,43 | 00,BA,B3,43 | 00    | 0F
  
  //
  // initialize
  //
  try {
    // socket = new DatagramSocket(4211, InetAddress.getByName("172.22.39.90")); // Set your port here
   socket = new DatagramSocket(4211, InetAddress.getByName("0.0.0.0")); // Set your port here
  }
  catch (Exception e) {
    e.printStackTrace(); 
    println(e.getMessage());
    return;
  }
  println(socket.getLocalSocketAddress());

  //
  // loop
  //
  while(true) {
    try {
      socket.receive(packet);
      packetsreceived++;
              
      // InetAddress address = packet.getAddress();
      // int port = packet.getPort();
      // packet = new DatagramPacket(buf, buf.length, address, port);
      // int len = packet.getLength();
      magicNumber  = toUInt(buf,  0);
      seqnNumber   = toUInt(buf, 16);
      onTimeMillis = toUInt(buf, 20);
      packetType   = ByteBuffer.wrap(buf,  4, 8).order(ByteOrder.LITTLE_ENDIAN).getInt();
      fromId       = ByteBuffer.wrap(buf,  8, 8).order(ByteOrder.LITTLE_ENDIAN).getInt();

      if (sharedData.getPackets() == 0) {
        byte[] fromIP = new byte[4];
        fromIP[0] = buf[12]; fromIP[1] = buf[13]; fromIP[2] = buf[14]; fromIP[3] = buf[15];
        SenderIP = InetAddress.getByAddress(fromIP).getHostAddress();
      }
      distance     = ByteBuffer.wrap(buf, 24, 4).order(ByteOrder.LITTLE_ENDIAN).getFloat(); 
      quality      = buf[33];
      if (distance > 0.1 && quality > 1)  {
        angle        = ByteBuffer.wrap(buf, 28, 4).order(ByteOrder.LITTLE_ENDIAN).getFloat(); 
        startBit     = (buf[32] == 1);
    
        String mark = "* ";
        
        if (packetType == 1) { // RPLIDAR DATA
          sharedData.set(angle, distance, quality, seqnNumber);
          mark  = "> ";
        } else {
          print(mark); println("Debug packet received");
        }
        // print (mark);
        // print(">"); print(magicNumber);
        // print("\t"); print(packetType);
        // print("\t"); print(fromId);
        // print("\t");
        //
        /*---------------------------------------------------------
        print(seqnNumber);
        print("\t"); print(onTimeMillis);
        print("\t"); print(SenderIP);
        print("\t"); print(angle);
        print("\t"); print(distance);
        print("\t"); print(startBit);
        print("\t"); print(quality);
        print("\tS:"); print(packetssent);
        print("\tR:"); print(packetsreceived);
        print("\tD:"); print(diff);
        print("\tL:"); print(packetslost);
        print("\t"); print(pctloss);
        println("%");
        ---------------------------------------------------------*/
        /*
            int i;
            for (i=0; i<len; i++) {
              print (hex(buf[i])); print(",");
            }
            println("");
        */    
      } // distance > 0.1
    } // Try
    catch (IOException e) {
      e.printStackTrace(); 
      println(e.getMessage());
    }  
  } // loop
} // LidarUDPListener()
  
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
    fill(0,0,0);
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

  
  //
  // stats
  fill(40,200,40);
  textFont(markerFont, 14);
  String label2 = String.format(" %d", sharedData.getPackets()) + " pkts  " + String.format("%.2f", sharedData.getLoss()) + "% loss"+ " ZOOM: " + String.format("%.3f ", zoomFactor*100)+"%  ";
  float lblwidth = textWidth(label2); 
  rect(1, 1, lblwidth+4, 30, 3);
  textAlign(LEFT);
  fill(0);
  text(label2, 8, 25);
  noFill();
  
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

void setup() {
  size(900, 900);
  background(0);  //107 GREY
  frameRate(240);
  markerFont = createFont("Arial", 16, true);  // Loading font

  //
  // Grid
  //
  drawGrid();

  //
  // Start UDP listener
  //
  
  thread("LidarUDPListener");

}


// *************************************************************************************************    
// 
// loop and draw points

float division_n = 0.0;  // degrees
DistanceData dd = new DistanceData();

void draw() {
  boolean newturn = false;
  //drawGrid();

  while (!newturn) {
    
    float curr_angle = division_n/DIVISIONS_PER_DEGREE;
  
    division_n += 1;
    if (division_n >= (360.0*DIVISIONS_PER_DEGREE)-1) {
      division_n = 0;
      newturn = !newturn;
    }
  
    boolean has_data = sharedData.get(curr_angle, dd, eraseData);
    
    if (!has_data)
      continue;

    long  nseen = dd.times_seen;
  
    float rads = (float) Math.toRadians(curr_angle);
    float rads2 = (float) Math.toRadians(curr_angle+(1.0/DIVISIONS_PER_DEGREE));
    float rads1 = (float) Math.toRadians(curr_angle-(1.0/DIVISIONS_PER_DEGREE));

    if ( (millis() - dd.last_seen) > MS_TO_FORGET) {
      sharedData.get(curr_angle, dd, true);
      drawGridSegment(rads, rads2, true);
      continue;
    }
    
    boolean has_moved = (millis() - dd.last_moved) < 1000;
    
    if (has_moved)    
      drawGridSegment(rads, rads2, true);    
    
    int ncolor = newturn? 100: 180;
    int rcolor = has_moved? 255: 0;
  
    strokeWeight(6);
    stroke(rcolor, ncolor, 0);
    centeredArcPx (width/1.1, rads1, rads);
   
    //
    //
    // Clear segment and draw arc
    //
    if (eraseData) {
      color col = #B0F0B0;
      if (nseen > 3) col = #D0F0D0;
      if ((millis() - dd.last_moved) < 80) col = #0000FF;
      stroke(col);
    } else {
      long c1 = nseen * 100;
      long c2 = 0;
      long c3 = 0;
      if (has_moved) {
        c1 = 80;
        c2 = 80;
        c3 = 255;
      } else {
        c1 = nseen * 100;
        if (c1 > 255) {
          c2 = c1 - 255;
          c1 = 255;
          if (c2 > 255) {
            c3 = Math.max(c2-255, 255);
            c2 = 255;
          }
        }
      }
      stroke(c1, c2, c3);
    }
    strokeWeight(4);
    centeredArc(dd.distance, rads, rads2);
  }
}

// *************************************************************************************************    
// 
// Handle keyboard commands

void keyPressed() {
  switch(key) {
    case '?':
    case 'h':
    case 'H':
      println();
      println("+/-/=  zomm in/out/100%");
      println("h/?    help");
      println("c      clear data");
      println("m      toggle mode (erase on new sweep on/off");
      println("l      distance labels on/off");
      println("m      toggle mode (erase on new sweep on/off");
      println("p      pause/restart");
      println();
      break;
    case '+': 
      zoomFactor *= 1.1;
      println("Zoom+: "+zoomFactor*100.0+"%");
      clear();
      drawGrid();
      break;
    case '-': 
      zoomFactor /= 1.1;
      println("Zoom-: "+zoomFactor*100.0+"%");
      clear();
      drawGrid();
      break;
    case '=': 
      zoomFactor = 1.0;
      println("Zoom=: "+zoomFactor*100.0+"%");
      clear();
      drawGrid();
      break;
    case 'c':
    case 'C':
      println("Clearing data");
      clear();
      sharedData.clear();
      drawGrid();
      break;
    case 'm':
    case 'M':
      eraseData = !eraseData;
      if (eraseData) {
        sharedData.clear();
      }
      println("Toggle erase data on sweep. Now: " + eraseData);
      drawGrid();
      break;
    case 'l':
    case 'L':
      labels_on = !labels_on;
      println("Toggle show labels. Now: " + labels_on);
      drawGrid();
      break;
    case 'p':
    case 'P':
      looping  = !looping;  
      break;
  }
}
