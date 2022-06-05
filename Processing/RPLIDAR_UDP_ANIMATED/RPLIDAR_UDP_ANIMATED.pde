import java.net.*;
import java.io.*;
import java.util.Arrays;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

final float mmPerPixel = 10.0;
final float maxDistMm = 15000.0;  // 12m
final float MOVED_FACTOR = 0.05;  // If displacement > 5% don't average, set new distance directly 
float zoomFactor = 1.0; 
boolean labels_on = false;  // show lables (slow)
boolean eraseData = false;  // erase data every cycle


PFont markerFont;  // Global font variable

class DistanceData {
  public float average;
  public float max;
  public float min;
  public long npoints;
}

class AngleData {
    // To do: keep average quality
    private float sum = 0.0;
    private long count = 0;
    private float maxd = 0;
    private float mind = 0;
    boolean dirty = false;
    private final Object lock = new Object();

    public void addDist(float d) {
        synchronized (lock) {
            if (count > 0) {
              float curd = sum/count;
              if (Math.abs((curd-d)/curd) > MOVED_FACTOR)
              {
                sum = d;
                count = 1;
                return;
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
          dirty = true;
        }
    }
    public boolean getData(DistanceData dd, boolean eraseit) {
        synchronized (lock) {
            dd.average = sum/count;
            dd.max = maxd;
            dd.min = mind;
            dd.npoints = count;
            if (eraseit) {
              this.erase();
            }
            return dirty;
        }
    }
    public boolean getData(DistanceData dd) {
      return getData(dd, false);
    }
    public long getCount() {
        synchronized (lock) {
            return count;
        }
    }
    public void erase() {
        synchronized (lock) {
            count = 0;
            sum = 0.0;
            dirty = false;
        }
    }
}

class LidarData {
  private AngleData[] data = new AngleData[360];
  
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

  private int getangle(float angle) {
    int iangle = (int) Math.floor(angle);
    if (angle<0) iangle = 0;
    if (iangle>=360) iangle = 359;
    return iangle;
  }

  public void set(float angle, float distance, int quality, long sequence) {
    packets++;
    if (firstpacket) {
      firstseq = sequence;
      firstpacket = false;
    }
    lastseq = sequence;
    if (quality > 0) {
      int i = getangle(angle);
      data[i].addDist(distance);
    }
  }
  
  public boolean get(float angle, DistanceData d, boolean erase) {
    int i = getangle(angle);
    return  data[i].getData(d, erase);
  }
  
  public boolean get(float angle, DistanceData d) {
    return get(angle, d, false);
  }
  
  public float  getLoss() {
    float packetssent = lastseq - firstseq; 
    float packetslost = packetssent - packets;
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
    //socket = new DatagramSocket(4211, InetAddress.getByName("172.22.39.90")); // Set your port here
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
void drawGridSegment(float angle1, float angle2) {
  //
  // Clear the segment
  fill(0,0,0);
  noStroke();
  centeredArc(maxDistMm, angle1, angle2);
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
  String label2 = String.format(" %d", sharedData.getPackets()) + " pkts  " + String.format("%.2f", sharedData.getLoss()) + "% loss"+ " ZOOM: " + String.format("%.3f ", zoomFactor)+"%  ";
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
  drawGridSegment(0.0, TWO_PI);
}

// *************************************************************************************************    

void setup() {
  size(900, 900);
  background(0);  //107 GREY
  frameRate(30);
  markerFont = createFont("Arial", 16, true);  // Loading font

  drawGrid();
  //
  // Start UDP listener
  //
  
  thread("LidarUDPListener");

}


// *************************************************************************************************    
// 
// loop and draw points

static final color[] coltab = new color[] { #A0A0A0, #C0A0A0, #F0A0A0, #FFB0B0, #FFF0B0, #FFFFC0, #FFFFFF, #FFFFFF, #FFFFFF, #FFFFFF, #FFFFFF, #FFFFFF, #FFFFFF, #FFFFFF, #FFFFFF };

float angle = 0.0;  // degrees
DistanceData dd = new DistanceData();

void draw() {
  // Draw al points
  for (angle = 0.0; angle < 360.0; angle += 1.0) {

    boolean isDirty = sharedData.get(angle, dd, eraseData);
    long  np = dd.npoints;

    float rads = (float) Math.toRadians(angle);
    float rads2 = (float) Math.toRadians(angle+1);
    drawGridSegment(rads, rads2);

    if (isDirty)
    {
      stroke(120,120,0);
    } else {
      stroke(0,255,0);
    }
    centeredArcPx (width/1.1, rads+0.01, rads2-0.01);
 
    /*------------------------------------------------------------
    // Mark min & max
    float sin_angle = (float) Math.sin(rads);
    float cos_angle = (float) Math.cos(rads);
   
    float xmin = Xworld2window (dd.min * cos_angle);
    float ymin = Yworld2window (dd.min * sin_angle);
  
    float xmax = Xworld2window (dd.max * cos_angle);
    float ymax = Yworld2window (dd.max * sin_angle);
  
    //  float xavg = Xworld2window (dd.average * cos_angle);
    //  float yavg = Yworld2window (dd.average * sin_angle);
    
    strokeWeight(1);
    stroke(11,33,11);
    line(xmin, ymin, xmax, ymax);
    -------------------------------------------------------------*/
  //
  //
  // draw arc
  //
    if (eraseData) {
      long c = np;
      if (c > 14) c = 14; 
      color col = coltab[(int)c];
      stroke(col);
    } else {
      long c2 = np * 10;
      long c3 = 0;
      if (c2 > 255) {
        c2 = 255;
        c3 = Math.max(np-255, 255);
      }
      stroke(c2, c3, 10);
    }
    strokeWeight(4);
    centeredArc(dd.average, rads, rads2);
    
  } // for angle
} // draw

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
    case 'c':
    case 'C':
      println("Clearing data");
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
    case 's':
    case 'S':
      looping  = !looping;  
      break;
  }
}
