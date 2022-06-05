#ifndef _RPLIDAR_DATA_H
#define _RPLIDAR_DATA_H

#include <Arduino.h>


static const float DIVISIONS_PER_DEGREE           = 4;      // Angular resolution = 0.25 degrees
static const unsigned int DIVISIONS_TOTAL         = (360 * DIVISIONS_PER_DEGREE);
static const float MOVED_FACTOR                   = 0.05;   // consider the point has moved if distance changes more than this factor (5% - 5cm at 1 m)
static const unsigned long MS_TO_FORGET           = 3000;   //   3s  - Miliseconds to forget a point if not seen
static const unsigned long MS_TRANSIENT_POI       =  200;   // 200ms - Wait this time to switch to a new POI to filter out transients
// Two points leass than 2.1 degrees apart and 5 cm apart in the radial dimension are considered 'near' to switch POI
static const float ANGULAR_NEAR                   = 3;    // degrees      
static const float RADIAL_NEAR                    = 200.0;  // mm 20cm
// The further apart from RPLIDar, the noisier the signal. Put a limit on POI detection
static const float DEFAULT_MAX_POI_DIST           = 5000;   // mm, 5m
// Absolute max distance
static const float MAX_DIST                       = 15000;  // 15m

//==============================================================================
// Mutexes
/* TO DO : implement for ESP32 and ES8266 if multithreading
 * 
 * https://github.com/raburton/esp8266/blob/master/mutex/mutex.c
 * https://forum.arduino.cc/t/how-to-use-mutex/964924
 * 
 */
/// </summary>
class MutexLock {
private:
    int dummy;
public:
    MutexLock() { dummy = 0;  }
};

static bool get_lock(MutexLock lock) {
    return true;
}


//==============================================================================
class DistanceData {
public:
    float angle;
    float distance;
    unsigned long times_seen;
    unsigned long last_seen;
    unsigned long last_moved;

    DistanceData() {
        clear();
    }

    void clear() {
        angle = -1.0;
        distance = 1.0e9;
        times_seen = 0;
        last_seen = 0;
        last_moved = 0;
    };

};

//==============================================================================
class AngleData {
    // To do: keep average quality?
private:
    float sum = 0.0;
    unsigned long count = 0;
    unsigned long last_seen_ms = 0;
    unsigned long when_moved_ms = 0;
    MutexLock lock;

public:
    // ------------------------------------------------------------------------
    // Record the distance (for the angle coresponding to 'this') as an average
    // of recent mesasurements.
    // Returns true if point moved
    // If the distance is relatively big (MOVED_FACTOR * average), then the new
    // position is recorded and a displacement of the point is assumed. Otherwise,
    // the distance is added to a running sum to compute the new average. 
    bool addDist(float d) {
        bool moved = false;
        if (get_lock(lock)) {
            if (count > 0) {
                float dist = sum / count;
                if ( (abs(d - dist) / dist) > MOVED_FACTOR) {
                    when_moved_ms = millis();
                    sum = 0;
                    count = 0;
                    moved = true;
                }
            }
            sum += d;
            count++;
            last_seen_ms = millis();
        } // lock
        return moved;
    }
    // ------------------------------------------------------------------------
    // Get the data for a given point
    // Returns true if there is valid data recorded
    // Optionally, erase (invalidate) the data
    bool getData(DistanceData& dd, bool eraseit) {
        if (get_lock(lock)) {
            dd.distance = sum / count;
            dd.times_seen = count;
            dd.last_seen = last_seen_ms;
            dd.last_moved = when_moved_ms;
            bool has_data = count > 0;
            if (eraseit) {
                this->erase();
            }
            return has_data;
        } // lock
        return false;
    }

    // ------------------------------------------------------------------------
    // Get the data for a given point, no erase option
    bool getData(DistanceData& dd) {
        return getData(dd, false);
    }

    // ------------------------------------------------------------------------
    // Erase the data for a given point
    void erase() {
        if (get_lock(lock)) {
            count = 0;
            sum = 0.0;
            last_seen_ms = 0;
            when_moved_ms = 0;
        } // lock
    }
};

//==============================================================================
class LidarData {
    // Keeps an array of distances for each angular position.
    // The number of points must be a multiple (or integer divisor) of 360,
    // thus establishing the angular resolution. (see DIVISIONS_PER_DEGREE). 
    // Additionally maintains the data of the point whose movement was last
    // detected (last_moved_point) as well as some statistics about data 
    // points (packets) received and lost based on th sequence numbers. 
    // Finally, the last_moved_point data is used to compute a 'point of
    //  interest'. This is a point that has moved recently, is nearer than
    //  other points that moved rcently and has some inertia to avoid jumps.
private:
    AngleData lidar_data[DIVISIONS_TOTAL];
    DistanceData last_moved_point;
    DistanceData point_of_interest; 
    float max_poi_dist = DEFAULT_MAX_POI_DIST;
    // Get the index of the angle
    int getIndex(float angle) {
        int iangle = (int)round(angle * DIVISIONS_PER_DEGREE);
        if (angle < 0) iangle = 0;
        if (iangle >= DIVISIONS_TOTAL) iangle = (DIVISIONS_TOTAL) - 1;
        return iangle;
    }

public:
    bool    firstpacket = true;
    float   pctloss = 0.;
    long    packets = 0;
    long    firstseq = 0;
    long    lastseq = 0;
    long    loop_index = 0;



    // ------------------------------------------------------------------------
    // constructor
    LidarData() {
        clear();
    }
    // Initialize
    void clear() {
        for (int i = 0; i < DIVISIONS_TOTAL; i++) {
            lidar_data[i].erase();
            firstpacket = true;
            pctloss = 0;
            packets = 0;
            firstseq = 0;
            lastseq = 0;
            last_moved_point.clear();
            point_of_interest.clear();
        }
    }
    // ------------------------------------------------------------------------
    // Change max POI distance 
    void setMaxPOIDist(float max) {
        if ( (max > 0) && (max <= MAX_DIST) )
        {
            max_poi_dist = max;
        }
    }
    // ------------------------------------------------------------------------
    // Set the position data for a given angle. Returns true if the point has moved
    // Keeps data position in `last_moved_angle' 
    bool set(float angle, float distance, int quality, long sequence) {
        packets++;
        if (firstpacket) {
            firstseq = sequence;
            firstpacket = false;
        }
        lastseq = sequence;
        int i = getIndex(angle);
        bool moved = lidar_data[i].addDist(distance);
        if (moved) {
            lidar_data[i].getData(last_moved_point);
            last_moved_point.angle = angle;
        }
        return moved;
    }
    // ------------------------------------------------------------------------
    // Get data for a given angle
    bool get(float angle, DistanceData& d, bool erase) {
        int i = getIndex(angle);
        return  lidar_data[i].getData(d, erase);
    }

    bool get(float angle, DistanceData& d) {
        return get(angle, d, false);
    }
    // ------------------------------------------------------------------------
    // 2022-05-28 Simplified criteria. POI = LMP with some delay to 
    //            avoid transients
    // Get point data for the point of interest (most recently moved point, LMP)
    // Returns true if there is a point of interest, false otherwise
    bool getPointOfInterest(DistanceData &dd) {
        if (point_of_interest.angle < 0) {        // No previous POI
            if (last_moved_point.angle < 0) {     // And no recently moved point
                dd.clear();                       // No current POI
                return false;
            } 
            // No POI but have LMP
            point_of_interest = last_moved_point;   // First POI
            dd = point_of_interest;
            return true;
        }
        // Have POI. If LMP is not near enough, keep current POI
        if (last_moved_point.distance > max_poi_dist) {
            dd = point_of_interest;
            return true;
        }
        // if the last moved point is 'near' the POI, set the LMP as the new POI
        float angle_diff =abs(point_of_interest.angle - last_moved_point.angle);
        float dist_diff = abs(point_of_interest.distance - last_moved_point.distance);
        if ( (angle_diff <= ANGULAR_NEAR) && (dist_diff <= RADIAL_NEAR) ) {
            point_of_interest = last_moved_point;
            dd = point_of_interest;
            return true;
        }
        // POI and LMP are relatively apart.
        // If last POI should be forgotten, use current LMP now
        unsigned long ms = millis();
        if ((ms - point_of_interest.last_seen)  > MS_TO_FORGET) {
            point_of_interest = last_moved_point;
            dd = point_of_interest;
            return true;
        }
        // Else wait MS_TRANSIENT_POI ms before setting new POI to filter out transients.
        if ((ms - last_moved_point.last_moved) > MS_TRANSIENT_POI) {
                point_of_interest = last_moved_point;
                dd = point_of_interest;
                return true;
        }
        dd = point_of_interest;
        return true;
    }
    // ------------------------------------------------------------------------
    // Get percentage of packets lost
    float  getLoss() {
        float packetssent = lastseq - firstseq;
        float packetslost = packetssent - packets;
        if (packetssent == 0) return 0.0;
        pctloss = 100.0 * packetslost / packetssent;
        return pctloss;
    }
    // ------------------------------------------------------------------------
    // Get number of packets received
    long getPackets() {
        return packets;
    }
    // Loop: call this function frequently to update
    void loop() {
        DistanceData dd;
        if ( !lidar_data[loop_index].getData(dd) ) {
            if ( (millis() - dd.last_seen) > MS_TO_FORGET) {
                lidar_data[loop_index].erase();
            }
        }
        loop_index = ++loop_index % DIVISIONS_TOTAL;
    }
};

#endif  // _RPLIDAR_DATA_H