#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <boat_msgs/GPS.h>
#include <math.h>

// Pins the gps is attached to
#define GPS_TX_PIN 3
#define GPS_RX_PIN 2

// The pin the wind vane sensor is connected to
#define WIND_VANE_PIN (A4)       

const int anemometerInterval = 100; //ms between anemometer reads

ros::NodeHandle nh;
std_msgs::Float32 winddir;
boat_msgs::GPS gpsData;

SoftwareSerial mySerial(GPS_TX_PIN, GPS_RX_PIN);
Adafruit_GPS GPS(&mySerial);
Servo servo_rudder;
Servo servo_winch;
float calWindDirection;
float lastWindDirection = 0;
uint32_t GPS_timer = millis();
float last_lat = 0, last_long = 0;
int last_status = 0;
unsigned long previousMillis = 0;


float mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh){
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

void rudder_cb( const std_msgs::Float32& cmd_msg){
    //Write the received data directly to the rudder servo
    servo_rudder.write(cmd_msg.data);  
}

void winch_cb( const std_msgs::Int32& pos_msg){
    //map the rotation (0-2160 degrees) to a motor value between 1000 and 2000
    int position_msg = map(pos_msg.data, 0, 2160, 1000, 2000);
    servo_winch.writeMicroseconds(position_msg);
}

ros::Subscriber<std_msgs::Float32> sub_rudder("rudder", rudder_cb);
ros::Subscriber<std_msgs::Int32> sub_winch("winch", winch_cb);
ros::Publisher anemometer("anemometer", &winddir);
ros::Publisher gps("gps_raw", &gpsData);

void setup(){
    // setup subscribers 
    nh.initNode();
    nh.subscribe(sub_rudder);
    nh.subscribe(sub_winch);
    nh.advertise(anemometer);
    nh.advertise(gps);
  
    servo_rudder.attach(3); // attach rudder to pin 3
    servo_winch.attach(4); // attach winch to pin 4

    GPS.begin(9600);
    // Tell gps to send RMC (recommended minimum) and GGA (fix data) data
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    //Enable WAAS
    GPS.sendCommand("$PMTK313,1*2E");
    GPS.sendCommand("$PMTK301,2*2E");
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

    bubbleSortlookupTable();
    
    pinMode(WIND_VANE_PIN, INPUT);
}

void loop(){
    if ((millis() - previousMillis) > anemometerInterval){
        // Map 0-1023 ADC value to 0-360
        calWindDirection = mapf(analogRead(WIND_VANE_PIN), 0.0, 1023.0, 0.0, 360.0);
      
        // Ensure the wind direction is between 0 and 360
        while(calWindDirection >= 360.0)
            calWindDirection = calWindDirection - 360.0;
        while(calWindDirection < 0)
            calWindDirection = calWindDirection + 360;
  
        calWindDirection = medianFilter(exponentialFilter(calWindDirection));
    
        // Convert to true value through lookup table
        calWindDirection = lookupTrueWindDirection(calWindDirection);
        
        // Only update the topic if change greater than 5 degrees. 
        if(abs(calWindDirection - lastWindDirection) > 5)
        { 
             lastWindDirection = calWindDirection;
             winddir.data = calWindDirection;
             anemometer.publish(&winddir);       
        }
    }

    GPS.read();

    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
            return;  // we can fail to parse a sentence in which case we should just wait for another
    }

    // approximately every second publish the current stats
    if (millis() - GPS_timer > 1000) { 
      GPS_timer = millis(); // reset the timer
    
      if (GPS.fix) {
          if (GPS.fixquality == 1){
              gpsData.status = gpsData.STATUS_FIX;
          }else if (GPS.fixquality == 2){
              gpsData.status = gpsData.STATUS_WAAS_FIX;
          }
          gpsData.satellites_used = GPS.satellites;
          gpsData.latitude = GPS.latitudeDegrees;
          gpsData.longitude = GPS.longitudeDegrees;
          gpsData.altitude = GPS.altitude;
          gpsData.track = GPS.angle;
          gpsData.speed = GPS.speed;
          gpsData.hdop = GPS.HDOP; // horizontal diltuion of precision
      }else{
          gpsData.status = gpsData.STATUS_NO_FIX;
      }

     //only publish is readings have changes
     if ((last_lat != gpsData.latitude) ||
          (last_long != gpsData.longitude) ||
          (last_status != gpsData.status)){
          gps.publish(&gpsData);
      
          last_lat = gpsData.latitude;
          last_long = gpsData.longitude;
          last_status = gpsData.status;
      }
    
    }
  
    nh.spinOnce();
}

