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
#include <boat_msgs/Joy.h>

// Defines for Futaba receiver
#define CH_1_PIN 5 // RUDDER
#define CH_3_PIN 6 // SAIL 
#define CH_5_PIN 7 // SWITCH_A
#define CH_6_PIN 8 // VR
#define BUFFER_SIZE 5

// The pin the wind vane sensor is connected to
#define WIND_VANE_PIN (A0)  
// Define serial port for the gps
#define mySerial Serial1

// Define servo pins
#define RUDDER_PIN1 2
#define RUDDER_PIN2 3
#define WINCH_PIN 4
const int anemometerInterval = 100; //ms between anemometer reads

ros::NodeHandle nh;
std_msgs::Float32 winddir;
boat_msgs::GPS gpsData;
boat_msgs::Joy joy;

Adafruit_GPS GPS(&mySerial);
Servo servo_rudder1;
Servo servo_rudder2;
Servo servo_winch;
float calWindDirection;
float lastWindDirection = 0;
uint32_t GPS_timer = millis();
float last_lat = 0, last_long = 0;
int last_status = 0;
unsigned long previousMillis = 0;
int ch_1_buf [BUFFER_SIZE] = {0};
int ch_3_buf [BUFFER_SIZE] = {0};
int ch_6_buf [BUFFER_SIZE] = {0};
int counter = 0;


float mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh){
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

void rudder_cb( const std_msgs::Float32& cmd_msg){
    // Write the received data directly to both rudder servos
    servo_rudder1.write(cmd_msg.data);
    servo_rudder2.write(cmd_msg.data);
}

void winch_cb( const std_msgs::Int32& pos_msg){
    // Write received data to the winch
    servo_winch.writeMicroseconds(pos_msg.data);
}

ros::Subscriber<std_msgs::Float32> sub_rudder("rudder", rudder_cb);
ros::Subscriber<std_msgs::Int32> sub_winch("winch", winch_cb);
ros::Publisher anemometer("anemometer", &winddir);
ros::Publisher gps("gps_raw", &gpsData);
ros::Publisher joy_pub("joy", &joy);

void setup(){
    // setup subscribers 
    nh.initNode();
    nh.subscribe(sub_rudder);
    nh.subscribe(sub_winch);
    nh.advertise(anemometer);
    nh.advertise(gps);
    nh.advertise(joy_pub);

    servo_rudder1.attach(RUDDER_PIN1); // attach rudder1
    servo_rudder2.attach(RUDDER_PIN2); // attach rudder2
    servo_winch.attach(WINCH_PIN); // attach winch
    pinMode(CH_1_PIN, INPUT);
    pinMode(CH_3_PIN, INPUT);
    pinMode(CH_5_PIN, INPUT);
    pinMode(CH_6_PIN, INPUT);
    pinMode(WIND_VANE_PIN, INPUT);

    GPS.begin(9600);
    // Tell gps to send RMC (recommended minimum) and GGA (fix data) data
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    //Enable WAAS
    GPS.sendCommand("$PMTK313,1*2E");
    GPS.sendCommand("$PMTK301,2*2E");
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);   // 1 Hz update rate

    bubbleSortlookupTable();
}

void loop(){
    // Read Anemometer
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
    
    // Read GPS
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
      
    // Read PWM pins on receiver for controller state  
    ch_1_buf[counter] = pulseIn(CH_1_PIN, HIGH);
    ch_3_buf[counter] = pulseIn(CH_3_PIN, HIGH);
    int ch_5_val = pulseIn(CH_5_PIN, HIGH);
    ch_6_buf[counter] = pulseIn(CH_6_PIN, HIGH);
  
    // Lookup table for switch position
    int switch_state = 0;
    if (ch_5_val < 1300){
        switch_state = joy.SWITCH_UP;
    }
    else if(ch_5_val > 1700){
        switch_state = joy.SWITCH_DOWN;
    }
    else{
        switch_state = joy.SWITCH_MIDDLE;
    }
  
    // Write averaged values to ROS, pwm oscillates slightly, so we average the readings in a buffer
    joy.right_stick_x = average(ch_1_buf) - 1000;
    joy.left_stick_y = average(ch_3_buf) - 1000 ;
    joy.switch_a = switch_state;
    joy.vr = average(ch_6_buf) - 875;
  
    // Check that controller values are valid (end up being -1000 if controller is not on)
    // Check uses -160 because the values can briefly drop to -1 and trim can set them as low as -150
    if (joy.right_stick_x > -160 && joy.left_stick_y > -160 && joy.vr > -25){
        joy_pub.publish(&joy);
        counter = (counter + 1) % 5;
    }
    
    nh.spinOnce();
}

