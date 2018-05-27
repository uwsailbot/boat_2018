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

#define DEBUG_SERIAL (false) //Output GPS data to Serial2 if true

// Defines for Futaba receiver
#define CH_1_PIN 8 // RUDDER
#define CH_3_PIN 9 // SAIL 
#define CH_5_PIN 10 // SWITCH_A
#define CH_6_PIN 11 // VR
#define BUFFER_SIZE 5

// The pin the wind vane sensor is connected to
#define WIND_VANE_PIN (A0)

// Define servo pins
#define RUDDER_PIN1 2
#define RUDDER_PIN2 3
#define WINCH_PIN 4
const int anemometerInterval = 100; //ms between anemometer reads

ros::NodeHandle nh;
std_msgs::Float32 winddir;
boat_msgs::GPS gpsData;
boat_msgs::Joy joy;

Adafruit_GPS GPS(&Serial1);
Servo servo_rudder1;
Servo servo_rudder2;
Servo servo_winch;
float calWindDirection;
float lastWindDirection = 0;
uint32_t GPS_timer = millis();
float last_lat = 0, last_long = 0;
float last_track = 0, last_speed = 0;
unsigned long previousMillis = 0;
int ch_1_buf [BUFFER_SIZE] = {0};
int ch_3_buf [BUFFER_SIZE] = {0};
int ch_6_buf [BUFFER_SIZE] = {0};
int counter = 0;

int average(int buffer[]){
    int sum = 0;
    int valid_data_counter = 0;
    for(int i = 0; i < BUFFER_SIZE; i++){
        sum += buffer[i];
        if (buffer[i] != 0){
            valid_data_counter ++;
        }
    }
    return sum/valid_data_counter;
}

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
    if(DEBUG_SERIAL){
        Serial2.begin(115200);
    }
    
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
    
    // Uncomment following lines to increase GPS baud rate for higher update frequency
    // GPS.sendCommand("PMTK251,115200*27");
    // GPS.begin(115200);
    
    // Tell gps to send RMC (recommended minimum) and GGA (fix data) data
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    //Enable WAAS
    GPS.sendCommand("$PMTK313,1*2E");
    GPS.sendCommand("$PMTK301,2*2E");
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    useInterrupt(true);


    bubbleSortlookupTable();
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
  }
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
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
            return;  // we can fail to parse a sentence in which case we should just wait for another
    }  
    // approximately publish at 1 HZ the current stats
    if (millis() - GPS_timer > 1000) {
      GPS_timer = millis(); // reset the timer
      
      if (DEBUG_SERIAL){
          Serial2.print("\nTime: ");
          Serial2.print(GPS.hour, DEC); Serial2.print(':');
          Serial2.print(GPS.minute, DEC); Serial2.print(':');
          Serial2.print(GPS.seconds, DEC); Serial2.print('.');
          Serial2.println(GPS.milliseconds);
          Serial2.print("Date: ");
          Serial2.print(GPS.day, DEC); Serial2.print('/');
          Serial2.print(GPS.month, DEC); Serial2.print("/20");
          Serial2.println(GPS.year, DEC);
          Serial2.print("Fix: "); Serial2.print((int)GPS.fix);
          Serial2.print(" quality: "); Serial2.println((int)GPS.fixquality); 
          
          if (GPS.fix) {
              Serial2.print("Location: ");
              Serial2.print(GPS.latitude, 4); Serial2.print(GPS.lat);
              Serial2.print(", "); 
              Serial2.print(GPS.longitude, 4); Serial2.println(GPS.lon);
              Serial2.print("Location (in degrees, works with Google Maps): ");
              Serial2.print(GPS.latitudeDegrees, 4);
              Serial2.print(", "); 
              Serial2.println(GPS.longitudeDegrees, 4);
          
              Serial2.print("Speed (knots): "); Serial2.println(GPS.speed);
              Serial2.print("Angle: "); Serial2.println(GPS.angle);
              Serial2.print("Altitude: "); Serial2.println(GPS.altitude);
              Serial2.print("Satellites: "); Serial2.println((int)GPS.satellites);
          }
      }
      
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
    }
     //only publish is readings have changes
    if (((last_lat != gpsData.latitude) ||
          (last_long != gpsData.longitude) || 
          (last_track != gpsData.track) || 
          (last_speed != gpsData.speed)) &&
          gpsData.status != gpsData.STATUS_NO_FIX){
          gps.publish(&gpsData);
      
          last_lat = gpsData.latitude;
          last_long = gpsData.longitude;
      
    }  
    useInterrupt(false);
    // Read PWM pins on receiver for controller state  
    ch_1_buf[counter] = pulseIn(CH_1_PIN, HIGH, 10000);
    ch_3_buf[counter] = pulseIn(CH_3_PIN, HIGH, 10000);
    int ch_5_val = pulseIn(CH_5_PIN, HIGH, 10000);
    ch_6_buf[counter] = pulseIn(CH_6_PIN, HIGH, 10000);
    useInterrupt(true);
    //TIMSK2 = 0x02;  
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

