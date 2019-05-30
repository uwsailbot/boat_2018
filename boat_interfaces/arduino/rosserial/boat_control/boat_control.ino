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

#define DEBUG_SERIAL (false) //Output GPS data to Serial2 if true

// The pin the wind vane sensor is connected to
#define WIND_VANE_PIN (A0)

// Define servo pins
#define RUDDER_PIN1 2
#define RUDDER_PIN2 3
#define WINCH_PIN 4
const int anemometerInterval = 100; //ms between anemometer reads
const int compassInterval = 20;

ros::NodeHandle nh;
std_msgs::Float32 winddir;
boat_msgs::GPS gpsData;
std_msgs::Float32 compassData;

Adafruit_GPS GPS(&Serial1);
Servo servo_rudder1;
Servo servo_rudder2;
Servo servo_winch;
float calWindDirection;
float lastWindDirection = 0;
float prevCompass = 0;
uint32_t GPS_timer = millis();
float last_lat = 0, last_long = 0;
float last_track = 0, last_speed = 0;
uint32_t compassTimer = millis();
uint32_t anemometerTimer = millis();

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
ros::Publisher compass("compass", &compassData);

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
    nh.advertise(compass);

    servo_rudder1.attach(RUDDER_PIN1); // attach rudder1
    servo_rudder2.attach(RUDDER_PIN2); // attach rudder2
    servo_winch.attach(WINCH_PIN); // attach winch
    pinMode(WIND_VANE_PIN, INPUT);

    GPS.begin(9600);
    // Tell gps to send RMC (recommended minimum) and GGA (fix data) data
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    //Enable WAAS
    GPS.sendCommand("$PMTK313,1*2E");
    GPS.sendCommand("$PMTK301,2*2E");
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);   // 2 Hz update rate
    useInterrupt(true);

    Serial3.begin(9600, SERIAL_8N2);

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
    if ((millis() - anemometerTimer) > anemometerInterval){
        // Map 0-1023 ADC value to 0-360
        anemometerTimer = millis();
        calWindDirection = mapf(analogRead(WIND_VANE_PIN), 0.0, 1023.0, 0.0, 360.0);

        // Ensure the wind direction is between 0 and 360
        while(calWindDirection >= 360.0)
            calWindDirection = calWindDirection - 360.0;
        while(calWindDirection < 0)
            calWindDirection = calWindDirection + 360;

        calWindDirection = medianFilter(exponentialFilter(calWindDirection));

        // Convert to true value through lookup table
        calWindDirection = lookupTrueWindDirection(calWindDirection);

        // Only update the topic if change greater than 2 degrees.
        if(abs(calWindDirection - lastWindDirection) >= 2.0)
        {
             lastWindDirection = calWindDirection;
             winddir.data = calWindDirection;
             anemometer.publish(&winddir);
        }
    }

    //Read compass
    /*if ((millis() - compassTimer) > compassInterval){
        compassTimer = millis();
        float data = 0;
        Serial3.write(0x13); // Request compass heading in two bytes
        while (Serial3.available()<2);
        data = Serial3.read() << 8;
        data += Serial3.read();
        data = data/10.0;
        // Only publish if change in heading is greater than a degree
        if (abs(data - prevCompass) >= 1.0){
            prevCompass = data;
            compassData.data = (data + 90) % 360;
            compass.publish(&compassData);
        }
    }*/


    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
            return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    // approximately publish at 1 HZ the current stats
    if (millis() - GPS_timer > 500) {
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
          last_track = gpsData.track;
          last_speed = gpsData.speed;
    }

    nh.spinOnce();
}
