#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#define USE_USBCON

#include <ros.h>
#include <boat_msgs/Joy.h>

#define JOY_PIN (A0) // RUDDER
#define BUFFER_SIZE 5

int buf [BUFFER_SIZE] = {0};
int counter = 0;

ros::NodeHandle nh;
boat_msgs::Joy joy;
ros::Publisher joy_pub("joy", &joy);

void setup() {
  pinMode(JOY_PIN, INPUT);

  Serial.begin(115200);
  
  nh.initNode();
  nh.advertise(joy_pub);
}

void loop() {
  buf[counter] = analogRead(JOY_PIN);
  
  // Write averaged values to ROS
  joy.right_stick_x = average(buf);
  joy.left_stick_y = 0;
  joy.switch_a = joy.SWITCH_MIDDLE;
  joy.vr = 0;

  Serial.print("rx: ");
  Serial.print(joy.right_stick_x);
  Serial.print("\tly: ");
  Serial.print(joy.left_stick_y);
  Serial.print("\tsw: ");
  Serial.println(joy.switch_a);
  
  joy_pub.publish(&joy);
  counter = (counter + 1) % 5;
  nh.spinOnce();
  delay(10);
}

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
  
