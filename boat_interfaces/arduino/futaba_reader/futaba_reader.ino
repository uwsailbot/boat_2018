#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#define USE_USBCON

#include <ros.h>
#include <boat_msgs/Joy.h>

#define CH_1_PIN 5 // RUDDER
#define CH_3_PIN 6 // SAIL 
#define CH_5_PIN 9 // SWITCH_A
#define CH_6_PIN 10 // VR
#define BUFFER_SIZE 5

int ch_1_buf [BUFFER_SIZE] = {0};
int ch_3_buf [BUFFER_SIZE] = {0};
int ch_6_buf [BUFFER_SIZE] = {0};
int counter = 0;

ros::NodeHandle nh;
boat_msgs::Joy joy;
ros::Publisher joy_pub("joy", &joy);

void setup() {
  pinMode(CH_1_PIN, INPUT);
  pinMode(CH_3_PIN, INPUT);
  pinMode(CH_5_PIN, INPUT);
  pinMode(CH_6_PIN, INPUT);

  //Serial.begin(115200);
  
  nh.initNode();
  nh.advertise(joy_pub);
}

void loop() {
  // Read PWM pins
  ch_1_buf[counter] = pulseIn(CH_1_PIN, HIGH, 10000);
  ch_3_buf[counter] = pulseIn(CH_3_PIN, HIGH, 10000);
  int ch_5_val = pulseIn(CH_5_PIN, HIGH, 10000);
  ch_6_buf[counter] = pulseIn(CH_6_PIN, HIGH, 10000);
  
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
  joy.right_stick_x = average(ch_1_buf) - 995;
  joy.left_stick_y = average(ch_3_buf) - 995 ;
  joy.switch_a = switch_state;
  joy.vr = average(ch_6_buf) - 870;
/*
  Serial.print("rx: ");
  Serial.print(joy.right_stick_x);
  Serial.print("\tly: ");
  Serial.print(joy.left_stick_y);
  Serial.print("\tsw: ");
  Serial.println(joy.switch_a);
  */
  // Check that controller values are valid (end up being -1000 if controller is not on)
  // Check uses -160 because the values can briefly drop to -1 and trim can set them as low as -150
  if (joy.right_stick_x > -160 && joy.left_stick_y > -160 && joy.vr > -25 && joy.right_stick_x < 3000 && joy.right_stick_y < 3000 && joy.vr < 3000){
    joy_pub.publish(&joy);
    counter = (counter + 1) % 5;
  }
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
  
