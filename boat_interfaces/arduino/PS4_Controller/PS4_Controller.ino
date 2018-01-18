#include <PS4USB.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif


byte buttons[11];
byte axes[8];
USB Usb;
PS4USB PS4(&Usb);

void setup() {
  Serial.begin(9600);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
}
void loop() {
  Usb.Task();
  if (PS4.connected()) {
    // Read left and right joysticks
    if (PS4.getAnalogHat(LeftHatX) > 137 || PS4.getAnalogHat(LeftHatX) < 117) {
      if (PS4.getAnalogHat(LeftHatX) > 137)
        PS4.setRumbleOn(PS4.getAnalogHat(LeftHatX) - 128, PS4.getAnalogHat(LeftHatX) - 128);
      else
        PS4.setRumbleOn(128 - PS4.getAnalogHat(LeftHatX), 128 - PS4.getAnalogHat(LeftHatX));
      axes[0] = float(PS4.getAnalogHat(LeftHatX));
    }
    else {
      axes[0] = 128;
      PS4.setRumbleOn(0, 0); // Vibrate based on left joystick position
    }
    if (PS4.getAnalogHat(LeftHatY) > 137 || PS4.getAnalogHat(LeftHatY) < 117)
      axes[1] = PS4.getAnalogHat(LeftHatY);
    else
      axes[1] = 128;
    if (PS4.getAnalogHat(RightHatX) > 137 || PS4.getAnalogHat(RightHatX) < 117)
      axes[3] = PS4.getAnalogHat(RightHatX);
    else
      axes[3] = 128;
    if (PS4.getAnalogHat(RightHatY) > 137 || PS4.getAnalogHat(RightHatY) < 117)
      axes[4] = PS4.getAnalogHat(RightHatY);
    else
      axes[4] = 128;

    // Read L2 and R2
    axes[2] = float(PS4.getAnalogButton(L2));
    axes[5] = float(PS4.getAnalogButton(R2));

    //Read up and down D Pad button
    if (PS4.getButtonPress(UP)) {
      PS4.setLed(Blue);
      axes[7] = 2;
    }
    else if (PS4.getButtonPress(DOWN)) {
      PS4.setLed(Red);
      axes[7] = 0;
    }
    else
      axes[7] = 1;
      
    // Read left and right D Pad button
    if (PS4.getButtonPress(LEFT)) {
      PS4.setLed(Yellow);
      axes[6] = 2;
    }
    else if (PS4.getButtonPress(RIGHT)) {
      PS4.setLed(Green);
      axes[6] = 0;
    }
    else
      axes[6] = 1;

    // Read options and share buttons
    if (PS4.getButtonPress(OPTIONS)) {
      buttons[7] = 1;
    }
    else
      buttons[7] = 0;
    if (PS4.getButtonPress(SHARE)) {
      buttons[6] = 1;
    }
    else
      buttons[6] = 0;

      
    // Read the rest of the push buttons
    buttons[0] = PS4.getButtonPress(CROSS);
    buttons[1] = PS4.getButtonPress(SQUARE);
    buttons[2] = PS4.getButtonPress(CIRCLE);
    buttons[3] = PS4.getButtonPress(TRIANGLE);
    buttons[9] = PS4.getButtonPress(L3);
    buttons[10] = PS4.getButtonPress(R3);
    buttons[4] = PS4.getButtonPress(L1);
    buttons[5] = PS4.getButtonPress(R1);

    if (PS4.getButtonPress(PS)) {
      PS4.setLedFlash(10, 10);
      buttons[8] = 1;
    }
    else {
      buttons[8] = 0;
      PS4.setLedFlash(0, 0);
    }

    // Invert the directions
    axes[1] = 255 - axes[1];
    axes[2] = 255 - axes[2];
    axes[4] = 255 - axes[4];
    axes[5] = 255 - axes[5];

    // Print starting character
    Serial.print("s ");
    
    // Print joystick values
    for (int i = 0; i < 8; i++) {
      Serial.write(axes[i]);
      Serial.print(" ");
    }
    
    // Print button values
    for (int i = 0; i < 11; i++) {
      Serial.write(buttons[i]);
      Serial.print(" ");
    }
    
    // Print end character 
    Serial.print("e");
    Serial.println();
  }
  delay(60); // Delay 60 ms which is about the time it takes to transmit fully
}
