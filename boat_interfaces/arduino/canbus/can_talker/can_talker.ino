// Example program to send user-inputted strings on the bus

#include "mcp2515_iso_tp.h"

MCP2515 mcp2515(4);
Mcp2515IsoTP can_wrapper(&mcp2515);
iso_tp::Message msg(0, 0b11110000000, 0b11110000001);

void setup() {
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  Serial.begin(9600);
}

void loop() {
  can_wrapper.update();

  if (Serial.available() > 0) {

    String text;
    while (Serial.available()) {
      char c = Serial.read();
      text += c;
      delay(2);
    }

    const char* input = text.c_str();
    Serial.println(input);

    msg.resize(strlen(input) + 1);
    memcpy(msg.data, input, strlen(input) + 1);
    can_wrapper.sendMessage(&msg);
  }
}
