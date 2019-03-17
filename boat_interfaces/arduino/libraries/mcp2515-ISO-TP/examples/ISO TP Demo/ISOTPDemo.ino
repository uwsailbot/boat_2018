#include "mcp2515_iso_tp.h"

MCP2515 mcp2515(4);
Mcp2515IsoTP can_wrapper(&mcp2515);
iso_tp::Message long_msg(50, 1, 2);
iso_tp::Message short_msg(5, 20);

void setup() {
  Serial.begin(115200);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  arrcpy((char*)long_msg.data,
         "Hello World! abcdefghijklmnopqrstuvwxyz0123456789!", long_msg.len);
  arrcpy((char*)short_msg.data, "Short", short_msg.len);
}

void loop() {
  can_wrapper.update();

  static iso_tp::Message canMsg;
  while (can_wrapper.readMessage(&canMsg)) {
    Serial.print("ID: ");
    Serial.print(canMsg.rx_id);
    Serial.print(" - MSG: ");
    Serial.write(canMsg.data, canMsg.len);
    Serial.println();
  }

  static unsigned long last_time_long = millis();
  if ((millis() - last_time_long) >= 1000) {
    can_wrapper.sendMessage(&long_msg);
    last_time_long = millis();
  }

  static unsigned long last_time_short = millis();
  if ((millis() - last_time_short) >= 200) {
    can_wrapper.sendMessage(&short_msg);
    last_time_short = millis();
  }
}
