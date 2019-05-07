// Example program to print received msgs to an LCD

#include <LiquidCrystal.h>
#include "mcp2515_iso_tp.h"

MCP2515 mcp2515(3);
Mcp2515IsoTP can_wrapper(&mcp2515);
LiquidCrystal display(8, 9, 4, 5, 6, 7);

void displayMsg(iso_tp::Message* canMsg) {
  display.clear();
  display.print("ID:0x");
  display.print(canMsg->rx_id, HEX);
  display.print(" LEN:");
  display.print(canMsg->len);
  display.setCursor(0, 1);

  for (int i = 0; i < canMsg->len && i < 16; i++) {
    display.print((char)canMsg->data[i]);
  }
}

void setup() {
  display.begin(16, 2);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  can_wrapper.configFlowFrame(0b11110000000, 0b11110000001, 0, 5);
}

void loop() {
  can_wrapper.update();

  static iso_tp::Message canMsg;
  while (can_wrapper.readMessage(&canMsg)) {
    displayMsg(&canMsg);
  }
}
