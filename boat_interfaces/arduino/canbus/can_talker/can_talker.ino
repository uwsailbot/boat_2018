// Example program to send joystick data on the bus

#include "mcp2515_iso_tp.h"

MCP2515 mcp2515(4);
Mcp2515IsoTP can_wrapper(&mcp2515);
iso_tp::Message msg(4, 0b11110000011, 0b11110000010);

void setup() {
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}


int count = 0;
void loop() {
  can_wrapper.update();

  if (count > 500) {
    count = 0;

    uint16_t value = analogRead(A0);
    float scaled = value / 512.0 - 1.0;

    for (unsigned i = 0; i < 4; i++) {
      memcpy(msg.data + i, ((uint8_t*) &scaled) + 3 - i, 1);
    }
    can_wrapper.sendMessage(&msg);
  }
  count++;
  delay(1);
}
