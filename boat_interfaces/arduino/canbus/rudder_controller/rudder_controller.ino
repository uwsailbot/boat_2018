#include "mcp2515_iso_tp.h"

#define MCP2515_CS_PIN (4) // SPI channel select
#define RUDDER_1_PIN (2)
#define RUDDER_2_PIN (3)

#define RUDDER_CMD_MSG_RX_ID (0b00100100001)
#define RUDDER_CMD_MSG_TX_ID (0b00100100000)

MCP2515 mcp2515(MCP2515_CS_PIN);
Mcp2515IsoTP can_wrapper(&mcp2515);

Servo servo_rudder1;
Servo servo_rudder2;

void setup() {
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF);
  mcp2515.setFilter(MCP2515::RXF0, false, RUDDER_CMD_MSG_ID);
  mcp2515.setNormalMode();

  servo_rudder1.attach(RUDDER_1_PIN);
  servo_rudder2.attach(RUDDER_2_PIN);

  servo_rudder1.write(90);
  servo_rudder2.write(90);

  can_wrapper.configFlowFrame(RUDDER_CMD_MSG_RX_ID, RUDDER_CMD_MSG_TX_ID, 1, 5);
}

void loop() {
  can_wrapper.update();

  static iso_tp::Message canMsg;
  while (can_wrapper.readMessage(&canMsg)) {
    float32_t pos;
    memcpy(&pos, cmd_msg.data, 4);
    servo_rudder1.write(pos);
    servo_rudder2.write(pos);
  }

  // delay(1);
}
