#include "mcp2515_iso_tp.h"

#define MCP2515_CS_PIN (4) // SPI channel select
#define CH_1_PIN (5)       // Futaba ch1, rudder control
#define CH_3_PIN (6)       // Futaba ch3, sail control
#define CH_5_PIN (9)       // Futaba ch5, switch A
#define CH_6_PIN (10)      // Futaba ch6, VR
#define BUFFER_SIZE (5)    // Filter size

MCP2515 mcp2515(MCP2515_CS_PIN);
Mcp2515IsoTP can_wrapper(&mcp2515);

iso_tp::Message futaba_msg(21, 0b00000000001, 0b00000000000);

uint32_t ch_1_buf[BUFFER_SIZE] = {0};
uint32_t ch_3_buf[BUFFER_SIZE] = {0};
uint32_t ch_6_buf[BUFFER_SIZE] = {0};
uint8_t counter = 0;

void setup() {
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF);
  mcp2515.setFilter(MCP2515::RXF0, false, futaba_msg.rx_id);
  mcp2515.setNormalMode();

  pinMode(CH_1_PIN, INPUT);
  pinMode(CH_3_PIN, INPUT);
  pinMode(CH_5_PIN, INPUT);
  pinMode(CH_6_PIN, INPUT);
}

void loop() {
  can_wrapper.update();

  static unsigned long last_time = millis();
  if ((millis() - last_time) >= 20) { // 50hz

    // Read PWM pins
    ch_1_buf[counter] = pulseIn(CH_1_PIN, HIGH, 10000);
    ch_3_buf[counter] = pulseIn(CH_3_PIN, HIGH, 10000);
    uint32_t ch_5_val = pulseIn(CH_5_PIN, HIGH, 10000);
    ch_6_buf[counter] = pulseIn(CH_6_PIN, HIGH, 10000);

    // Lookup table for switch position
    uint8_t switch_state = 0;
    if (ch_5_val < 1300) {
      switch_state = joy.SWITCH_UP;
    } else if (ch_5_val > 1700) {
      switch_state = joy.SWITCH_DOWN;
    } else {
      switch_state = joy.SWITCH_MIDDLE;
    }

    int32_t right_stick_x = average(ch_1_buf) - 995;
    int32_t left_stick_y = average(ch_3_buf) - 995;
    uint8_t switch_a = switch_state;
    int32_t vr = average(ch_6_buf) - 870;

    // Check that controller values are valid (end up being -1000 if controller
    // is not on) Check uses -160 because the values can briefly drop to -1 and
    // trim can set them as low as -150
    if (joy.right_stick_x > -160 && joy.left_stick_y > -160 && joy.vr > -25 &&
        joy.right_stick_x < 3000 && joy.right_stick_y < 3000 && joy.vr < 3000) {
      memset(futaba_msg.data, 0, futaba_msg.len);

      // skip left_stick_x
      memcpy(futaba_msg.data + 4, left_stick_y, 4);
      memcpy(futaba_msg.data + 8, right_stick_x, 4);
      // skip right_stick_y
      memcpy(futaba_msg.data + 16, switch_a, 1);
      memcpy(futaba_msg.data + 17, vr, 4);

      can_wrapper.sendMessage(&futaba_msg);
    }

    counter = (counter + 1) % 5;
    last_time = millis();
  }
}

uint32_t average(uint32_t* buffer) {
  uint64_t sum = 0;
  uint8_t valid_data_counter = 0;
  for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
    sum += buffer[i];
    if (buffer[i] != 0)
      valid_data_counter++;
  }
  return sum / valid_data_counter;
}
