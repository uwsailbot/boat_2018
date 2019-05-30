#include "mcp2515_iso_tp.h"

#define MCP2515_CS_PIN (3) // SPI channel select
#define CH_1_PIN (4)       // Futaba ch1, rudder control
#define CH_2_PIN (5)       // Futaba ch2, unused
#define CH_3_PIN (6)       // Futaba ch3, sail control
#define CH_4_PIN (7)       // Futaba ch4, unused
#define CH_5_PIN (8)       // Futaba ch5, switch A
#define CH_6_PIN (9)       // Futaba ch6, VR
#define CH_7_PIN (10)      // Futaba ch7/B, unused
#define BUFFER_SIZE (5)    // Filter size

MCP2515 mcp2515(MCP2515_CS_PIN);
Mcp2515IsoTP can_wrapper(&mcp2515);

iso_tp::Message futaba_msg(21, 0b00000000001, 0b00000000000);

uint32_t ch_1_buf[BUFFER_SIZE] = {0};
uint32_t ch_2_buf[BUFFER_SIZE] = {0};
uint32_t ch_3_buf[BUFFER_SIZE] = {0};
uint32_t ch_4_buf[BUFFER_SIZE] = {0};
// uint32_t ch_5_buf[BUFFER_SIZE] = {0};
uint32_t ch_6_buf[BUFFER_SIZE] = {0};
uint32_t ch_7_buf[BUFFER_SIZE] = {0};
uint8_t counter = 0;

enum SwitchState {
  SWITCH_UP = 2,
  SWITCH_MIDDLE = 1,
  SWITCH_DOWN = 0
};

void setup() {
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF);
  mcp2515.setFilter(MCP2515::RXF0, false, futaba_msg.rx_id);
  mcp2515.setNormalMode();

  pinMode(CH_1_PIN, INPUT);
  pinMode(CH_2_PIN, INPUT);
  pinMode(CH_3_PIN, INPUT);
  pinMode(CH_4_PIN, INPUT);
  pinMode(CH_5_PIN, INPUT);
  pinMode(CH_6_PIN, INPUT);
  pinMode(CH_7_PIN, INPUT);
}

void loop() {
  can_wrapper.update();

  static unsigned long last_time = millis();
  if ((millis() - last_time) >= 20) { // 50hz

    // Read PWM pins
    noInterrupts();
    ch_1_buf[counter] = pulseIn(CH_1_PIN, HIGH, 10000);
    ch_2_buf[counter] = pulseIn(CH_2_PIN, HIGH, 10000);
    ch_3_buf[counter] = pulseIn(CH_3_PIN, HIGH, 10000);
    ch_4_buf[counter] = pulseIn(CH_4_PIN, HIGH, 10000);
    uint32_t ch_5_val = pulseIn(CH_5_PIN, HIGH, 10000);
    ch_6_buf[counter] = pulseIn(CH_6_PIN, HIGH, 10000);
    ch_7_buf[counter] = pulseIn(CH_7_PIN, HIGH, 10000);
    interrupts();

    // Lookup table for switch position
    uint8_t switch_a = 0;
    if (ch_5_val < 1300) {
      switch_a = SWITCH_UP;
    } else if (ch_5_val > 1700) {
      switch_a = SWITCH_DOWN;
    } else {
      switch_a = SWITCH_MIDDLE;
    }

    int32_t right_stick_x = average(ch_1_buf) - 995;
    int32_t right_stick_y = average(ch_2_buf) - 995; // TODO: Verify
    int32_t left_stick_y = average(ch_3_buf) - 995;
    int32_t left_stick_x = average(ch_4_buf) - 995; // TODO: Verify
    int32_t vr = average(ch_6_buf) - 870;
    int32_t unused = average(ch_7_buf) - 995;

    // Check that controller values are valid (end up being -1000 if controller
    // is not on) Check uses -160 because the values can briefly drop to -1 and
    // trim can set them as low as -150
    if (right_stick_x > -160 && left_stick_y > -160 && vr > -25 &&
        right_stick_x < 3000 && left_stick_y < 3000 && vr < 3000) {
      memset(futaba_msg.data, 0, futaba_msg.len);

      memcpy(futaba_msg.data + 0, left_stick_x, 4);
      memcpy(futaba_msg.data + 4, left_stick_y, 4);
      memcpy(futaba_msg.data + 8, right_stick_x, 4);
      memcpy(futaba_msg.data + 12, right_stick_y, 4);
      memcpy(futaba_msg.data + 16, switch_a, 1);
      memcpy(futaba_msg.data + 17, vr, 4);
      // memcpy(futaba_msg.data + 21, unused, 4);

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
