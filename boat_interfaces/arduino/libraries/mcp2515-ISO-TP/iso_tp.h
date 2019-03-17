#ifndef _ISO_TP_H_
#define _ISO_TP_H_

#include <Arduino.h>
#include "can.h"

// Let the compiler optimize deep array copies. Large arrays will likely use
// memcpy. Small arrays will likely use use element-by-element copy.
template <typename T>
inline void* arrcpy(T* dest, const T* src, size_t len) {
  for (int i = 0; i < len; i++) dest[i] = src[i];
}

namespace iso_tp {

/**
 * @brief A general message structure, containing data of an arbitrary length
 */
struct Message {
  Message() = default;
  explicit Message(uint8_t new_len, canid_t tx_id = 0, canid_t rx_id = 0) {
    resize(new_len);
    this->tx_id = tx_id;
    this->rx_id = rx_id;
  }

  Message(const Message& o) {
    tx_id = o.tx_id;
    rx_id = o.rx_id;
    resize(o.len);
    arrcpy(data, o.data, len);
  }

  Message& operator=(const Message& o) {
    tx_id = o.tx_id;
    rx_id = o.rx_id;
    resize(o.len);
    arrcpy(data, o.data, len);
  }

  ~Message() { reset(); }

  void reset() {
    len = 0;
    delete[] data;
    data = nullptr;
  }

  void resize(uint8_t new_len) {
    len = new_len;
    delete[] data;
    data = new uint8_t[len];
  }

  canid_t tx_id;
  canid_t rx_id;
  uint8_t* data;
  uint8_t len;
} __attribute__((packed));

/**
 * @brief Single CAN frame
 *
 * Each single frame contains up to 7 bytes, plus 1 PCI byte.
 *
 * Schema:
 *  - 0H = ISO-TP Frame Type = 0
 *  - 0L = Data length, from 1 to 7 bytes
 *  - 1-7 = Data
 */
struct SingleFrame : can_frame {
  static const uint8_t FRAME_TYPE = 0;

  SingleFrame() { data[0] = FRAME_TYPE << 4; }

  explicit SingleFrame(uint16_t msg_len) : SingleFrame() {
    data[0] |= msg_len & 0x0F;
    can_dlc = min(msg_len + 1, 7);
  }

  explicit SingleFrame(uint16_t msg_len, const uint8_t* msg_data)
      : SingleFrame(msg_len) {
    arrcpy(data + 1, msg_data, can_dlc);
  }

  explicit SingleFrame(const Message* msg) : SingleFrame(msg->len, msg->data) {
    can_id = msg->tx_id;
  }
};

/**
 * @brief First CAN frame in a multi-frame message
 *
 * Each first frame contains exactly 6 bytes of data, plus 2 PCI bytes,
 * resulting in a total payload length of 8 bytes.
 *
 * Schema:
 *  - 0H = ISO-TP Frame Type = 1
 *  - 0L 1H 1L = Data length, from 8 to 4095 bytes
 *  - 2-7 = Data
 */
struct FirstFrame : can_frame {
  static const uint8_t FRAME_TYPE = 1;

  FirstFrame() {
    data[0] = FRAME_TYPE << 4;
    can_dlc = 8;
  }

  explicit FirstFrame(uint16_t msg_len) : FirstFrame() {
    msg_len = min(msg_len, 4095);
    data[0] |= (msg_len >> 8 & 0x0F);
    data[1] = msg_len & 0xFF;
  }

  explicit FirstFrame(uint16_t msg_len, const uint8_t* msg_data)
      : FirstFrame(msg_len) {
    arrcpy(data + 2, msg_data, 6);
  }

  explicit FirstFrame(const Message* msg) : FirstFrame(msg->len, msg->data) {
    can_id = msg->tx_id;
  }
};

/**
 * @brief Consecutive CAN frame in a multi-frame message
 *
 * Each consecutive frame contains 1 to 7 bytes of data, plus 1 PCI byte.
 *
 * The sequence number is 1 for the first consecutive frame in a message, and
 * increments with each subsequent consecutive frame. It overflows/wraps at 15,
 * resulting in a sequence (1, 2, 3, ... , 15, 0, 1, ... )
 *
 * Schema:
 *  - 0H = ISO-TP Frame Type = 2
 *  - 0L = Sequence number, from 0-15
 *  - 1-7 = Data
 */
struct ConsecutiveFrame : can_frame {
  static const uint8_t FRAME_TYPE = 2;

  ConsecutiveFrame() { data[0] = FRAME_TYPE << 4; }

  explicit ConsecutiveFrame(uint16_t sequence) : ConsecutiveFrame() {
    data[0] |= sequence & 0x0F;
  }

  explicit ConsecutiveFrame(uint16_t sequence, uint16_t msg_len,
                            const uint8_t* msg_data)
      : ConsecutiveFrame(sequence) {
    uint16_t offset = (sequence * 7) - 1;
    can_dlc = min(msg_len - offset, 7) + 1;
    arrcpy(data + 1, msg_data + offset, can_dlc);
  }

  explicit ConsecutiveFrame(uint16_t sequence, const Message* msg)
      : ConsecutiveFrame(sequence, msg->len, msg->data) {
    can_id = msg->tx_id;
  }
};

/**
 * @brief Flow control CAN frame in a multi-frame message
 *
 * Each flow frame contains exactly 0 bytes of data, plus 3 PCI bytes,
 * resulting in a total payload length of 3 bytes.
 *
 * This frame is a response from the receiver after the first frame,
 * acknowledging the beginning of the message and setting the parameters for the
 * transmission of the next set of frames in the message.
 *
 * Byte 1 of the PCI describes the number of frames that the sender may send
 * before it must wait for the next flow control frame to be received. If it is
 * 0, then all remaining frames may be sent.
 *
 * Byte 2 of the PCI describes the minimum required separation time between the
 * end of one message and the start of the next that the sender must adhere to.
 * Values of 0 to 127 correspond to milliseconds. Values of 0xF1 to 0xF9
 * correspond to 0.1-0.9 milliseconds. All other values are illegal.
 *
 * Schema:
 *  - 0H = ISO-TP Frame Type = 3
 *  - 0L = Flow status. 0=Continue, 1=Wait, 2=Abort
 *  - 1  = The number of frames that may be sent before the next flow frame
 *  - 2  = The separation time between the frames
 */
struct FlowFrame : can_frame {
  static const uint8_t FRAME_TYPE = 3;
  static const uint8_t STATUS_CONTINUE = 0;
  static const uint8_t STATUS_WAIT = 1;
  static const uint8_t STATUS_ABORT = 2;

  FlowFrame() {
    data[0] = FRAME_TYPE << 4;
    can_dlc = 3;
  }

  explicit FlowFrame(uint8_t status, uint8_t num_frames, uint8_t separation)
      : FlowFrame() {
    data[0] |= status & 0x0F;
    data[1] = num_frames;
    data[2] = separation;
  }
};
}  // namespace iso_tp

#endif
