#ifndef _mcp2515_iso_tp_H_
#define _mcp2515_iso_tp_H_

#include "iso_tp.h"
#include "mcp2515.h"

// TODO: Better errors

class Mcp2515IsoTP {
public:
  enum class ArbitrationMode { PRIORITY_ID, LIFO, FIFO };

  // Maximum number of multi-frame messages to send, multi-frame messages to
  // receive, and assembled/ready to read messages
  static const uint8_t MAX_TX_MSGS = 2;
  static const uint8_t MAX_RX_MSGS = 2;
  static const uint8_t MAX_READY_MSGS = 2;

  MCP2515* const mcp2515;
  const ArbitrationMode mode;

  Mcp2515IsoTP(const Mcp2515IsoTP&) = delete;
  Mcp2515IsoTP& operator=(const Mcp2515IsoTP&) = delete;

  explicit Mcp2515IsoTP(MCP2515* interface,
                        ArbitrationMode mode = ArbitrationMode::PRIORITY_ID)
      : mcp2515(interface), mode(mode) {}

  void update();
  bool hasMessage() const { return num_ready_msgs_ > 0; };
  bool readMessage(iso_tp::Message* msg);
  bool sendMessage(const iso_tp::Message* msg);

  bool configFlowFrame(canid_t rx_id, canid_t tx_id, uint8_t block_size,
                       uint8_t separation);
  bool removeFlowFrame(canid_t rx_id, canid_t tx_id = 0);

private:
  struct TxConfig {
    iso_tp::Message msg;

    // Tracking transmission
    bool waiting;                   // Is waiting for flow frame
    uint16_t sequence;              // Sequence of last sent consecutive frame
    uint8_t remaining_frames;       // Remaining num of frames that may be sent
    unsigned long prev_frame_time;  // Time the last frame was sent, in us

    // Configuration
    uint16_t separation;  // Separation time between frames, in 0.1ms
  };

  struct RxConfig {
    iso_tp::Message msg;

    // Tracking transmission
    bool is_active;             // Transmission is active
    uint16_t sequence;          // Sequence of last received consecutive frame
    uint16_t remaining_frames;  // Remaining num of frames before Flow Frame

    // Configuration
    uint8_t block_size;   // Num of frames that may be sent
    uint16_t separation;  // Separation time between frames, in 0.1ms
  };

  uint8_t num_tx_msgs_ = 0;
  uint8_t num_rx_msgs_ = 0;
  uint8_t num_ready_msgs_ = 0;
  TxConfig tx_msgs_[MAX_TX_MSGS];
  RxConfig rx_msgs_[MAX_RX_MSGS];
  iso_tp::Message ready_msgs_[MAX_READY_MSGS];

  void readBuffer(MCP2515::RXBn buffer);
  void sendFlowFrame(const RxConfig* config, uint8_t status);

  bool removeTxMsg(uint8_t index);
  bool removeRxMsg(uint8_t index);
  bool removeReadyMsg(uint8_t index);

  TxConfig* insertTxMsg(const iso_tp::Message* msg);
  RxConfig* insertRxMsg(canid_t rx_id, canid_t tx_id);
  iso_tp::Message* insertReadyMsg(canid_t rx_id);
};

#endif
