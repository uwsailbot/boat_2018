#include "mcp2515_iso_tp.h"

// TODO: Check/return/handle errors

void Mcp2515IsoTP::update() {

  // Check for new received frames
  // Note: We read buffer 1 first
  uint8_t ints = mcp2515->getInterrupts();
  if (ints & MCP2515::CANINTF_RX0IF) readBuffer(MCP2515::RXB0);
  if (ints & MCP2515::CANINTF_RX1IF) readBuffer(MCP2515::RXB1);

  // Check if outgoing messages are ready to send their next consecutive frame
  for (uint8_t i = 0; i < num_tx_msgs_; i++) {
    TxConfig* msg = &tx_msgs_[i];

    // Waiting for flow frame or separation period not exceeded
    if (msg->waiting ||
        (micros() - msg->prev_frame_time) / 100 < msg->separation)
      continue;

    // Update the msg data
    msg->sequence++;
    msg->prev_frame_time = micros();
    if (msg->remaining_frames == 0)
      ;
    else if (msg->remaining_frames == 1)
      msg->waiting = true;
    else
      msg->remaining_frames--;

    // Send the next frame
    iso_tp::ConsecutiveFrame consec(msg->sequence, &msg->msg);
    mcp2515->sendFrame(&consec);

    // If all the data has been sent, cleanup
    if ((msg->sequence * 7) + 6 >= msg->msg.len) {
      removeTxMsg(i);
      msg = nullptr;
    }
  }
}

bool Mcp2515IsoTP::readMessage(iso_tp::Message* msg) {
  if (!hasMessage()) return false;
  *msg = ready_msgs_[num_ready_msgs_ - 1];
  return removeReadyMsg(num_ready_msgs_ - 1);
}

bool Mcp2515IsoTP::sendMessage(const iso_tp::Message* msg) {

  // If the message is very short, send a Single Frame
  if (msg->len < 8) {
    iso_tp::SingleFrame frame(msg);
    return mcp2515->sendFrame(&frame) == MCP2515::ERROR_OK;
  }

  // Create a new message with the data
  TxConfig* config = insertTxMsg(msg);
  if (config == nullptr) return false;
  config->sequence = 0;
  config->waiting = true;

  // Send the first frame
  iso_tp::FirstFrame first(&(config->msg));
  return mcp2515->sendFrame(&first) == MCP2515::ERROR_OK;
}

bool Mcp2515IsoTP::configFlowFrame(canid_t rx_id, canid_t tx_id,
                                   uint8_t block_size, uint8_t separation) {
  RxConfig* msg = insertRxMsg(rx_id, tx_id);
  if (msg == nullptr) return false;

  msg->block_size = block_size;
  msg->separation = separation;
  return true;
}

bool Mcp2515IsoTP::removeFlowFrame(canid_t rx_id, canid_t tx_id) {
  for (uint8_t i = 0; i < num_rx_msgs_; i++) {
    if (rx_msgs_[i].msg.rx_id == rx_id &&
        (tx_id != 0 && rx_msgs_[i].msg.tx_id == tx_id)) {
      removeRxMsg(i);
      return true;
    }
  }
  return false;
}

void Mcp2515IsoTP::readBuffer(MCP2515::RXBn buffer) {
  using namespace iso_tp;

  // Read the frame
  static can_frame recv_msg;
  if (mcp2515->readFrame(buffer, &recv_msg) != MCP2515::ERROR_OK) return;

  // Check the type of the received frame
  // We always ignore malformed frames
  switch (recv_msg.data[0] >> 4) {

    // Received frame is a Single Frame
    case SingleFrame::FRAME_TYPE: {
      for (uint8_t i = 0; i < num_rx_msgs_; i++) {
        RxConfig* msg = &rx_msgs_[i];
        if (msg->msg.rx_id != recv_msg.can_id) return;
      }

      Message* msg = insertReadyMsg(recv_msg.can_id);
      if (msg != nullptr) {
        msg->resize(recv_msg.can_dlc - 1);
        arrcpy(msg->data, recv_msg.data + 1, msg->len);
      }
      return;
    }

    // Received frame is a First Frame
    case FirstFrame::FRAME_TYPE:
      for (uint8_t i = 0; i < num_rx_msgs_; i++) {
        RxConfig* msg = &rx_msgs_[i];
        if (msg->msg.rx_id != recv_msg.can_id) continue;

        // Append the new data, prepare for consecutive frames
        uint16_t len = recv_msg.data[1];
        len |= (recv_msg.data[0] & 0x0F) << 8;
        msg->msg.resize(len);
        arrcpy(msg->msg.data, recv_msg.data + 2, 6);
        msg->sequence = 0;
        msg->is_active = true;
        msg->remaining_frames = msg->block_size;

        // Send flow frame
        sendFlowFrame(msg, 0);
      }

      return;

    // Received frame is a Consecutive Frame
    case ConsecutiveFrame::FRAME_TYPE:
      for (uint8_t i = 0; i < num_rx_msgs_; i++) {
        RxConfig* msg = &rx_msgs_[i];
        if (msg->msg.rx_id != recv_msg.can_id) continue;

        if (!msg->is_active) {
          // TODO: Error, expect First Frame before Consecutive Frame
          Serial.println("Err: Got consec frame, msg inactive");
          return;
        }

        msg->sequence++;
        if ((msg->sequence & 0x0F) != (recv_msg.data[0] & 0x0F)) {
          // TODO: Error, unexpected sequence number
          Serial.println("Err: Got consec frame, bad sequence");
          return;
        }

        // Append the new data
        uint16_t start = (msg->sequence * 7) - 1;
        arrcpy(msg->msg.data + start, recv_msg.data + 1, recv_msg.can_dlc - 1);

        if (msg->block_size != 0) {
          msg->remaining_frames--;
        }

        // If this is the last frame in the message, add the message to the
        // ready buffer and reset
        if (start + 7 >= msg->msg.len) {
          Message* ready_msg = insertReadyMsg(msg->msg.rx_id);

          if (ready_msg != nullptr) {
            ready_msg->resize(msg->msg.len);
            arrcpy(ready_msg->data, msg->msg.data, ready_msg->len);
          }

          msg->sequence = 0;
          msg->is_active = false;
          msg->msg.reset();
        }

        // If this is not the last frame of the message, but it is the last
        // frame of the block, send the next flow frame
        else if (msg->block_size != 0 && msg->remaining_frames == 0) {
          sendFlowFrame(msg, 0);
          msg->remaining_frames = msg->block_size;
        }
        break;
      }
      return;

    // Received frame is a Flow Control Frame
    case FlowFrame::FRAME_TYPE:
      for (uint8_t i = 0; i < num_tx_msgs_; i++) {
        TxConfig* msg = &tx_msgs_[i];
        if (msg->msg.rx_id != recv_msg.can_id) continue;

        switch (recv_msg.data[0] & 0x0F) {

          // Continue transmission
          case FlowFrame::STATUS_CONTINUE: {
            uint8_t val = recv_msg.data[2];
            if (val >> 4 == 1)
              msg->separation = val & 0x0F;
            else
              msg->separation = val * 10;
            msg->waiting = false;
            msg->remaining_frames = recv_msg.data[1];
            msg->prev_frame_time = micros();
          } break;

          // Wait for next flow control frame
          case FlowFrame::STATUS_WAIT:
            msg->waiting = true;
            break;

          // Abort transmission
          case FlowFrame::STATUS_ABORT:
            removeTxMsg(i);
            msg = nullptr;
            break;
        }
        break;
      }
      break;
  }
}

void Mcp2515IsoTP::sendFlowFrame(const RxConfig* config, uint8_t status) {
  iso_tp::FlowFrame frame(status, config->block_size, config->separation);
  frame.can_id = config->msg.tx_id;
  mcp2515->sendFrame(&frame);
}

bool Mcp2515IsoTP::removeTxMsg(uint8_t index) {
  if (index >= num_tx_msgs_) return false;

  // Shift all subsequent messages down
  for (uint8_t i = index; i < num_tx_msgs_ - 1; i++) {
    tx_msgs_[i] = tx_msgs_[i + 1];
  }

  // Delete the last msg
  tx_msgs_[num_tx_msgs_ - 1].msg.reset();
  num_tx_msgs_--;
  return true;
}

bool Mcp2515IsoTP::removeRxMsg(uint8_t index) {
  if (index >= num_rx_msgs_) return false;

  // Shift all subsequent messages down
  for (uint8_t i = index; i < num_rx_msgs_ - 1; i++) {
    rx_msgs_[i] = rx_msgs_[i + 1];
  }

  // Delete the last msg
  rx_msgs_[num_rx_msgs_ - 1].msg.reset();
  num_rx_msgs_--;
  return true;
}

bool Mcp2515IsoTP::removeReadyMsg(uint8_t index) {
  if (index >= num_ready_msgs_) return false;

  // Shift all subsequent messages down
  for (uint8_t i = index; i < num_ready_msgs_ - 1; i++) {
    ready_msgs_[i] = ready_msgs_[i + 1];
  }

  // Delete the last msg
  ready_msgs_[num_ready_msgs_ - 1].reset();
  num_ready_msgs_--;
  return true;
}

Mcp2515IsoTP::TxConfig* Mcp2515IsoTP::insertTxMsg(const iso_tp::Message* msg) {

  // If the message is already in the buffer, update it
  for (uint8_t i = 0; i < num_tx_msgs_; i++) {
    if (tx_msgs_[i].msg.rx_id == msg->rx_id ||
        tx_msgs_[i].msg.tx_id == msg->tx_id) {
      tx_msgs_[i].msg = *msg;
      return &tx_msgs_[i];
    }
  }

  // If the buffer is full, determine which message to replace
  if (num_tx_msgs_ >= MAX_TX_MSGS) {
    switch (mode) {

      case ArbitrationMode::PRIORITY_ID:
        for (uint8_t i = 0; i < num_tx_msgs_; i++) {
          if (tx_msgs_[i].msg.rx_id < msg->rx_id ||
              tx_msgs_[i].msg.tx_id < msg->tx_id) {
            tx_msgs_[i].msg = *msg;
            return &tx_msgs_[i];
          }
        }
        return nullptr;

      case ArbitrationMode::LIFO:
        tx_msgs_[num_tx_msgs_ - 1].msg = *msg;
        return &tx_msgs_[num_tx_msgs_ - 1];

      case ArbitrationMode::FIFO:
        return nullptr;
    }
  }

  // If the buffer isn't full, append the new message
  num_tx_msgs_++;
  tx_msgs_[num_tx_msgs_ - 1].msg = *msg;
  return &tx_msgs_[num_tx_msgs_ - 1];
}

Mcp2515IsoTP::RxConfig* Mcp2515IsoTP::insertRxMsg(canid_t rx_id,
                                                  canid_t tx_id) {

  // If the message is already in the buffer, update it
  for (uint8_t i = 0; i < num_rx_msgs_; i++) {
    if (rx_msgs_[i].msg.rx_id == rx_id || rx_msgs_[i].msg.tx_id == tx_id) {
      return &rx_msgs_[i];
    }
  }

  // If the buffer is full, determine which message to replace
  if (num_rx_msgs_ >= MAX_RX_MSGS) {
    switch (mode) {

      case ArbitrationMode::PRIORITY_ID:
        for (uint8_t i = 0; i < num_rx_msgs_; i++) {
          if (rx_msgs_[i].msg.rx_id < rx_id || rx_msgs_[i].msg.tx_id < tx_id) {
            rx_msgs_[i].msg.rx_id = rx_id;
            rx_msgs_[i].msg.tx_id = tx_id;
            return &rx_msgs_[i];
          }
        }
        return nullptr;

      case ArbitrationMode::LIFO:
        rx_msgs_[num_rx_msgs_ - 1].msg.rx_id = rx_id;
        rx_msgs_[num_rx_msgs_ - 1].msg.tx_id = tx_id;
        return &rx_msgs_[num_rx_msgs_ - 1];

      case ArbitrationMode::FIFO:
        return nullptr;
    }
  }

  // If the buffer isn't full, append the new message
  num_rx_msgs_++;
  rx_msgs_[num_rx_msgs_ - 1].msg.rx_id = rx_id;
  rx_msgs_[num_rx_msgs_ - 1].msg.tx_id = tx_id;
  return &rx_msgs_[num_rx_msgs_ - 1];
}

iso_tp::Message* Mcp2515IsoTP::insertReadyMsg(canid_t rx_id) {

  // If the message is already in the buffer, update it
  for (uint8_t i = 0; i < num_ready_msgs_; i++) {
    if (ready_msgs_[i].rx_id == rx_id) {
      return &ready_msgs_[i];
    }
  }

  // If the buffer is full, determine which message to replace
  if (num_ready_msgs_ >= MAX_READY_MSGS) {
    switch (mode) {

      case ArbitrationMode::PRIORITY_ID:
        for (uint8_t i = 0; i < num_ready_msgs_; i++) {
          if (ready_msgs_[i].rx_id < rx_id) {
            ready_msgs_[i].rx_id = rx_id;
            return &ready_msgs_[i];
          }
        }
        return nullptr;

      case ArbitrationMode::LIFO:
        ready_msgs_[num_ready_msgs_ - 1].rx_id = rx_id;
        return &ready_msgs_[num_ready_msgs_ - 1];

      case ArbitrationMode::FIFO:
        return nullptr;
    }
  }

  // If the buffer isn't full, append the new message
  num_ready_msgs_++;
  ready_msgs_[num_ready_msgs_ - 1].rx_id = rx_id;
  return &ready_msgs_[num_ready_msgs_ - 1];
}
