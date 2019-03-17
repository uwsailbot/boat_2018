#ifndef _MCP2515_H_
#define _MCP2515_H_

#include <SPI.h>
#include "can.h"
#include "mcp2515_configs.h"

class MCP2515 {
public:

  enum ERROR {
    ERROR_OK        = 0,
    ERROR_FAIL      = 1,
    ERROR_ALLTXBUSY = 2,
    ERROR_FAILINIT  = 3,
    ERROR_FAILTX    = 4,
    ERROR_NOMSG     = 5
  };

  enum MASK {
    MASK0,
    MASK1
  };

  enum RXF {
    RXF0 = 0,
    RXF1 = 1,
    RXF2 = 2,
    RXF3 = 3,
    RXF4 = 4,
    RXF5 = 5
  };

  enum RXBn {
    RXB0 = 0,
    RXB1 = 1
  };

  enum TXBn {
    TXB0 = 0,
    TXB1 = 1,
    TXB2 = 2
  };

  enum /*class*/ CANINTF : uint8_t {
    CANINTF_RX0IF = 0x01,
    CANINTF_RX1IF = 0x02,
    CANINTF_TX0IF = 0x04,
    CANINTF_TX1IF = 0x08,
    CANINTF_TX2IF = 0x10,
    CANINTF_ERRIF = 0x20,
    CANINTF_WAKIF = 0x40,
    CANINTF_MERRF = 0x80
  };

private:

  // Operation modes for CAN bus
  enum class CANOperationMode : uint8_t {
    NORMAL     = 0b000<<5,
    SLEEP      = 0b001<<5,
    LOOPBACK   = 0b010<<5,
    LISTENONLY = 0b011<<5,
    CONFIG     = 0b100<<5,
    POWERUP    = 0b111<<5
  };

  // Masks for the CAN control register (CANCTRL)
  static const uint8_t CANCTRL_REQOP_MASK = 0xE0;   // Request operation mode
  static const uint8_t CANCTRL_ABAT_MASK = 0x10;    // Abort all pending transmissions
  static const uint8_t CANCTRL_OSM_MASK = 0x08;     // One-Shot mode (Should retry failed transmissions)
  static const uint8_t CANCTRL_CLKEN_MASK = 0x04;   // Enable CLKOUT pin
  static const uint8_t CANCTRL_CLKPRE_MASK = 0x03;  // CLKOUT prescaler

  // Masks for the CAN status register (CANSTAT)
  static const uint8_t CANSTAT_OPMOD_MASK = 0xE0;   // Operation mode
  static const uint8_t CANSTAT_ICOD_MASK = 0x0E;    // Interrupt flags

  // Mask for the Standard Identifier Low registers (TXBnSIDL, RXBnSIDL)
  static const uint8_t SID_EXIDE_MASK = 0x08;

  // Masks for the Receive Buffer Control registers (RXBnCTRL)
  static const uint8_t RXBnCTRL_RXM_MASK   = 0x60;
  static const uint8_t RXBnCTRL_RTR_MASK   = 0x08;
  static const uint8_t RXB0CTRL_BUKT_MASK  = 0x04;

  // Masks for the Transmit Buffer Control registers (TXBnCTRL)
  static const uint8_t TXBnCTRL_ABTF   = 0x40;
  static const uint8_t TXBnCTRL_MLOA   = 0x20;
  static const uint8_t TXBnCTRL_TXERR  = 0x10;
  static const uint8_t TXBnCTRL_TXREQ  = 0x08;
  static const uint8_t TXBnCTRL_TXP    = 0x03;

  static const uint8_t DLC_MASK       = 0x0F;
  static const uint8_t RTR_MASK       = 0x40;

  static const uint8_t MCP_SIDH = 0;
  static const uint8_t MCP_SIDL = 1;
  static const uint8_t MCP_EID8 = 2;
  static const uint8_t MCP_EID0 = 3;
  static const uint8_t MCP_DLC  = 4;
  static const uint8_t MCP_DATA = 5;

  // Masks for the READ STATUS instruction result
  static const uint8_t STAT_RX0IF_MASK = 0x1;
  static const uint8_t STAT_RX1IF_MASK = 0x2;
  static const uint8_t STAT_RXIF_MASK = STAT_RX0IF_MASK | STAT_RX1IF_MASK;

  // Masks for the error flag register (EFLG)
  static const uint8_t EFLG_RX1OVR = (1<<7);
  static const uint8_t EFLG_RX0OVR = (1<<6);
  static const uint8_t EFLG_TXBO   = (1<<5);
  static const uint8_t EFLG_TXEP   = (1<<4);
  static const uint8_t EFLG_RXEP   = (1<<3);
  static const uint8_t EFLG_TXWAR  = (1<<2);
  static const uint8_t EFLG_RXWAR  = (1<<1);
  static const uint8_t EFLG_EWARN  = (1<<0);
  static const uint8_t EFLG_ERRORMASK = EFLG_RX1OVR
                                      | EFLG_RX0OVR
                                      | EFLG_TXBO
                                      | EFLG_TXEP
                                      | EFLG_RXEP;
  static const uint8_t EFLG_WARNMASK = EFLG_TXWAR
                                      | EFLG_RXWAR
                                      | EFLG_EWARN;

  enum /*class*/ INSTRUCTION : uint8_t {
    INSTRUCTION_WRITE       = 0x02,
    INSTRUCTION_READ        = 0x03,
    INSTRUCTION_BITMOD      = 0x05,
    INSTRUCTION_LOAD_TX0    = 0x40,
    INSTRUCTION_LOAD_TX1    = 0x42,
    INSTRUCTION_LOAD_TX2    = 0x44,
    INSTRUCTION_RTS_TX0     = 0x81,
    INSTRUCTION_RTS_TX1     = 0x82,
    INSTRUCTION_RTS_TX2     = 0x84,
    INSTRUCTION_RTS_ALL     = 0x87,
    INSTRUCTION_READ_RX0    = 0x90,
    INSTRUCTION_READ_RX1    = 0x94,
    INSTRUCTION_READ_STATUS = 0xA0,
    INSTRUCTION_RX_STATUS   = 0xB0,
    INSTRUCTION_RESET       = 0xC0
  };

  enum /*class*/ REGISTER : uint8_t {
    MCP_RXF0SIDH = 0x00,
    MCP_RXF0SIDL = 0x01,
    MCP_RXF0EID8 = 0x02,
    MCP_RXF0EID0 = 0x03,
    MCP_RXF1SIDH = 0x04,
    MCP_RXF1SIDL = 0x05,
    MCP_RXF1EID8 = 0x06,
    MCP_RXF1EID0 = 0x07,
    MCP_RXF2SIDH = 0x08,
    MCP_RXF2SIDL = 0x09,
    MCP_RXF2EID8 = 0x0A,
    MCP_RXF2EID0 = 0x0B,
    MCP_CANSTAT  = 0x0E,
    MCP_CANCTRL  = 0x0F,
    MCP_RXF3SIDH = 0x10,
    MCP_RXF3SIDL = 0x11,
    MCP_RXF3EID8 = 0x12,
    MCP_RXF3EID0 = 0x13,
    MCP_RXF4SIDH = 0x14,
    MCP_RXF4SIDL = 0x15,
    MCP_RXF4EID8 = 0x16,
    MCP_RXF4EID0 = 0x17,
    MCP_RXF5SIDH = 0x18,
    MCP_RXF5SIDL = 0x19,
    MCP_RXF5EID8 = 0x1A,
    MCP_RXF5EID0 = 0x1B,
    MCP_TEC      = 0x1C,
    MCP_REC      = 0x1D,
    MCP_RXM0SIDH = 0x20,
    MCP_RXM0SIDL = 0x21,
    MCP_RXM0EID8 = 0x22,
    MCP_RXM0EID0 = 0x23,
    MCP_RXM1SIDH = 0x24,
    MCP_RXM1SIDL = 0x25,
    MCP_RXM1EID8 = 0x26,
    MCP_RXM1EID0 = 0x27,
    MCP_CNF3     = 0x28,
    MCP_CNF2     = 0x29,
    MCP_CNF1     = 0x2A,
    MCP_CANINTE  = 0x2B,
    MCP_CANINTF  = 0x2C,
    MCP_EFLG     = 0x2D,
    MCP_TXB0CTRL = 0x30,
    MCP_TXB0SIDH = 0x31,
    MCP_TXB0SIDL = 0x32,
    MCP_TXB0EID8 = 0x33,
    MCP_TXB0EID0 = 0x34,
    MCP_TXB0DLC  = 0x35,
    MCP_TXB0DATA = 0x36,
    MCP_TXB1CTRL = 0x40,
    MCP_TXB1SIDH = 0x41,
    MCP_TXB1SIDL = 0x42,
    MCP_TXB1EID8 = 0x43,
    MCP_TXB1EID0 = 0x44,
    MCP_TXB1DLC  = 0x45,
    MCP_TXB1DATA = 0x46,
    MCP_TXB2CTRL = 0x50,
    MCP_TXB2SIDH = 0x51,
    MCP_TXB2SIDL = 0x52,
    MCP_TXB2EID8 = 0x53,
    MCP_TXB2EID0 = 0x54,
    MCP_TXB2DLC  = 0x55,
    MCP_TXB2DATA = 0x56,
    MCP_RXB0CTRL = 0x60,
    MCP_RXB0SIDH = 0x61,
    MCP_RXB0SIDL = 0x62,
    MCP_RXB0EID8 = 0x63,
    MCP_RXB0EID0 = 0x64,
    MCP_RXB0DLC  = 0x65,
    MCP_RXB0DATA = 0x66,
    MCP_RXB1CTRL = 0x70,
    MCP_RXB1SIDH = 0x71,
    MCP_RXB1SIDL = 0x72,
    MCP_RXB1EID8 = 0x73,
    MCP_RXB1EID0 = 0x74,
    MCP_RXB1DLC  = 0x75,
    MCP_RXB1DATA = 0x76
  };

  static const uint32_t SPI_CLOCK = 10000000; // 10MHz

  static const uint8_t N_TXBUFFERS = 3;
  static const uint8_t N_RXBUFFERS = 2;

  static const struct TXBn_REGS {
    REGISTER CTRL;
    REGISTER SIDH;
    REGISTER DATA;
  } TXB[N_TXBUFFERS];

  static const struct RXBn_REGS {
    REGISTER CTRL;
    REGISTER SIDH;
    REGISTER DATA;
    CANINTF  CANINTF_RXnIF;
  } RXB[N_RXBUFFERS];

  uint8_t SPI_CS;

private:

    void startSPI();
    void endSPI();

    ERROR setMode(const CANOperationMode mode);

    uint8_t readRegister(const REGISTER reg);
    void readRegisters(const REGISTER reg, uint8_t values[], const uint8_t n);
    void setRegister(const REGISTER reg, const uint8_t value);
    void setRegisters(const REGISTER reg, const uint8_t values[], const uint8_t n);
    void modifyRegister(const REGISTER reg, const uint8_t mask, const uint8_t data);

    void prepareId(uint8_t *buffer, const bool ext, const uint32_t id);

public:
    MCP2515(const uint8_t _CS);
    ERROR reset(void);
    ERROR setConfigMode();
    ERROR setListenOnlyMode();
    ERROR setSleepMode();
    ERROR setLoopbackMode();
    ERROR setNormalMode();
    ERROR setBitrate(const CAN_SPEED canSpeed);
    ERROR setBitrate(const CAN_SPEED canSpeed, const CAN_CLOCK canClock);
    ERROR setFilterMask(const MASK num, const bool ext, const uint32_t ulData);
    ERROR setFilter(const RXF num, const bool ext, const uint32_t ulData);
    ERROR sendFrame(const TXBn txbn, const struct can_frame *frame);
    ERROR sendFrame(const struct can_frame *frame);
    ERROR readFrame(const RXBn rxbn, struct can_frame *frame);
    ERROR readFrame(struct can_frame *frame);
    bool checkReceive(void);
    bool checkError(void);
    uint8_t getErrorFlags(void);
    void clearRXnOVRFlags(void);
    uint8_t getInterrupts(void);
    uint8_t getInterruptMask(void);
    void clearInterrupts(void);
    void clearTXInterrupts(void);
    uint8_t getStatus(void);
    void clearRXnOVR(void);
    void clearMERR();
};

#endif
