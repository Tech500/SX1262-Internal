/*
  SX1262.h - Updated Arduino SPI Library for Semtech SX1262 Rev 2.2
*/

#ifndef SX1262_H
#define SX1262_H

#include <Arduino.h>
#include <SPI.h>

// ============================================================================
// SX1262 COMMAND OPCODES
// ============================================================================
#define SX126X_CMD_SET_SLEEP                  0x84
#define SX126X_CMD_SET_STANDBY                0x80
#define SX126X_CMD_SET_FS                     0xC1
#define SX126X_CMD_SET_TX                     0x83
#define SX126X_CMD_SET_RX                     0x82
#define SX126X_CMD_STOP_TIMER_ON_PREAMBLE     0x9F
#define SX126X_CMD_SET_RX_DUTY_CYCLE          0x94
#define SX126X_CMD_SET_CAD                    0xC5
#define SX126X_CMD_SET_TX_CONTINUOUS_WAVE     0xD1
#define SX126X_CMD_SET_TX_INFINITE_PREAMBLE   0x8F
#define SX126X_CMD_SET_REGULATOR_MODE         0x96
#define SX126X_CMD_CALIBRATE                  0x89
#define SX126X_CMD_CALIBRATE_IMAGE            0x98
#define SX126X_CMD_SET_PA_CONFIG              0x95
#define SX126X_CMD_SET_RX_TX_FALLBACK_MODE    0x93

#define SX126X_CMD_WRITE_REGISTER             0x0D
#define SX126X_CMD_READ_REGISTER              0x1D
#define SX126X_CMD_WRITE_BUFFER               0x0E
#define SX126X_CMD_READ_BUFFER                0x1E

#define SX126X_CMD_SET_DIO_IRQ_PARAMS         0x08
#define SX126X_CMD_GET_IRQ_STATUS             0x12
#define SX126X_CMD_CLEAR_IRQ_STATUS           0x02
#define SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL 0x9D
#define SX126X_CMD_SET_DIO3_AS_TCXO_CTRL      0x97

#define SX126X_CMD_SET_RF_FREQUENCY           0x86
#define SX126X_CMD_SET_PACKET_TYPE            0x8A
#define SX126X_CMD_GET_PACKET_TYPE            0x11
#define SX126X_CMD_SET_TX_PARAMS              0x8E
#define SX126X_CMD_SET_MODULATION_PARAMS      0x8B
#define SX126X_CMD_SET_PACKET_PARAMS          0x8C
#define SX126X_CMD_SET_CAD_PARAMS             0x88
#define SX126X_CMD_SET_BUFFER_BASE_ADDRESS    0x8D
#define SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT  0xA0

#define SX126X_CMD_GET_STATUS                 0xC0
#define SX126X_CMD_GET_RX_BUFFER_STATUS       0x14
#define SX126X_CMD_GET_PACKET_STATUS          0x14
#define SX126X_CMD_GET_RSSI_INST              0x15
#define SX126X_CMD_GET_STATS                  0x10
#define SX126X_CMD_RESET_STATS                0x00
#define SX126X_CMD_GET_DEVICE_ERRORS          0x17
#define SX126X_CMD_CLEAR_DEVICE_ERRORS        0x07

// ============================================================================
// REGISTER MAP LOCATIONS
// ============================================================================
#define SX126X_REG_BW1000_OFS                 0x0805
#define SX126X_REG_LORA_CR_RX                 0x0891
#define SX126X_REG_OCP_CONFIG                 0x08E7
#define SX126X_REG_XTA_TRIM                   0x0911
#define SX126X_REG_XTB_TRIM                   0x0912

// ============================================================================
// CONFIGURATION ENUMS & CONSTANTS
// ============================================================================
enum SX1262PacketType_t {
  SX1262_PACKET_TYPE_GFSK = 0x00,
  SX1262_PACKET_TYPE_LORA = 0x01,
  SX1262_PACKET_TYPE_LR_FHSS = 0x03
};

enum SX1262StandbyMode_t {
  SX1262_STDBY_RC = 0x00,
  SX1262_STDBY_XOSC = 0x01
};

enum SX1262RegulatorMode_t {
  SX1262_REGULATOR_LDO = 0x00,
  SX1262_REGULATOR_DC_DC = 0x01
};

enum SX1262LoRaSF_t {
  SX1262_LORA_SF5 = 0x05,
  SX1262_LORA_SF6 = 0x06,
  SX1262_LORA_SF7 = 0x07,
  SX1262_LORA_SF8 = 0x08,
  SX1262_LORA_SF9 = 0x09,
  SX1262_LORA_SF10 = 0x0A,
  SX1262_LORA_SF11 = 0x0B,
  SX1262_LORA_SF12 = 0x0C
};

enum SX1262LoRaBW_t {
  SX1262_LORA_BW_7_8 = 0x00,
  SX1262_LORA_BW_10_4 = 0x01,
  SX1262_LORA_BW_15_6 = 0x02,
  SX1262_LORA_BW_20_8 = 0x03,
  SX1262_LORA_BW_31_25 = 0x04,
  SX1262_LORA_BW_41_7 = 0x05,
  SX1262_LORA_BW_62_5 = 0x06,
  SX1262_LORA_BW_125_0 = 0x07,
  SX1262_LORA_BW_250_0 = 0x08,
  SX1262_LORA_BW_500_0 = 0x09
};

enum SX1262LoRaCR_t {
  SX1262_LORA_CR_4_5 = 0x01,
  SX1262_LORA_CR_4_6 = 0x02,
  SX1262_LORA_CR_4_7 = 0x03,
  SX1262_LORA_CR_4_8 = 0x04
};

#define SX126X_IRQ_TX_DONE                    (1 << 0)
#define SX126X_IRQ_RX_DONE                    (1 << 1)
#define SX126X_IRQ_PREAMBLE_DETECTED          (1 << 2)
#define SX126X_IRQ_SYNC_WORD_VALID            (1 << 3)
#define SX126X_IRQ_HEADER_VALID               (1 << 4)
#define SX126X_IRQ_HEADER_ERR                 (1 << 5)
#define SX126X_IRQ_CRC_ERR                    (1 << 6)
#define SX126X_IRQ_CAD_DONE                   (1 << 7)
#define SX126X_IRQ_CAD_DETECTED               (1 << 8)
#define SX126X_IRQ_TIMEOUT                    (1 << 9)

// ============================================================================
// SX1262 DRIVER CLASS
// ============================================================================
class SX1262 {
  public:
    SX1262(int nssPin, int resetPin, int busyPin, int dio1Pin);

    bool begin(SPIClass &spiBus = SPI);
    void reset();
    void waitOnBusy();

    void writeCommand(uint8_t opCode, uint8_t *buffer, uint16_t length);
    void readCommand(uint8_t opCode, uint8_t *buffer, uint16_t length);
    void writeRegister(uint16_t address, uint8_t *data, uint16_t length);
    void readRegister(uint16_t address, uint8_t *data, uint16_t length);
    void writeBuffer(uint8_t offset, uint8_t *data, uint16_t length);
    void readBuffer(uint8_t offset, uint8_t *data, uint16_t length);

    void setSleep(uint8_t sleepConfig = 0x04);
    void setStandby(SX1262StandbyMode_t mode = SX1262_STDBY_RC);
    void setRegulatorMode(SX1262RegulatorMode_t mode = SX1262_REGULATOR_DC_DC);
    void setPaConfig(uint8_t paDutyCycle, uint8_t hpStartComp, uint8_t deviceSel = 0x00, uint8_t paLut = 0x01);
    void setTxParams(int8_t power, uint8_t rampTime = 0x02);
    void setRfFrequency(uint32_t frequencyHz);
    void setPacketType(SX1262PacketType_t packetType);

    void setLoRaModulationParams(SX1262LoRaSF_t sf, SX1262LoRaBW_t bw, SX1262LoRaCR_t cr, uint8_t ldro = 0x00);
    void setLoRaPacketParams(uint16_t preambleLength, uint8_t headerType, uint8_t payloadLength, uint8_t crcType, uint8_t invertIQ);
    void setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask = 0x0000, uint16_t dio3Mask = 0x0000);

    void setTx(uint32_t timeoutMs = 0);
    void setRx(uint32_t timeoutMs = 0);
    uint16_t getIrqStatus();
    void clearIrqStatus(uint16_t irqMask = 0xFFFF);
    
    uint8_t getStatus();
    void setDIO3AsTCXOCtrl(uint8_t tcxoVoltage, uint32_t delayUs);
    void calibrateImage(uint8_t freq1, uint8_t freq2);

  private:
    int _nss;
    int _reset;
    int _busy;
    int _dio1;
    SPIClass *_spi;
    SPISettings _spiSettings;
};

#endif // SX1262_H