//SX1262.cpp
//Chrome AI Mode


#ifndef SX1262_HPP
#define SX1262_HPP

#include <cstdint>
#include <cstddef>

class SX1262 {
public:
    // =========================================================================
    // Enums and Constants
    // =========================================================================

    // Opcodes (List of Commands)
    enum class OpCode : uint8_t {
        // Operational Modes Functions
        SET_SLEEP                  = 0x84,
        SET_STANDBY                = 0x80,
        SET_FS                     = 0xC1,
        SET_TX                     = 0x83,
        SET_RX                     = 0x82,
        STOP_TIMER_ON_PREAMBLE     = 0x9F,
        SET_RX_DUTY_CYCLE          = 0x94,
        SET_CAD                    = 0xC5,
        SET_TX_CONTINUOUS_WAVE     = 0xD1,
        SET_TX_INFINITE_PREAMBLE   = 0xD2,
        SET_REGULATOR_MODE         = 0x96,
        CALIBRATE                  = 0x89,
        CALIBRATE_IMAGE            = 0x98,
        SET_PA_CONFIG              = 0x95,
        SET_RX_TX_FALLBACK_MODE    = 0x93,

        // Register and Buffer Access Commands
        WRITE_REGISTER             = 0x0D,
        READ_REGISTER              = 0x1D,
        WRITE_BUFFER               = 0x0E,
        READ_BUFFER                = 0x1E,

        // DIO and IRQ Control
        SET_DIO_IRQ_PARAMS         = 0x08,
        GET_IRQ_STATUS             = 0x12,
        CLEAR_IRQ_STATUS           = 0x02,
        SET_DIO2_AS_RF_SWITCH_CTRL = 0x9D,
        SET_DIO3_AS_TCXO_CTRL      = 0x97,

        // RF, Modulation and Packet Commands
        SET_RF_FREQUENCY           = 0x86,
        SET_PACKET_TYPE            = 0x8A,
        GET_PACKET_TYPE            = 0x11,
        SET_TX_PARAMS              = 0x8E,
        SET_MODULATION_PARAMS      = 0x8B,
        SET_PACKET_PARAMS          = 0x8C,
        SET_CAD_PARAMS             = 0x88,
        SET_BUFFER_BASE_ADDRESS    = 0x8F,
        SET_LORA_SYMB_NUM_TIMEOUT  = 0xA0,

        // Communication Status Information
        GET_STATUS                 = 0xC0,
        GET_RX_BUFFER_STATUS       = 0x14,
        GET_PACKET_STATUS          = 0x17,
        GET_RSSI_INST              = 0x15,
        GET_STATS                  = 0x10,
        RESET_STATS                = 0x00,

        // Miscellaneous
        GET_DEVICE_ERRORS          = 0x17, // Shared opcode pattern
        CLEAR_DEVICE_ERRORS        = 0x07
    };

    // Register Map Addresses
    enum class Register : uint16_t {
        WHITENING_INITIAL_MSB = 0x06B8,
        WHITENING_INITIAL_LSB = 0x06B9,
        CRC_MSB_INITIAL       = 0x06BC,
        CRC_LSB_INITIAL       = 0x06BD,
        CRC_MSB_POLYNOMIAL    = 0x06BE,
        CRC_LSB_POLYNOMIAL    = 0x06BF,
        OCP_CONFIGURATION     = 0x08E7,
        XTA_TRIM              = 0x0911,
        XTB_TRIM              = 0x0912
    };

    // Parameter Types
    enum class PacketType : uint8_t {
        GFSK = 0x00,
        LORA = 0x01
    };

    enum class StandbyConfig : uint8_t {
        STDBY_RC   = 0x00,
        STDBY_XOSC = 0x01
    };

    enum class RegulatorMode : uint8_t {
        USE_LDO  = 0x00,
        USE_DCDC = 0x01
    };

    enum class TCXOVoltage : uint8_t {
        VOLTAGE_1_6V = 0x00,
        VOLTAGE_1_7V = 0x01,
        VOLTAGE_1_8V = 0x02,
        VOLTAGE_2_2V = 0x03,
        VOLTAGE_2_4V = 0x04,
        VOLTAGE_2_7V = 0x05,
        VOLTAGE_3_0V = 0x06,
        VOLTAGE_3_3V = 0x07
    };

    enum class RampTime : uint8_t {
        RAMP_10U   = 0x00,
        RAMP_20U   = 0x01,
        RAMP_40U   = 0x02,
        RAMP_80U   = 0x03,
        RAMP_200U  = 0x04,
        RAMP_800U  = 0x05,
        RAMP_1700U = 0x06,
        RAMP_3400U = 0x07
    };

    enum class IRQMask : uint16_t {
        NONE             = 0x0000,
        TX_DONE          = 0x0001,
        RX_DONE          = 0x0002,
        PREAMBLE_DETECTED = 0x0004,
        SYNC_WORD_VALID  = 0x0008,
        HEADER_VALID     = 0x0010,
        HEADER_ERR       = 0x0020,
        CRC_ERR          = 0x0040,
        CAD_DONE         = 0x0080,
        CAD_DETECTED     = 0x0100,
        TIMEOUT          = 0x0200,
        ALL              = 0x03FF
    };

    // Structures
    struct PaConfig {
        uint8_t paDutyCycle;
        uint8_t hpStart;
        uint8_t deviceSelect; // 0x00 = SX1262, 0x01 = SX1261
        uint8_t paLut;
    };

    struct RxBufferStatus {
        uint8_t payloadLengthRx;
        uint8_t rxStartBufferPointer;
    };

    // Hardware Abstraction Function Pointers
    using SpiTransferFn = void(*)(const uint8_t* tx, uint8_t* rx, size_t length);
    using DigitalWriteFn = void(*)(bool level);
    using DigitalReadFn  = bool(*)();
    using DelayMsFn      = void(*)(uint32_t ms);

    // =========================================================================
    // Constructor & Control
    // =========================================================================

    SX1262(SpiTransferFn spiTransfer,
           DigitalWriteFn setNss,
           DigitalWriteFn setNreset,
           DigitalReadFn getBusy,
           DelayMsFn delayMs);

    void reset();
    void waitUntilNotBusy();

    // =========================================================================
    // API Driver Commands
    // =========================================================================

    void setStandby(StandbyConfig config);
    void setRegulatorMode(RegulatorMode mode);
    void setPaConfig(const PaConfig& config);
    void setTxParams(int8_t power, RampTime rampTime);
    void setPacketType(PacketType type);
    void setRfFrequency(uint32_t frequencyHz);
    void setBufferBaseAddress(uint8_t txBaseAddr, uint8_t rxBaseAddr);
    void setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
    void clearIrqStatus(uint16_t clearIrqMask);
    uint16_t getIrqStatus();
    
    void setDIO2AsRfSwitchCtrl(bool enable);
    void setDIO3AsTCXOCtrl(TCXOVoltage voltage, uint32_t delayUs);

    void setTx(uint32_t timeoutMs);
    void setRx(uint32_t timeoutMs);

    void writeRegister(Register reg, const uint8_t* data, size_t length);
    void readRegister(Register reg, uint8_t* data, size_t length);
    void writeBuffer(uint8_t offset, const uint8_t* data, size_t length);
    void readBuffer(uint8_t offset, uint8_t* data, size_t length);

    RxBufferStatus getRxBufferStatus();

private:
    SpiTransferFn m_spiTransfer;
    DigitalWriteFn m_setNss;
    DigitalWriteFn m_setNreset;
    DigitalReadFn  m_getBusy;
    DelayMsFn      m_delayMs;

    void writeCommand(OpCode opCode, const uint8_t* params, size_t paramLength);
    void readCommand(OpCode opCode, uint8_t* data, size_t dataLength);
};

#endif // SX1262_HPP