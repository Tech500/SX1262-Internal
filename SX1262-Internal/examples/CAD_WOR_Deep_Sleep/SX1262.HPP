//SX1262.h
//Chrome AI Mode

#include "SX1262.h"

SX1262::SX1262(SpiTransferFn spiTransfer, DigitalWriteFn setNss, 
               DigitalWriteFn setNreset, DigitalReadFn getBusy, DelayMsFn delayMs)
    : m_spiTransfer(spiTransfer), m_setNss(setNss), 
      m_setNreset(setNreset), m_getBusy(getBusy), m_delayMs(delayMs) {
    m_setNss(true);
}

void SX1262::waitUntilNotBusy() {
    while (m_getBusy()) {
        // Wait until BUSY line goes low
    }
}

void SX1262::reset() {
    m_setNreset(false);
    m_delayMs(1);
    m_setNreset(true);
    m_delayMs(10);
    waitUntilNotBusy();
}

void SX1262::writeCommand(OpCode opCode, const uint8_t* params, size_t paramLength) {
    waitUntilNotBusy();
    m_setNss(false);

    uint8_t cmd = static_cast<uint8_t>(opCode);
    m_spiTransfer(&cmd, nullptr, 1);
    if (paramLength > 0 && params != nullptr) {
        m_spiTransfer(params, nullptr, paramLength);
    }

    m_setNss(true);
    waitUntilNotBusy();
}

void SX1262::readCommand(OpCode opCode, uint8_t* data, size_t dataLength) {
    waitUntilNotBusy();
    m_setNss(false);

    uint8_t cmd[2] = { static_cast<uint8_t>(opCode), 0x00 }; // OpCode + NOP byte
    m_spiTransfer(cmd, nullptr, 2);
    m_spiTransfer(nullptr, data, dataLength);

    m_setNss(true);
    waitUntilNotBusy();
}

void SX1262::setStandby(StandbyConfig config) {
    uint8_t param = static_cast<uint8_t>(config);
    writeCommand(OpCode::SET_STANDBY, &param, 1);
}

void SX1262::setRegulatorMode(RegulatorMode mode) {
    uint8_t param = static_cast<uint8_t>(mode);
    writeCommand(OpCode::SET_REGULATOR_MODE, &param, 1);
}

void SX1262::setPaConfig(const PaConfig& config) {
    uint8_t params[4] = {
        config.paDutyCycle,
        config.hpStart,
        config.deviceSelect,
        config.paLut
    };
    writeCommand(OpCode::SET_PA_CONFIG, params, 4);
}

void SX1262::setTxParams(int8_t power, RampTime rampTime) {
    uint8_t params[2] = {
        static_cast<uint8_t>(power),
        static_cast<uint8_t>(rampTime)
    };
    writeCommand(OpCode::SET_TX_PARAMS, params, 2);
}

void SX1262::setPacketType(PacketType type) {
    uint8_t param = static_cast<uint8_t>(type);
    writeCommand(OpCode::SET_PACKET_TYPE, &param, 1);
}

void SX1262::setRfFrequency(uint32_t frequencyHz) {
    // Freq = (frequencyHz * 2^25) / FXOSC (32 MHz)
    uint32_t freqReg = static_cast<uint32_t>((static_cast<uint64_t>(frequencyHz) << 25) / 32000000);
    uint8_t params[4] = {
        static_cast<uint8_t>((freqReg >> 24) & 0xFF),
        static_cast<uint8_t>((freqReg >> 16) & 0xFF),
        static_cast<uint8_t>((freqReg >> 8) & 0xFF),
        static_cast<uint8_t>(freqReg & 0xFF)
    };
    writeCommand(OpCode::SET_RF_FREQUENCY, params, 4);
}

void SX1262::setBufferBaseAddress(uint8_t txBaseAddr, uint8_t rxBaseAddr) {
    uint8_t params[2] = { txBaseAddr, rxBaseAddr };
    writeCommand(OpCode::SET_BUFFER_BASE_ADDRESS, params, 2);
}

void SX1262::setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask) {
    uint8_t params[8] = {
        static_cast<uint8_t>((irqMask >> 8) & 0xFF), static_cast<uint8_t>(irqMask & 0xFF),
        static_cast<uint8_t>((dio1Mask >> 8) & 0xFF), static_cast<uint8_t>(dio1Mask & 0xFF),
        static_cast<uint8_t>((dio2Mask >> 8) & 0xFF), static_cast<uint8_t>(dio2Mask & 0xFF),
        static_cast<uint8_t>((dio3Mask >> 8) & 0xFF), static_cast<uint8_t>(dio3Mask & 0xFF)
    };
    writeCommand(OpCode::SET_DIO_IRQ_PARAMS, params, 8);
}

void SX1262::clearIrqStatus(uint16_t clearIrqMask) {
    uint8_t params[2] = {
        static_cast<uint8_t>((clearIrqMask >> 8) & 0xFF),
        static_cast<uint8_t>(clearIrqMask & 0xFF)
    };
    writeCommand(OpCode::CLEAR_IRQ_STATUS, params, 2);
}

uint16_t SX1262::getIrqStatus() {
    uint8_t data[2] = {0};
    readCommand(OpCode::GET_IRQ_STATUS, data, 2);
    return (static_cast<uint16_t>(data[0]) << 8) | data[1];
}

void SX1262::setDIO2AsRfSwitchCtrl(bool enable) {
    uint8_t param = enable ? 0x01 : 0x00;
    writeCommand(OpCode::SET_DIO2_AS_RF_SWITCH_CTRL, &param, 1);
}

void SX1262::setDIO3AsTCXOCtrl(TCXOVoltage voltage, uint32_t delayUs) {
    uint32_t timeout = (delayUs * 64) / 1000; // 15.625 us steps
    uint8_t params[4] = {
        static_cast<uint8_t>(voltage),
        static_cast<uint8_t>((timeout >> 16) & 0xFF),
        static_cast<uint8_t>((timeout >> 8) & 0xFF),
        static_cast<uint8_t>(timeout & 0xFF)
    };
    writeCommand(OpCode::SET_DIO3_AS_TCXO_CTRL, params, 4);
}

void SX1262::setTx(uint32_t timeoutMs) {
    uint32_t timeout = timeoutMs * 64; // 15.625 us steps
    uint8_t params[3] = {
        static_cast<uint8_t>((timeout >> 16) & 0xFF),
        static_cast<uint8_t>((timeout >> 8) & 0xFF),
        static_cast<uint8_t>(timeout & 0xFF)
    };
    writeCommand(OpCode::SET_TX, params, 3);
}

void SX1262::setRx(uint32_t timeoutMs) {
    uint32_t timeout = timeoutMs * 64; // 15.625 us steps
    uint8_t params[3] = {
        static_cast<uint8_t>((timeout >> 16) & 0xFF),
        static_cast<uint8_t>((timeout >> 8) & 0xFF),
        static_cast<uint8_t>(timeout & 0xFF)
    };
    writeCommand(OpCode::SET_RX, params, 3);
}

void SX1262::writeBuffer(uint8_t offset, const uint8_t* data, size_t length) {
    waitUntilNotBusy();
    m_setNss(false);

    uint8_t header[2] = { static_cast<uint8_t>(OpCode::WRITE_BUFFER), offset };
    m_spiTransfer(header, nullptr, 2);
    m_spiTransfer(data, nullptr, length);

    m_setNss(true);
    waitUntilNotBusy();
}

void SX1262::readBuffer(uint8_t offset, uint8_t* data, size_t length) {
    waitUntilNotBusy();
    m_setNss(false);

    uint8_t header[3] = { static_cast<uint8_t>(OpCode::READ_BUFFER), offset, 0x00 };
    m_spiTransfer(header, nullptr, 3);
    m_spiTransfer(nullptr, data, length);

    m_setNss(true);
    waitUntilNotBusy();
}

SX1262::RxBufferStatus SX1262::getRxBufferStatus() {
    uint8_t status[2] = {0};
    readCommand(OpCode::GET_RX_BUFFER_STATUS, status, 2);
    return RxBufferStatus{ status[0], status[1] };
}
