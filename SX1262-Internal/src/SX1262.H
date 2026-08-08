#pragma once
#include <Arduino.h>

class SX1262 {
public:
    typedef void (*SpiTransferFn)(const uint8_t* tx, uint8_t* rx, size_t length);
    typedef void (*SetPinFn)(bool level);
    typedef bool (*GetPinFn)();
    typedef void (*DelayMsFn)(uint32_t ms);

    enum class TCXOVoltage : uint8_t { VOLTAGE_1_6V = 0x00 };
    enum class RegulatorMode : uint8_t { USE_DCDC = 0x01 };
    enum class PacketType : uint8_t { LORA = 0x01 };
    enum class StandbyConfig : uint8_t { STDBY_RC = 0x00 };
    enum class IRQMask : uint16_t { 
        NONE = 0x0000, 
        PREAMBLE_DETECTED = 0x0004, 
        HEADER_VALID = 0x0008, 
        RX_DONE = 0x0002, 
        ALL = 0xFFFF 
    };

    struct RxBufferStatus {
        uint8_t payloadLengthRx;
        uint8_t rxStartBufferPointer;
    };

    SX1262(SpiTransferFn spi, SetPinFn nss, SetPinFn rst, GetPinFn busy, DelayMsFn delayMs)
        : transferSpi(spi), setNss(nss), setRst(rst), getBusy(busy), delay(delayMs) {}

    inline void reset() {
        if (setRst) {
            setRst(false);
            if (delay) delay(10);
            setRst(true);
            if (delay) delay(10);
        }
    }

    inline void setStandby(StandbyConfig config) {
        uint8_t buf[2] = { 0x80, static_cast<uint8_t>(config) };
        writeCommand(buf, 2);
    }

    inline void setDIO3AsTCXOCtrl(TCXOVoltage voltage, uint32_t delayUs) {
        uint8_t buf[5] = {
            0x97,
            static_cast<uint8_t>(voltage),
            static_cast<uint8_t>((delayUs >> 16) & 0xFF),
            static_cast<uint8_t>((delayUs >> 8) & 0xFF),
            static_cast<uint8_t>(delayUs & 0xFF)
        };
        writeCommand(buf, 5);
    }

    inline void setRegulatorMode(RegulatorMode mode) {
        uint8_t buf[2] = { 0x96, static_cast<uint8_t>(mode) };
        writeCommand(buf, 2);
    }

    inline void setPacketType(PacketType type) {
        uint8_t buf[2] = { 0x8A, static_cast<uint8_t>(type) };
        writeCommand(buf, 2);
    }

    inline void setRfFrequency(uint32_t freqHz) {
        uint64_t frf = ((uint64_t)freqHz << 25) / 32000000;
        uint8_t buf[5] = {
            0x86,
            static_cast<uint8_t>((frf >> 24) & 0xFF),
            static_cast<uint8_t>((frf >> 16) & 0xFF),
            static_cast<uint8_t>((frf >> 8) & 0xFF),
            static_cast<uint8_t>(frf & 0xFF)
        };
        writeCommand(buf, 5);
    }

    inline void setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask) {
        uint8_t buf[9] = {
            0x08,
            static_cast<uint8_t>((irqMask >> 8) & 0xFF), static_cast<uint8_t>(irqMask & 0xFF),
            static_cast<uint8_t>((dio1Mask >> 8) & 0xFF), static_cast<uint8_t>(dio1Mask & 0xFF),
            static_cast<uint8_t>((dio2Mask >> 8) & 0xFF), static_cast<uint8_t>(dio2Mask & 0xFF),
            static_cast<uint8_t>((dio3Mask >> 8) & 0xFF), static_cast<uint8_t>(dio3Mask & 0xFF)
        };
        writeCommand(buf, 9);
    }

    inline uint16_t getIrqStatus() {
        uint8_t cmd[4] = { 0x12, 0x00, 0x00, 0x00 };
        uint8_t rx[4] = { 0 };
        executeSpi(cmd, rx, 4);
        return (static_cast<uint16_t>(rx[2]) << 8) | rx[3];
    }

    inline void clearIrqStatus(uint16_t clearMask) {
        uint8_t buf[3] = {
            0x02,
            static_cast<uint8_t>((clearMask >> 8) & 0xFF),
            static_cast<uint8_t>(clearMask & 0xFF)
        };
        writeCommand(buf, 3);
    }

    inline RxBufferStatus getRxBufferStatus() {
        uint8_t cmd[4] = { 0x13, 0x00, 0x00, 0x00 };
        uint8_t rx[4] = { 0 };
        executeSpi(cmd, rx, 4);
        return { rx[2], rx[3] };
    }

    inline void readBuffer(uint8_t offset, uint8_t* data, size_t length) {
        if (!data || length == 0) return;
        waitOnBusy();
        if (setNss) setNss(false);
        uint8_t header[3] = { 0x1E, offset, 0x00 };
        if (transferSpi) transferSpi(header, nullptr, 3);
        if (transferSpi) transferSpi(nullptr, data, length);
        if (setNss) setNss(true);
    }

private:
    SpiTransferFn transferSpi;
    SetPinFn setNss;
    SetPinFn setRst;
    GetPinFn getBusy;
    DelayMsFn delay;

    inline void waitOnBusy() {
        if (!getBusy) return;
        uint32_t start = millis();
        while (getBusy()) {
            if (millis() - start > 1000) break;
        }
    }

    inline void writeCommand(const uint8_t* buffer, size_t size) {
        waitOnBusy();
        if (setNss) setNss(false);
        if (transferSpi) transferSpi(buffer, nullptr, size);
        if (setNss) setNss(true);
    }

    inline void executeSpi(const uint8_t* tx, uint8_t* rx, size_t size) {
        waitOnBusy();
        if (setNss) setNss(false);
        if (transferSpi) transferSpi(tx, rx, size);
        if (setNss) setNss(true);
    }
};