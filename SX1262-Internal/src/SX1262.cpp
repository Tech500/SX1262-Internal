/*
  SX1262.cpp - Implementation for Semtech SX1262 Arduino Library
*/

#include "SX1262.h"

SX1262::SX1262(int nssPin, int resetPin, int busyPin, int dio1Pin)
  : _nss(nssPin), _reset(resetPin), _busy(busyPin), _dio1(dio1Pin), _spi(&SPI), _spiSettings(2000000, MSBFIRST, SPI_MODE0) {}

bool SX1262::begin(SPIClass &spiBus) {
  _spi = &spiBus;
  pinMode(_nss, OUTPUT);
  pinMode(_reset, OUTPUT);
  pinMode(_busy, INPUT);
  if (_dio1 >= 0) pinMode(_dio1, INPUT);

  digitalWrite(_nss, HIGH);

  reset();
  setStandby(SX1262_STDBY_RC);
  return true;
}

void SX1262::reset() {
  digitalWrite(_reset, LOW);
  delay(10);
  digitalWrite(_reset, HIGH);
  delay(20);
  waitOnBusy();
}

void SX1262::waitOnBusy() {
  while (digitalRead(_busy) == HIGH) {
    yield();
  }
}

void SX1262::writeCommand(uint8_t opCode, uint8_t *buffer, uint16_t length) {
  waitOnBusy();
  _spi->beginTransaction(_spiSettings);
  digitalWrite(_nss, LOW);
  _spi->transfer(opCode);
  for (uint16_t i = 0; i < length; i++) {
    _spi->transfer(buffer[i]);
  }
  digitalWrite(_nss, HIGH);
  _spi->endTransaction();
  if (opCode != SX126X_CMD_SET_SLEEP) {
    waitOnBusy();
  }
}

void SX1262::readCommand(uint8_t opCode, uint8_t *buffer, uint16_t length) {
  waitOnBusy();
  _spi->beginTransaction(_spiSettings);
  digitalWrite(_nss, LOW);
  _spi->transfer(opCode);
  _spi->transfer(0x00); // Status/NOP byte
  for (uint16_t i = 0; i < length; i++) {
    buffer[i] = _spi->transfer(0x00);
  }
  digitalWrite(_nss, HIGH);
  _spi->endTransaction();
  waitOnBusy();
}

void SX1262::writeRegister(uint16_t address, uint8_t *data, uint16_t length) {
  waitOnBusy();
  _spi->beginTransaction(_spiSettings);
  digitalWrite(_nss, LOW);
  _spi->transfer(SX126X_CMD_WRITE_REGISTER);
  _spi->transfer((address >> 8) & 0xFF);
  _spi->transfer(address & 0xFF);
  for (uint16_t i = 0; i < length; i++) {
    _spi->transfer(data[i]);
  }
  digitalWrite(_nss, HIGH);
  _spi->endTransaction();
  waitOnBusy();
}

void SX1262::readRegister(uint16_t address, uint8_t *data, uint16_t length) {
  waitOnBusy();
  _spi->beginTransaction(_spiSettings);
  digitalWrite(_nss, LOW);
  _spi->transfer(SX126X_CMD_READ_REGISTER);
  _spi->transfer((address >> 8) & 0xFF);
  _spi->transfer(address & 0xFF);
  _spi->transfer(0x00); // NOP
  for (uint16_t i = 0; i < length; i++) {
    data[i] = _spi->transfer(0x00);
  }
  digitalWrite(_nss, HIGH);
  _spi->endTransaction();
  waitOnBusy();
}

void SX1262::writeBuffer(uint8_t offset, uint8_t *data, uint16_t length) {
  waitOnBusy();
  _spi->beginTransaction(_spiSettings);
  digitalWrite(_nss, LOW);
  _spi->transfer(SX126X_CMD_WRITE_BUFFER);
  _spi->transfer(offset);
  for (uint16_t i = 0; i < length; i++) {
    _spi->transfer(data[i]);
  }
  digitalWrite(_nss, HIGH);
  _spi->endTransaction();
  waitOnBusy();
}

void SX1262::readBuffer(uint8_t offset, uint8_t *data, uint16_t length) {
  waitOnBusy();
  _spi->beginTransaction(_spiSettings);
  digitalWrite(_nss, LOW);
  _spi->transfer(SX126X_CMD_READ_BUFFER);
  _spi->transfer(offset);
  _spi->transfer(0x00); // NOP
  for (uint16_t i = 0; i < length; i++) {
    data[i] = _spi->transfer(0x00);
  }
  digitalWrite(_nss, HIGH);
  _spi->endTransaction();
  waitOnBusy();
}

void SX1262::setSleep(uint8_t sleepConfig) {
  writeCommand(SX126X_CMD_SET_SLEEP, &sleepConfig, 1);
}

void SX1262::setStandby(SX1262StandbyMode_t mode) {
  uint8_t buf = (uint8_t)mode;
  writeCommand(SX126X_CMD_SET_STANDBY, &buf, 1);
}

void SX1262::setRegulatorMode(SX1262RegulatorMode_t mode) {
  uint8_t buf = (uint8_t)mode;
  writeCommand(SX126X_CMD_SET_REGULATOR_MODE, &buf, 1);
}

void SX1262::setPaConfig(uint8_t paDutyCycle, uint8_t hpStartComp, uint8_t deviceSel, uint8_t paLut) {
  uint8_t buf[4] = {paDutyCycle, hpStartComp, deviceSel, paLut};
  writeCommand(SX126X_CMD_SET_PA_CONFIG, buf, 4);
}

void SX1262::setTxParams(int8_t power, uint8_t rampTime) {
  uint8_t buf[2] = {(uint8_t)power, rampTime};
  writeCommand(SX126X_CMD_SET_TX_PARAMS, buf, 2);
}

void SX1262::setRfFrequency(uint32_t frequencyHz) {
  uint32_t frf = (uint32_t)((uint64_t)frequencyHz * (1 << 25) / 32000000);
  uint8_t buf[4] = {
    (uint8_t)((frf >> 24) & 0xFF),
    (uint8_t)((frf >> 16) & 0xFF),
    (uint8_t)((frf >> 8) & 0xFF),
    (uint8_t)(frf & 0xFF)
  };
  writeCommand(SX126X_CMD_SET_RF_FREQUENCY, buf, 4);
}

void SX1262::setPacketType(SX1262PacketType_t packetType) {
  uint8_t buf = (uint8_t)packetType;
  writeCommand(SX126X_CMD_SET_PACKET_TYPE, &buf, 1);
}

void SX1262::setLoRaModulationParams(SX1262LoRaSF_t sf, SX1262LoRaBW_t bw, SX1262LoRaCR_t cr, uint8_t ldro) {
  uint8_t buf[4] = {(uint8_t)sf, (uint8_t)bw, (uint8_t)cr, ldro};
  writeCommand(SX126X_CMD_SET_MODULATION_PARAMS, buf, 4);
}

void SX1262::setLoRaPacketParams(uint16_t preambleLength, uint8_t headerType, uint8_t payloadLength, uint8_t crcType, uint8_t invertIQ) {
  uint8_t buf[6] = {
    (uint8_t)((preambleLength >> 8) & 0xFF),
    (uint8_t)(preambleLength & 0xFF),
    headerType,
    payloadLength,
    crcType,
    invertIQ
  };
  writeCommand(SX126X_CMD_SET_PACKET_PARAMS, buf, 6);
}

void SX1262::setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask) {
  uint8_t buf[8] = {
    (uint8_t)((irqMask >> 8) & 0xFF), (uint8_t)(irqMask & 0xFF),
    (uint8_t)((dio1Mask >> 8) & 0xFF), (uint8_t)(dio1Mask & 0xFF),
    (uint8_t)((dio2Mask >> 8) & 0xFF), (uint8_t)(dio2Mask & 0xFF),
    (uint8_t)((dio3Mask >> 8) & 0xFF), (uint8_t)(dio3Mask & 0xFF)
  };
  writeCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8);
}

void SX1262::setTx(uint32_t timeoutMs) {
  uint32_t timeout = timeoutMs * 64; // Convert ms to 15.625us units
  uint8_t buf[3] = {
    (uint8_t)((timeout >> 16) & 0xFF),
    (uint8_t)((timeout >> 8) & 0xFF),
    (uint8_t)(timeout & 0xFF)
  };
  writeCommand(SX126X_CMD_SET_TX, buf, 3);
}

void SX1262::setRx(uint32_t timeoutMs) {
  uint32_t timeout = (timeoutMs == 0) ? 0xFFFFFF : (timeoutMs * 64);
  uint8_t buf[3] = {
    (uint8_t)((timeout >> 16) & 0xFF),
    (uint8_t)((timeout >> 8) & 0xFF),
    (uint8_t)(timeout & 0xFF)
  };
  writeCommand(SX126X_CMD_SET_RX, buf, 3);
}

uint16_t SX1262::getIrqStatus() {
  uint8_t buf[2] = {0x00, 0x00};
  readCommand(SX126X_CMD_GET_IRQ_STATUS, buf, 2);
  return (buf[0] << 8) | buf[1];
}

void SX1262::clearIrqStatus(uint16_t irqMask) {
  uint8_t buf[2] = {
    (uint8_t)((irqMask >> 8) & 0xFF),
    (uint8_t)(irqMask & 0xFF)
  };
  writeCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2);
}

uint8_t SX1262::getStatus() {
  uint8_t status = 0;
  readCommand(SX126X_CMD_GET_STATUS, &status, 1);
  return status;
}

void SX1262::setDIO3AsTCXOCtrl(uint8_t tcxoVoltage, uint32_t delayUs) {
  uint32_t timeout = (delayUs / 15.625);
  uint8_t buf[4] = {
    tcxoVoltage,
    (uint8_t)((timeout >> 16) & 0xFF),
    (uint8_t)((timeout >> 8) & 0xFF),
    (uint8_t)(timeout & 0xFF)
  };
  writeCommand(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, buf, 4);
}

void SX1262::calibrateImage(uint8_t freq1, uint8_t freq2) {
  uint8_t buf[2] = {freq1, freq2};
  writeCommand(SX126X_CMD_CALIBRATE_IMAGE, buf, 2);
}