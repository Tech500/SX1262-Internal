//CAD + WOR + Deep Sleep example
//Chrome AI Mode


#include <Arduino.h>
#include <SPI.h>
#include "driver/rtc_io.h"
#include "SX1262.h"

// Hardware Pin Definitions for EoRa-S3-900TB
constexpr gpio_num_t PIN_SCLK = GPIO_NUM_5;
constexpr gpio_num_t PIN_MISO = GPIO_NUM_3;
constexpr gpio_num_t PIN_MOSI = GPIO_NUM_6;
constexpr gpio_num_t PIN_NSS  = GPIO_NUM_7;   // CS
constexpr gpio_num_t PIN_RST  = GPIO_NUM_8;   // RST
constexpr gpio_num_t PIN_BUSY = GPIO_NUM_34;  // BUSY
constexpr gpio_num_t PIN_DIO1 = GPIO_NUM_15;  // DIO1 / WAKEUP

// HAL Function Wrappers
void halSpiTransfer(const uint8_t* tx, uint8_t* rx, size_t length) {
    for (size_t i = 0; i < length; i++) {
        uint8_t out = tx ? tx[i] : 0x00;
        uint8_t in = SPI.transfer(out);
        if (rx) rx[i] = in;
    }
}
void halSetNss(bool level)   { digitalWrite(PIN_NSS, level ? HIGH : LOW); }
void halSetNreset(bool level){ digitalWrite(PIN_RST, level ? HIGH : LOW); }
bool halGetBusy()            { return digitalRead(PIN_BUSY) == HIGH; }
void halDelayMs(uint32_t ms) { delay(ms); }

SX1262 lora(halSpiTransfer, halSetNss, halSetNreset, halGetBusy, halDelayMs);

// Helper 1: Low-Level SetRxDutyCycle (Opcode 0x94)
void setRxDutyCycle(uint32_t rxPeriodMs, uint32_t sleepPeriodMs) {
    // SX1262 step resolution: 15.625 us (64 steps = 1 ms)
    uint32_t rxSteps    = rxPeriodMs * 64;
    uint32_t sleepSteps = sleepPeriodMs * 64;

    uint8_t buf[7] = {
        0x94, // Opcode: SetRxDutyCycle
        static_cast<uint8_t>((rxSteps >> 16) & 0xFF),
        static_cast<uint8_t>((rxSteps >> 8) & 0xFF),
        static_cast<uint8_t>(rxSteps & 0xFF),
        static_cast<uint8_t>((sleepSteps >> 16) & 0xFF),
        static_cast<uint8_t>((sleepSteps >> 8) & 0xFF),
        static_cast<uint8_t>(sleepSteps & 0xFF)
    };

    digitalWrite(PIN_NSS, LOW);
    for (size_t i = 0; i < 7; i++) {
        SPI.transfer(buf[i]);
    }
    digitalWrite(PIN_NSS, HIGH);
}

// Helper 2: Matches RadioLib Settings (SF7, BW 125k, CR 4/7, Private Sync Word)
void configureRadioParams() {
    lora.setPacketType(SX1262::PacketType::LORA);
    lora.setRfFrequency(915000000);

    // Modulation Params: SF7, BW 125kHz, CR 4/7 (0x03), LDRO OFF
    uint8_t modParams[5] = { 0x8B, 0x07, 0x04, 0x03, 0x00 };
    digitalWrite(PIN_NSS, LOW);
    for (int i = 0; i < 5; i++) SPI.transfer(modParams[i]);
    digitalWrite(PIN_NSS, HIGH);
    delay(5);

    // Packet Params: Preamble 5000 symbols (0x1388), Explicit Header, Max Len 255, CRC ON
    uint8_t pktParams[7] = { 0x8C, 0x13, 0x88, 0x00, 0xFF, 0x01, 0x00 };
    digitalWrite(PIN_NSS, LOW);
    for (int i = 0; i < 7; i++) SPI.transfer(pktParams[i]);
    digitalWrite(PIN_NSS, HIGH);
    delay(5);

    // Set Private Sync Word (0x1424) -> Matches RADIOLIB_SX126X_SYNC_WORD_PRIVATE
    uint8_t syncWordCmd[5] = { 0x0D, 0x07, 0x40, 0x14, 0x24 };
    digitalWrite(PIN_NSS, LOW);
    for (int i = 0; i < 5; i++) SPI.transfer(syncWordCmd[i]);
    digitalWrite(PIN_NSS, HIGH);
    delay(5);
}

void enterDeepSleepWOR() {
    // 1. Hardware setup
    lora.setDIO3AsTCXOCtrl(SX1262::TCXOVoltage::VOLTAGE_1_6V, 5000);
    lora.setRegulatorMode(SX1262::RegulatorMode::USE_DCDC);

    // 2. Configure matched RadioLib parameters
    configureRadioParams();

    // 3. Map interrupts to DIO1 pin
    lora.clearIrqStatus(static_cast<uint16_t>(SX1262::IRQMask::ALL));
    uint16_t irqMask = static_cast<uint16_t>(SX1262::IRQMask::RX_DONE) |
                      static_cast<uint16_t>(SX1262::IRQMask::PREAMBLE_DETECTED) |
                      static_cast<uint16_t>(SX1262::IRQMask::HEADER_VALID);

    lora.setDioIrqParams(irqMask, irqMask, 0x0000, 0x0000);

    // 4. Arm Duty Cycle (10ms Wake, 500ms Sleep)
    setRxDutyCycle(10, 500);
    delay(5);

    // 5. Isolate SPI & enable ESP32-S3 EXT1 wakeup on DIO1
    SPI.end();
    pinMode(PIN_NSS, OUTPUT);
    digitalWrite(PIN_NSS, HIGH);
    gpio_hold_en(PIN_NSS);
    gpio_deep_sleep_hold_en();

    pinMode(PIN_DIO1, INPUT_PULLDOWN);
    esp_sleep_enable_ext1_wakeup(1ULL << PIN_DIO1, ESP_EXT1_WAKEUP_ANY_HIGH);

    Serial.println(F("=== WOR Active (500ms Cycle) -> Deep Sleep ==="));
    Serial.flush();

    esp_deep_sleep_start();
}

void setup() {
    gpio_hold_dis(PIN_NSS);
    gpio_deep_sleep_hold_dis();

    Serial.begin(115200);
    while (!Serial && millis() < 1500);

    Serial.println(F("\n--- EoRa-S3-900TB WOR Receiver Node ---"));

    pinMode(PIN_NSS, OUTPUT);
    pinMode(PIN_RST, OUTPUT);
    pinMode(PIN_BUSY, INPUT);

    SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_NSS);
    lora.reset();

    // Check Wakeup cause
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {
        Serial.println(F("[WAKE] Woken up by DIO1 High (EXT1)!"));

        uint16_t irqStatus = lora.getIrqStatus();
        lora.clearIrqStatus(static_cast<uint16_t>(SX1262::IRQMask::ALL));

        if (irqStatus & static_cast<uint16_t>(SX1262::IRQMask::RX_DONE)) {
            SX1262::RxBufferStatus buf = lora.getRxBufferStatus();
            uint8_t payload[256] = {0};
            lora.readBuffer(buf.rxStartBufferPointer, payload, buf.payloadLengthRx);
            Serial.printf("[LORA RX] Packet Received (%d bytes): %s\n", buf.payloadLengthRx, (char*)payload);
        }
    } else {
        Serial.println(F("[BOOT] Cold start reset complete."));
    }

    // Re-arm WOR and return to sleep
    enterDeepSleepWOR();
}

void loop() {
    // Unused
}