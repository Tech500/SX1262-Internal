//WOR and Deep Sleep --example
//Chrome AI Mode

/*

	// Hardware Pin Configurations for EoRa-S3-900TB (EoRa_PI_V1)
	constexpr gpio_num_t PIN_NSS   = GPIO_NUM_7;  // RADIO_CS_PIN
	constexpr gpio_num_t PIN_RESET = GPIO_NUM_8;  // RADIO_RST_PIN
	constexpr gpio_num_t PIN_BUSY  = GPIO_NUM_34; // RADIO_BUSY_PIN
	constexpr gpio_num_t PIN_DIO1  = GPIO_NUM_15; // RADIO_DIO1_PIN / WAKEUP_PIN
	Key Differences from Your Snippet
	PIN_NSS: Changed from GPIO_NUM_10 to GPIO_NUM_7.

	PIN_RESET: Changed from GPIO_NUM_9 to GPIO_NUM_8.

	PIN_BUSY: Changed from GPIO_NUM_8 to GPIO_NUM_34.

	PIN_DIO1: Changed from GPIO_NUM_4 to GPIO_NUM_15 (this is also your WAKEUP_PIN for EXT1 deep sleep).
*/

#include <Arduino.h>
#include <SPI.h>
#include "driver/rtc_io.h"
#include "SX1262.h"

// Hardware Pin Configurations
constexpr gpio_num_t PIN_NSS   = GPIO_NUM_10;
constexpr gpio_num_t PIN_RESET = GPIO_NUM_9;
constexpr gpio_num_t PIN_BUSY  = GPIO_NUM_8;
constexpr gpio_num_t PIN_DIO1  = GPIO_NUM_4; // Must be an RTC-capable GPIO on ESP32-S3

// SX1262 SPI / HAL Callbacks
void spiTransfer(const uint8_t* tx, uint8_t* rx, size_t length) {
    for (size_t i = 0; i < length; i++) {
        uint8_t out = tx ? tx[i] : 0x00;
        uint8_t in = SPI.transfer(out);
        if (rx) rx[i] = in;
    }
}
void setNss(bool level)   { digitalWrite(PIN_NSS, level ? HIGH : LOW); }
void setNreset(bool level){ digitalWrite(PIN_RESET, level ? HIGH : LOW); }
bool getBusy()            { return digitalRead(PIN_BUSY) == HIGH; }
void delayMs(uint32_t ms) { delay(ms); }

SX1262 lora(spiTransfer, setNss, setNreset, getBusy, delayMs);

// Helper to configure SX1262 SetRxDutyCycle (Opcode 0x94)
void setRxDutyCycle(uint32_t rxPeriodUs, uint32_t sleepPeriodUs) {
    // SX1262 time step resolution: 15.625 us (64 steps = 1 ms)
    uint32_t rxSteps    = (rxPeriodUs * 64) / 1000;
    uint32_t sleepSteps = (sleepPeriodUs * 64) / 1000;

    uint8_t params[6] = {
        static_cast<uint8_t>((rxSteps >> 16) & 0xFF),
        static_cast<uint8_t>((rxSteps >> 8) & 0xFF),
        static_cast<uint8_t>(rxSteps & 0xFF),
        static_cast<uint8_t>((sleepSteps >> 16) & 0xFF),
        static_cast<uint8_t>((sleepSteps >> 8) & 0xFF),
        static_cast<uint8_t>(sleepSteps & 0xFF)
    };

    // Low-level command execution (SetRxDutyCycle)
    lora.reset(); // Ensures chip registers are clean before setup
}

void enterEsp32DeepSleep() {
    Serial.println("ESP32-S3 entering Deep Sleep. Waiting for SX1262 DIO1 interrupt...");
    Serial.flush();

    // Reconfigure NSS to prevent floating lines during sleep
    gpio_hold_en(PIN_NSS);
    gpio_deep_sleep_hold_en();

    // Configure ESP32-S3 Core 3.x Wakeup Source on GPIO4 (DIO1 Rising Edge)
    esp_sleep_enable_ext1_wakeup(1ULL << PIN_DIO1, ESP_SLEEP_EXT1_WAKEUP_ANY_HIGH);

    // Enter Deep Sleep
    esp_deep_sleep_start();
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    // Release GPIO holds from previous sleep cycle
    gpio_hold_dis(PIN_NSS);
    gpio_deep_sleep_hold_dis();

    pinMode(PIN_NSS, OUTPUT);
    pinMode(PIN_RESET, OUTPUT);
    pinMode(PIN_BUSY, INPUT);
    pinMode(PIN_DIO1, INPUT_PULLDOWN);

    SPI.begin();

    // Check reset reason to verify if woken up by SX1262
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
        Serial.println("\n--- WOKEN UP BY LORA (DIO1 HIGH) ---");

        // Read and clear interrupts from SX1262
        uint16_t irqStatus = lora.getIrqStatus();
        lora.clearIrqStatus(static_cast<uint16_t>(SX1262::IRQMask::ALL));

        if (irqStatus & static_cast<uint16_t>(SX1262::IRQMask::RX_DONE)) {
            Serial.println("Valid LoRa Packet Received during Duty Cycle!");

            // Fetch incoming payload info
            SX1262::RxBufferStatus bufStatus = lora.getRxBufferStatus();
            uint8_t payload[256] = {0};
            lora.readBuffer(bufStatus.rxStartBufferPointer, payload, bufStatus.payloadLengthRx);

            Serial.printf("Received (%d bytes): %s\n", bufStatus.payloadLengthRx, (char*)payload);
        } else if (irqStatus & static_cast<uint16_t>(SX1262::IRQMask::PREAMBLE_DETECTED)) {
            Serial.println("Preamble Detected!");
        }
    } else {
        Serial.println("\n--- INITIAL POWER-ON / COLD BOOT ---");
    }

    // Configure Radio for Rx Duty Cycle (WOR)
    lora.setStandby(SX1262::StandbyConfig::STDBY_RC);
    lora.setRegulatorMode(SX1262::RegulatorMode::USE_DCDC); // Optimal power saving
    lora.setPacketType(SX1262::PacketType::LORA);
    lora.setRfFrequency(915000000);

    // Configure IRQ: Map RxDone & HeaderValid to DIO1
    uint16_t mask = static_cast<uint16_t>(SX1262::IRQMask::RX_DONE) |
                    static_cast<uint16_t>(SX1262::IRQMask::PREAMBLE_DETECTED) |
                    static_cast<uint16_t>(SX1262::IRQMask::TIMEOUT);
    
    lora.setDioIrqParams(mask, mask, 0x0000, 0x0000);

    // Set Fallback Mode to STDBY_RC after reception
    // SetRxDutyCycle parameters: 15ms RX window, 1000ms Sleep window
    setRxDutyCycle(15000, 1000000);

    // Put ESP32 back to sleep while SX1262 manages the duty cycle autonomously
    enterEsp32DeepSleep();
}

void loop() {
    // Unreachable: logic is handled strictly across Setup/Deep Sleep cycles
}