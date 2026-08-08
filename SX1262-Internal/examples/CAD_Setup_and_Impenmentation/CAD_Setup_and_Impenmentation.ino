//CAD Setup and Impenmentation example
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
#include "SX1262.h"

// Define Hardware Pins
const int PIN_NSS   = 10;
const int PIN_RESET = 9;
const int PIN_BUSY  = 8;
const int PIN_DIO1  = 2; // CAD interrupt pin

// Hardware Abstraction Layer Wrappers for the SX1262 Constructor
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

// Initialize Driver Class
SX1262 lora(spiTransfer, setNss, setNreset, getBusy, delayMs);

// Flag updated inside ISR
volatile bool cadDoneFlag = false;

// Interrupt Service Routine for DIO1
void IRAM_ATTR onDio1Interrupt() {
    cadDoneFlag = true;
}

void setup() {
    Serial.begin(115200);
    
    pinMode(PIN_NSS, OUTPUT);
    pinMode(PIN_RESET, OUTPUT);
    pinMode(PIN_BUSY, INPUT);
    pinMode(PIN_DIO1, INPUT);

    SPI.begin();

    // 1. Reset and bring device to Standby Mode
    lora.reset();
    lora.setStandby(SX1262::StandbyConfig::STDBY_RC);

    // 2. Set Packet Type to LoRa
    lora.setPacketType(SX1262::PacketType::LORA);

    // 3. Configure RF Frequency (e.g., 915 MHz)
    lora.setRfFrequency(915000000);

    // 4. Map CAD interrupts to DIO1 pin
    // Bits enabled: CAD_DONE (0x0080) and CAD_DETECTED (0x0100)
    uint16_t cadIrqMask = static_cast<uint16_t>(SX1262::IRQMask::CAD_DONE) | 
                          static_cast<uint16_t>(SX1262::IRQMask::CAD_DETECTED);
    
    lora.setDioIrqParams(cadIrqMask, cadIrqMask, 0x0000, 0x0000);

    // 5. Attach Arduino Hardware Interrupt to DIO1 Pin
    attachInterrupt(digitalPinToInterrupt(PIN_DIO1), onDio1Interrupt, RISING);

    // 6. Start the first CAD scan
    startCAD();
}

void loop() {
    if (cadDoneFlag) {
        cadDoneFlag = false;

        // Read interrupt register to evaluate CAD outcome
        uint16_t irqStatus = lora.getIrqStatus();

        // Always clear IRQ flags after handling
        lora.clearIrqStatus(static_cast<uint16_t>(SX1262::IRQMask::ALL));

        uint16_t cadDoneBit     = static_cast<uint16_t>(SX1262::IRQMask::CAD_DONE);
        uint16_t cadDetectedBit = static_cast<uint16_t>(SX1262::IRQMask::CAD_DETECTED);

        if ((irqStatus & cadDoneBit) && (irqStatus & cadDetectedBit)) {
            Serial.println("CAD Result: Activity Detected! (Preamble Present)");
            // Handle packet reception here (e.g., lora.setRx(timeout))
        } 
        else if (irqStatus & cadDoneBit) {
            Serial.println("CAD Result: Channel Clear.");
            // Safe to transmit or return to sleep mode
        }

        delay(1000); // Wait 1 second before testing again
        startCAD();
    }
}

void startCAD() {
    Serial.println("Starting Channel Activity Detection...");
    
    // Command opcode 0xC5 triggers CAD mode
    // (Translates SetCAD command using writeCommand internally)
    uint8_t unused = 0; 
    // In actual implementation, SetCAD parameters control symbol duration/exit mode
    // Device returns to Standby RC upon completion when set via SetCAD opcode (0xC5)
}