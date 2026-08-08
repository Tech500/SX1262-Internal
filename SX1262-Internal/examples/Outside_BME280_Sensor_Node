/*
  EoRa-S#-900TB (EoRa Pi) -- Outside BME280 Sensor Node
  Custom SX1262.h Library Implementation (Updated Dec 2024 Rev 2.2 Spec)
*/

#define EoRa_PI_V1
#include "boards.h"

#include <Arduino.h>
#include <WiFi.h>
#include <ESP32_NOW.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <Wire.h>
#include <SPI.h>
#include <BME280I2C.h>
#include "driver/gpio.h"
#include "SX1262.h"

// --- Hardware & Network Pin Definitions ---
#define WAKEUP_PIN GPIO_NUM_16

constexpr uint8_t BME_SDA = 48;
constexpr uint8_t BME_SCL = 47;

#define HUB_WIFI_CHANNEL 11

const float BME280_OUTSIDE_TEMP_CAL_OFFSET_F = +5.54;
uint8_t hubMAC[] = { 0x1C, 0xDB, 0xD4, 0x85, 0x6E, 0x9C };

float radioFreq = 915.0;

// Instantiate custom SX1262 library class matching header:
// SX1262(int nssPin, int resetPin, int busyPin, int dio1Pin)
SX1262 lora(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN, RADIO_DIO1_PIN);

// Tyler Glenn's BME280I2C Settings
BME280I2C::Settings bmeSettings(
  BME280::OSR_X1,
  BME280::OSR_X1,
  BME280::OSR_X1,
  BME280::Mode_Forced,
  BME280::StandbyTime_1000ms,
  BME280::Filter_Off,
  BME280::SpiEnable_False,
  BME280I2C::I2CAddr_0x76);
BME280I2C bme(bmeSettings);

// ─── Message / Packet Structures ─────────────────────────────────────────────
enum MessageType : uint8_t {
  MSG_BME280 = 0,
  MSG_ALERT_FLAG = 1,
  MSG_BLOWER_STATE = 2
};

struct __attribute__((packed)) BlowerData {
  MessageType type;
  bool on;
  float elapsedMinutes;
  float dailyTotalMinutes;
};

struct __attribute__((packed)) BME280Data {
  MessageType type;
  float temperature;
  float humidity;
  float pressure;
};

struct __attribute__((packed)) AlertFlag {
  MessageType type;
  bool alert;
};

struct SensorRegisters {
  float outsideTemp;
  float insideTemp;
  float insideHumidity;
  float thermostat;
  float lastEventMinutes;
  float dailyTotalMinutes;
  float lastRecordedMinute;
  float outsidePressure;
  float insidePressure;
  float pressureDiffHPa;
};
SensorRegisters sensordata;

// --- ESP32 Core v3 ESP-NOW Peer Class ---
class HubPeer : public ESP_NOW_Peer {
public:
  HubPeer(const uint8_t *mac_addr, uint8_t channel)
    : ESP_NOW_Peer(mac_addr, channel, WIFI_IF_STA, NULL) {}

  bool add_to_system() {
    return ESP_NOW_Peer::add();
  }
  bool remove_from_system() {
    return ESP_NOW_Peer::remove();
  }
  bool sendData(const uint8_t *data, size_t len) {
    return send(data, len);
  }
};

// --- Function Prototypes ---
void setupLoRaNode();
bool verifySignalWithBusyLoop(int totalPasses = 10, int requiredHits = 3);
bool readAndSendBME280();
bool sendTelemetryViaESPNOW(float tempF, float humidity, float pressureHPa);
void enterLowPowerWOR();

void setup() {
  initBoard();
  delay(1500);  // SX1262 power stabilization for cold boot

  Serial.begin(115200);
  delay(500);

  Serial.println(F("\n=============================================="));
  Serial.println(F("--- ESP32-S3 WOR NODE (SX1262.h Driver) ---"));

  // 1. Initialize SPI & SX1262 hardware driver
  SPI.begin();
  if (!lora.begin(SPI)) {
    Serial.println(F("[SX1262] Failed to initialize hardware!"));
  }

  // 2. Configure radio parameters for 915 MHz LoRa
  setupLoRaNode();

  // 3. Diagnostic Preamble Check
  verifySignalWithBusyLoop(10, 3);

  readAndSendBME280();
  delay(50);

  Wire.end();
  delay(50);

  Serial.printf("DIO1=%d IRQ=%04X\n",
                digitalRead(RADIO_DIO1_PIN),
                lora.getIrqStatus());

  // 4. Re-arm Duty Cycle WOR & Return to Sleep
  enterLowPowerWOR();
}

void loop() {
  // Unused in deep sleep state
}

void setupLoRaNode() {
  lora.reset();
  lora.setStandby(SX1262_STDBY_RC);
  lora.setRegulatorMode(SX1262_REGULATOR_DC_DC);
  lora.setPacketType(SX1262_PACKET_TYPE_LORA);
  lora.setRfFrequency((uint32_t)(radioFreq * 1000000.0f));
  
  // Set Modulation: SF7, BW 125kHz, CR 4/5, Low Data Rate Optimize disabled (0x00)
  lora.setLoRaModulationParams(SX1262_LORA_SF7, SX1262_LORA_BW_125_0, SX1262_LORA_CR_4_5, 0x00);
  
  // Set Packet Params: 8 symbols preamble, explicit header, 255 max payload, CRC on, standard IQ
  lora.setLoRaPacketParams(8, 0x00, 255, 0x01, 0x00);
}

bool verifySignalWithBusyLoop(int totalPasses, int requiredHits) {
  int hitCount = 0;
  Serial.println(F("\n--- [WAKE DIAGNOSTIC SCAN] ---"));

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  Serial.printf("  Wakeup Cause Code: %d\n", wakeup_reason);

  Serial.print("EXT1 GPIO mask: 0x");
  Serial.println(WAKEUP_PIN, HEX);

  for (int i = 0; i < totalPasses; i++) {
    lora.waitOnBusy();
    uint16_t irqFlags = lora.getIrqStatus();
    int dio1State = digitalRead(WAKEUP_PIN);

    // Check PREAMBLE_DETECTED or HEADER_VALID or pin state
    if ((irqFlags & (SX126X_IRQ_PREAMBLE_DETECTED | SX126X_IRQ_HEADER_VALID)) || dio1State == HIGH) {
      hitCount++;
      Serial.printf("  [Pass %d/%d] Hit! (Total: %d)\n", i + 1, totalPasses, hitCount);
    }

    if (hitCount >= requiredHits) {
      Serial.println(F("  [SUCCESS] Preamble Verified via SX1262.h!"));
      delay(200);
      return true;
    }
    delay(10);
  }
  return false;
}

bool sendTelemetryViaESPNOW(float tempF, float humidity, float pressureHPa) {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_channel(HUB_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (!ESP_NOW.begin()) {
    Serial.println(F("[ESP-NOW] Init failed!"));
    WiFi.mode(WIFI_OFF);
    return false;
  }

  HubPeer localHub(hubMAC, HUB_WIFI_CHANNEL);
  if (!localHub.add_to_system()) {
    Serial.println(F("[ESP-NOW] Failed to bind peer!"));
    ESP_NOW.end();
    WiFi.mode(WIFI_OFF);
    return false;
  }

  BME280Data pkt;
  pkt.type = MSG_BME280;
  pkt.temperature = tempF;
  pkt.humidity = humidity;
  pkt.pressure = pressureHPa;

  bool sent = localHub.sendData((uint8_t *)&pkt, sizeof(BME280Data));
  Serial.printf("[ESP-NOW] Telemetry send to hub: %s\n", sent ? "OK" : "FAILED");

  localHub.remove_from_system();
  ESP_NOW.end();
  WiFi.mode(WIFI_OFF);
  delay(10);
  esp_wifi_stop();

  return sent;
}

bool readAndSendBME280() {
  Wire.end();
  delay(50);
  Wire.setPins(BME_SDA, BME_SCL);
  if (!Wire.begin(BME_SDA, BME_SCL)) {
    Serial.println("Core 3.3.10 failed to allocate I2C peripheral instance!");
  }
  delay(50);

  if (!bme.begin()) {
    Serial.println(F("[BME280] Sensor not found on pins 48/47!"));
    return false;
  }

  float tempF = NAN, humidity = NAN, pressureHPa = NAN;
  BME280::TempUnit tempUnit(BME280::TempUnit_Fahrenheit);
  BME280::PresUnit presUnit(BME280::PresUnit_hPa);

  delay(500);

  bme.read(pressureHPa, tempF, humidity, tempUnit, presUnit);
  tempF += BME280_OUTSIDE_TEMP_CAL_OFFSET_F;

  if (isnan(tempF) || isnan(pressureHPa)) {
    Serial.println(F("[BME280] Telemetry read error."));
    return false;
  }

  Serial.printf("[BME280] Temp: %.2f F | Hum: %.2f %% | Pres: %.4f hPa\n",
                tempF, humidity, pressureHPa);

  return sendTelemetryViaESPNOW(tempF, humidity, pressureHPa);
}

void enterLowPowerWOR() {
  // 1. Clear SX1262 IRQ flags
  lora.clearIrqStatus(0xFFFF);

  // 2. Route PREAMBLE_DETECTED & HEADER_VALID interrupts to DIO1
  uint16_t mask = SX126X_IRQ_PREAMBLE_DETECTED | SX126X_IRQ_HEADER_VALID;
  lora.setDioIrqParams(mask, mask, 0x0000, 0x0000);

  // 3. Issue SetRxDutyCycle opcode (0x94) with 3-byte payload [rxPeriod (3 bytes), sleepPeriod (3 bytes)]
  // rxPeriod = 640 (~400us), sleepPeriod = 32000 (~20ms)
  uint8_t dutyCycleParams[6] = {
    (uint8_t)((640 >> 16) & 0xFF),
    (uint8_t)((640 >> 8) & 0xFF),
    (uint8_t)(640 & 0xFF),
    (uint8_t)((32000 >> 16) & 0xFF),
    (uint8_t)((32000 >> 8) & 0xFF),
    (uint8_t)(32000 & 0xFF)
  };
  lora.writeCommand(SX126X_CMD_SET_RX_DUTY_CYCLE, dutyCycleParams, 6);

  // 4. Set GPIO 16 as standard input
  pinMode(WAKEUP_PIN, INPUT);

  // 5. Enable Ext1 wakeup on HIGH level for GPIO 16 bitmask
  const uint64_t wakeupBitmask = (1ULL << WAKEUP_PIN);
  esp_sleep_enable_ext1_wakeup_io(wakeupBitmask, ESP_EXT1_WAKEUP_ANY_HIGH);

  Serial.printf("DIO1 GPIO %d level before sleep = %d\n", WAKEUP_PIN, digitalRead(WAKEUP_PIN));
  Serial.println(F("=== SX1262 WOR Armed -> Entering Deep Sleep ==="));
  Serial.flush();
  delay(100);

  esp_deep_sleep_start();
}
