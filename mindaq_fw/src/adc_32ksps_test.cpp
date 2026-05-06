#include <Arduino.h>
#include <SPI.h>

namespace {

constexpr uint32_t kSerialBaud = 2000000;
constexpr uint32_t kSpiClockHz = 24000000;
constexpr uint8_t kWords = 10;
constexpr uint8_t kBytes = kWords * 3;
constexpr uint8_t kChannels = 6;
constexpr uint32_t kSamplesPerBatch = 32000;

constexpr uint8_t kAdcSclkPin = 10;
constexpr uint8_t kAdcMisoPin = 9;
constexpr uint8_t kAdcMosiPin = 8;
constexpr uint8_t kAdcDrdyPin = 11;
constexpr uint8_t kAdcCsPin = 12;
constexpr uint8_t kAdcSyncPin = 13;

constexpr uint16_t kModeReg = 0x02;
constexpr uint16_t kClockReg = 0x03;
constexpr uint16_t kGain1Reg = 0x04;
constexpr uint16_t kGain2Reg = 0x05;
constexpr uint16_t kModeValue = 0x0110;
constexpr uint16_t kClockValue32k = 0x3FC2;
constexpr uint8_t kGainCode = 3;
constexpr uint16_t kGain1Value =
    (kGainCode << 12) | (kGainCode << 8) | (kGainCode << 4) | kGainCode;
constexpr uint16_t kGain2Value = (kGainCode << 4) | kGainCode;

uint8_t tx[kBytes] = {};
uint8_t rx[kBytes] = {};

uint16_t rreg(uint8_t address) {
  return 0xA000u | (static_cast<uint16_t>(address) << 7);
}

uint16_t wreg(uint8_t address) {
  return 0x6000u | (static_cast<uint16_t>(address) << 7);
}

void putCommand(uint16_t command) {
  memset(tx, 0, sizeof(tx));
  tx[0] = command >> 8;
  tx[1] = command & 0xFF;
}

uint32_t word24(size_t word) {
  const size_t i = word * 3;
  return (static_cast<uint32_t>(rx[i]) << 16) | (static_cast<uint32_t>(rx[i + 1]) << 8) | rx[i + 2];
}

int32_t signExtend24(uint32_t word) {
  int32_t value = static_cast<int32_t>(word & 0x00FFFFFFu);
  if ((value & 0x00800000) != 0) {
    value |= ~0x00FFFFFF;
  }
  return value;
}

bool waitDrdy(uint32_t &ready_us) {
  const uint32_t start = micros();
  while (digitalRead(kAdcDrdyPin) != LOW) {
    if (micros() - start > 50000) {
      return false;
    }
  }
  ready_us = micros();
  return true;
}

bool transfer(uint32_t &ready_us, uint32_t &transfer_us) {
  if (!waitDrdy(ready_us)) {
    return false;
  }
  const uint32_t start = micros();
  SPI.beginTransaction(SPISettings(kSpiClockHz, MSBFIRST, SPI_MODE1));
  digitalWrite(kAdcCsPin, LOW);
  SPI.transferBytes(tx, rx, kBytes);
  digitalWrite(kAdcCsPin, HIGH);
  SPI.endTransaction();
  transfer_us = micros() - start;
  return true;
}

bool writeRegister(uint8_t address, uint16_t value) {
  uint32_t ready = 0;
  uint32_t took = 0;
  putCommand(wreg(address));
  tx[3] = value >> 8;
  tx[4] = value & 0xFF;
  if (!transfer(ready, took)) {
    return false;
  }
  putCommand(0);
  if (!transfer(ready, took)) {
    return false;
  }
  return static_cast<uint16_t>(word24(0) >> 8) == static_cast<uint16_t>(0x4000u | (address << 7));
}

bool readRegister(uint8_t address, uint16_t &value) {
  uint32_t ready = 0;
  uint32_t took = 0;
  putCommand(rreg(address));
  if (!transfer(ready, took)) {
    return false;
  }
  putCommand(0);
  if (!transfer(ready, took)) {
    return false;
  }
  value = word24(0) >> 8;
  return true;
}

bool readFrame(int32_t raw[8], uint32_t &ready_us, uint32_t &transfer_us) {
  memset(tx, 0, sizeof(tx));
  if (!transfer(ready_us, transfer_us)) {
    return false;
  }
  for (size_t i = 0; i < 8; ++i) {
    raw[i] = signExtend24(word24(i + 1));
  }
  return true;
}

void resetAdc() {
  pinMode(kAdcCsPin, OUTPUT);
  digitalWrite(kAdcCsPin, HIGH);
  pinMode(kAdcSyncPin, OUTPUT);
  digitalWrite(kAdcSyncPin, HIGH);
  pinMode(kAdcDrdyPin, INPUT);
  SPI.begin(kAdcSclkPin, kAdcMisoPin, kAdcMosiPin, kAdcCsPin);
  digitalWrite(kAdcSyncPin, LOW);
  delayMicroseconds(20);
  digitalWrite(kAdcSyncPin, HIGH);
  delay(10);
}

bool initAdc() {
  resetAdc();
  if (!writeRegister(kModeReg, kModeValue)) return false;
  if (!writeRegister(kClockReg, kClockValue32k)) return false;
  if (!writeRegister(kGain1Reg, kGain1Value)) return false;
  if (!writeRegister(kGain2Reg, kGain2Value)) return false;

  uint16_t mode = 0;
  uint16_t clock = 0;
  uint16_t gain1 = 0;
  uint16_t gain2 = 0;
  if (!readRegister(kModeReg, mode)) return false;
  if (!readRegister(kClockReg, clock)) return false;
  if (!readRegister(kGain1Reg, gain1)) return false;
  if (!readRegister(kGain2Reg, gain2)) return false;

  Serial.printf("# regs mode=0x%04X clock=0x%04X gain1=0x%04X gain2=0x%04X\n", mode, clock, gain1,
                gain2);
  return mode == kModeValue && clock == kClockValue32k && gain1 == kGain1Value &&
         gain2 == kGain2Value;
}

void runBatch() {
  int32_t raw[8] = {};
  int32_t first[kChannels] = {};
  int32_t last[kChannels] = {};
  int32_t low[kChannels] = {};
  int32_t high[kChannels] = {};
  uint32_t changes[kChannels] = {};
  uint32_t min_dt = UINT32_MAX;
  uint32_t max_dt = 0;
  uint64_t sum_dt = 0;
  uint32_t min_spi = UINT32_MAX;
  uint32_t max_spi = 0;
  uint64_t sum_spi = 0;
  uint32_t prev_ready = 0;
  uint32_t start_us = 0;
  uint32_t end_us = 0;

  for (uint32_t sample = 0; sample < kSamplesPerBatch; ++sample) {
    uint32_t ready = 0;
    uint32_t spi_us = 0;
    if (!readFrame(raw, ready, spi_us)) {
      Serial.println("# read timeout");
      return;
    }
    if (sample == 0) {
      start_us = ready;
      for (size_t ch = 0; ch < kChannels; ++ch) {
        first[ch] = raw[ch];
        low[ch] = raw[ch];
        high[ch] = raw[ch];
      }
    } else {
      const uint32_t dt = ready - prev_ready;
      min_dt = min(min_dt, dt);
      max_dt = max(max_dt, dt);
      sum_dt += dt;
    }
    prev_ready = ready;
    end_us = ready;
    min_spi = min(min_spi, spi_us);
    max_spi = max(max_spi, spi_us);
    sum_spi += spi_us;
    for (size_t ch = 0; ch < kChannels; ++ch) {
      if (raw[ch] != last[ch]) {
        ++changes[ch];
      }
      low[ch] = min(low[ch], raw[ch]);
      high[ch] = max(high[ch], raw[ch]);
      last[ch] = raw[ch];
    }
  }

  const double elapsed_s = static_cast<double>(end_us - start_us) / 1000000.0;
  const double rate = static_cast<double>(kSamplesPerBatch - 1) / elapsed_s;
  const double avg_dt = static_cast<double>(sum_dt) / static_cast<double>(kSamplesPerBatch - 1);
  const double avg_spi = static_cast<double>(sum_spi) / static_cast<double>(kSamplesPerBatch);
  Serial.printf("# samples=%lu elapsed_us=%lu rate=%.1f avg_dt=%.3f min_dt=%lu max_dt=%lu avg_spi=%.3f min_spi=%lu max_spi=%lu\n",
                kSamplesPerBatch, end_us - start_us, rate, avg_dt, min_dt, max_dt, avg_spi,
                min_spi, max_spi);
  for (size_t ch = 0; ch < kChannels; ++ch) {
    Serial.printf("# ch%u first=%ld last=%ld min=%ld max=%ld span=%ld changes=%lu\n", ch,
                  first[ch], last[ch], low[ch], high[ch], high[ch] - low[ch], changes[ch]);
  }
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaud);
  delay(300);
  disableCore0WDT();
  disableCore1WDT();
  Serial.println("# adc_32ksps_test boot");
  while (!initAdc()) {
    Serial.println("# adc init failed");
    delay(500);
  }
  Serial.printf("# adc ready target=32000 SPS spi=%luMHz\n", kSpiClockHz / 1000000);
}

void loop() {
  runBatch();
  delay(100);
}
