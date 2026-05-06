#include <Arduino.h>
#include <SPI.h>

namespace {

constexpr uint32_t kSerialBaud = 2000000;
constexpr uint32_t kSpiClockHz = 24000000;
constexpr uint8_t kWords = 10;
constexpr uint8_t kFrameBytes = kWords * 3;
constexpr uint8_t kChannels = 6;
constexpr uint32_t kSync = 0xA55AA55A;
constexpr size_t kBatchPackets = 64;
constexpr size_t kRingPackets = 4096;
constexpr size_t kRingMask = kRingPackets - 1;
constexpr BaseType_t kAdcCore = 0;
constexpr BaseType_t kSerialCore = ARDUINO_RUNNING_CORE;
constexpr UBaseType_t kAdcPriority = 2;
constexpr UBaseType_t kSerialPriority = 1;
constexpr uint32_t kTaskStackBytes = 8192;

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

struct __attribute__((packed)) Sample {
  uint32_t seq;
  uint8_t raw[kChannels * 3];
};

struct __attribute__((packed)) Block {
  uint32_t sync;
  uint32_t seq;
  uint8_t raw[kBatchPackets][kChannels * 3];
  uint8_t checksum;
};

uint8_t tx[kFrameBytes] = {};
uint8_t rx[kFrameBytes] = {};
Sample ring[kRingPackets] = {};
Block block = {};
volatile uint32_t write_index = 0;
volatile uint32_t read_index = 0;
uint32_t seq = 0;

uint16_t rreg(uint8_t address) {
  return 0xA000u | (static_cast<uint16_t>(address) << 7);
}

uint16_t wreg(uint8_t address) {
  return 0x6000u | (static_cast<uint16_t>(address) << 7);
}

void putCommand(uint16_t command) {
  memset(tx, 0, sizeof(tx));
  tx[0] = static_cast<uint8_t>(command >> 8);
  tx[1] = static_cast<uint8_t>(command & 0xFF);
}

uint32_t word24(size_t word) {
  const size_t i = word * 3;
  return (static_cast<uint32_t>(rx[i]) << 16) | (static_cast<uint32_t>(rx[i + 1]) << 8) | rx[i + 2];
}

bool waitDrdy() {
  const uint32_t start = micros();
  while (digitalRead(kAdcDrdyPin) != LOW) {
    if (micros() - start > 50000) {
      return false;
    }
  }
  return true;
}

bool transfer() {
  if (!waitDrdy()) {
    return false;
  }
  SPI.beginTransaction(SPISettings(kSpiClockHz, MSBFIRST, SPI_MODE1));
  digitalWrite(kAdcCsPin, LOW);
  SPI.transferBytes(tx, rx, kFrameBytes);
  digitalWrite(kAdcCsPin, HIGH);
  SPI.endTransaction();
  return true;
}

bool writeRegister(uint8_t address, uint16_t value) {
  putCommand(wreg(address));
  tx[3] = static_cast<uint8_t>(value >> 8);
  tx[4] = static_cast<uint8_t>(value & 0xFF);
  if (!transfer()) {
    return false;
  }
  putCommand(0);
  if (!transfer()) {
    return false;
  }
  return static_cast<uint16_t>(word24(0) >> 8) == static_cast<uint16_t>(0x4000u | (address << 7));
}

bool readRegister(uint8_t address, uint16_t &value) {
  putCommand(rreg(address));
  if (!transfer()) {
    return false;
  }
  putCommand(0);
  if (!transfer()) {
    return false;
  }
  value = static_cast<uint16_t>(word24(0) >> 8);
  return true;
}

void startStreamSpi() {
  memset(tx, 0, sizeof(tx));
  SPI.beginTransaction(SPISettings(kSpiClockHz, MSBFIRST, SPI_MODE1));
}

bool readStreamFrame() {
  if (!waitDrdy()) {
    return false;
  }
  digitalWrite(kAdcCsPin, LOW);
  SPI.transferBytes(tx, rx, kFrameBytes);
  digitalWrite(kAdcCsPin, HIGH);
  return true;
}

uint8_t blockChecksum(const Block &block) {
  const auto *bytes = reinterpret_cast<const uint8_t *>(&block);
  uint8_t value = 0;
  for (size_t i = 0; i + 1 < sizeof(block); ++i) {
    value ^= bytes[i];
  }
  return value;
}

void fillSample(Sample &sample, uint32_t sample_seq) {
  sample.seq = sample_seq;
  for (size_t ch = 0; ch < kChannels; ++ch) {
    const uint32_t value = word24(ch + 1);
    sample.raw[ch * 3 + 0] = static_cast<uint8_t>(value & 0xFF);
    sample.raw[ch * 3 + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    sample.raw[ch * 3 + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  }
}

void adcTask(void *param) {
  (void)param;
  startStreamSpi();
  while (true) {
    if (!readStreamFrame()) {
      continue;
    }

    if (write_index - read_index < kRingPackets) {
      fillSample(ring[write_index & kRingMask], seq);
      ++write_index;
    }
    ++seq;
  }
}

void serialTask(void *param) {
  (void)param;
  while (true) {
    if (Serial.availableForWrite() < static_cast<int>(sizeof(Block))) {
      taskYIELD();
      continue;
    }

    const uint32_t available = write_index - read_index;
    if (available < kBatchPackets) {
      taskYIELD();
      continue;
    }

    block.sync = kSync;
    block.seq = ring[read_index & kRingMask].seq;
    for (uint32_t i = 0; i < kBatchPackets; ++i) {
      memcpy(block.raw[i], ring[(read_index + i) & kRingMask].raw, kChannels * 3);
    }
    block.checksum = blockChecksum(block);
    Serial.write(reinterpret_cast<const uint8_t *>(&block), sizeof(block));
    read_index += kBatchPackets;
  }
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

}  // namespace

void setup() {
  Serial.setTxBufferSize(8192);
  Serial.setTxTimeoutMs(0);
  Serial.begin(kSerialBaud);
  delay(300);
  disableLoopWDT();
  disableCore0WDT();
  disableCore1WDT();
  Serial.println("# ft_raw_stream boot");
  while (!initAdc()) {
    Serial.println("# adc init failed");
    delay(500);
  }
  Serial.printf("# stream sync=0x%08lX block_bytes=%u samples_per_block=%u rate=32000 channels=%u\n",
                kSync, static_cast<unsigned>(sizeof(Block)),
                static_cast<unsigned>(kBatchPackets), kChannels);
  delay(3000);
  xTaskCreatePinnedToCore(serialTask, "serial", kTaskStackBytes, nullptr, kSerialPriority, nullptr,
                          kSerialCore);
  xTaskCreatePinnedToCore(adcTask, "adc", kTaskStackBytes, nullptr, kAdcPriority, nullptr, kAdcCore);
}

void loop() {
  delay(1000);
}
