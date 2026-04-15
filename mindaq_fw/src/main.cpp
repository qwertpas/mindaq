#include <Arduino.h>
#include <SPI.h>

namespace {

constexpr uint32_t kSerialBaud = 2000000;
constexpr uint32_t kSpiClockHz = 8000000;
constexpr uint8_t kFrameWords = 10;
constexpr uint8_t kActiveChannels = 6;
constexpr uint16_t kPacketSync = 0xA55A;
constexpr bool kAutoGain = true; // set to true to automatically select gain on start up
constexpr uint8_t kFixedGainCode = 7;

constexpr uint8_t kAdcSclkPin = 10;
constexpr uint8_t kAdcMisoPin = 9;
constexpr uint8_t kAdcMosiPin = 8;
constexpr uint8_t kAdcDrdyPin = 11;
constexpr uint8_t kAdcCsPin = 12;
constexpr uint8_t kAdcSyncPin = 13;

constexpr uint8_t kModeReg = 0x02;
constexpr uint8_t kClockReg = 0x03;
constexpr uint8_t kGain1Reg = 0x04;
constexpr uint8_t kGain2Reg = 0x05;
constexpr uint8_t kChCfgRegs[kActiveChannels] = {0x09, 0x0E, 0x13, 0x18, 0x1D, 0x22};

constexpr uint16_t kModeValue = 0x0110;
constexpr uint16_t kClockValue = 0x3FD2;
constexpr uint16_t kGain1Value = 0x7777;
constexpr uint16_t kGain2Value = 0x0077;

constexpr uint16_t kNullCommand = 0x0000;
constexpr uint32_t kResetPulseUs = 20;
constexpr uint32_t kResetWaitMs = 10;
constexpr uint32_t kDrdyTimeoutUs = 50000;
constexpr uint8_t kStartupDiscardFrames = 8;
constexpr uint8_t kSettleDiscardFrames = 16;

constexpr int32_t kGainLimitCode = 0x700000;

struct __attribute__((packed)) TelemetryPacket {
  uint16_t sync;
  uint32_t timestamp_us;
  float ft_uv[kActiveChannels];
  uint8_t checksum;
};

struct AdsFrame {
  uint16_t status = 0;
  int32_t raw[8] = {};
  uint16_t crc = 0;
};

enum class InputMode : uint8_t {
  External = 0,
  Shorted = 1,
  TestPositive = 2,
  TestNegative = 3,
};

uint8_t packetChecksum(const TelemetryPacket &packet) {
  const auto *bytes = reinterpret_cast<const uint8_t *>(&packet);
  uint8_t checksum = 0;
  for (size_t i = 0; i + 1 < sizeof(packet); ++i) {
    checksum ^= bytes[i];
  }
  return checksum;
}

int32_t signExtend24(uint32_t word) {
  int32_t value = static_cast<int32_t>(word & 0x00FFFFFFu);
  if ((value & 0x00800000) != 0) {
    value |= ~0x00FFFFFF;
  }
  return value;
}

class Ads131m08 {
 public:
  bool begin() {
    pinMode(kAdcCsPin, OUTPUT);
    digitalWrite(kAdcCsPin, HIGH);
    pinMode(kAdcSyncPin, OUTPUT);
    digitalWrite(kAdcSyncPin, HIGH);
    pinMode(kAdcDrdyPin, INPUT);

    SPI.begin(kAdcSclkPin, kAdcMisoPin, kAdcMosiPin, kAdcCsPin);

    hardwareReset();

    if (!writeRegister(kModeReg, kModeValue)) {
      return false;
    }
    if (!writeRegister(kClockReg, kClockValue)) {
      return false;
    }
    if (!writeRegister(kGain1Reg, kGain1Value)) {
      return false;
    }
    if (!writeRegister(kGain2Reg, kGain2Value)) {
      return false;
    }

    uint16_t value = 0;
    if (!readRegister(kModeReg, value) || value != kModeValue) {
      return false;
    }
    if (!readRegister(kClockReg, value) || value != kClockValue) {
      return false;
    }
    if (!readRegister(kGain1Reg, value) || value != kGain1Value) {
      return false;
    }
    if (!readRegister(kGain2Reg, value) || value != kGain2Value) {
      return false;
    }

    AdsFrame frame;
    for (uint8_t i = 0; i < kStartupDiscardFrames; ++i) {
      if (!readFrame(frame)) {
        return false;
      }
    }

    return true;
  }

  bool readFrame(AdsFrame &frame) {
    uint32_t tx[kFrameWords] = {};
    uint32_t rx[kFrameWords] = {};
    if (!transferFrame(tx, rx)) {
      return false;
    }

    frame.status = static_cast<uint16_t>(rx[0] >> 8);
    for (size_t i = 0; i < 8; ++i) {
      frame.raw[i] = signExtend24(rx[i + 1]);
    }
    frame.crc = static_cast<uint16_t>(rx[9] >> 8);
    return true;
  }

  bool setInputMode(InputMode mode) {
    const uint16_t cfg = static_cast<uint16_t>(mode);
    for (uint8_t address : kChCfgRegs) {
      if (!writeRegister(address, cfg)) {
        return false;
      }
    }
    return discardFrames(kSettleDiscardFrames);
  }

  bool setGainCode(uint8_t code) {
    const uint16_t gain1 = static_cast<uint16_t>((code << 12) | (code << 8) | (code << 4) | code);
    const uint16_t gain2 = static_cast<uint16_t>((code << 4) | code);
    if (!writeRegister(kGain1Reg, gain1)) {
      return false;
    }
    if (!writeRegister(kGain2Reg, gain2)) {
      return false;
    }
    return discardFrames(kSettleDiscardFrames);
  }

  bool readStatus(uint16_t &status) {
    AdsFrame frame;
    if (!readFrame(frame)) {
      return false;
    }
    status = frame.status;
    return true;
  }

 private:
  static uint16_t rreg(uint8_t address, uint8_t count_minus_one) {
    return static_cast<uint16_t>(0xA000u | (static_cast<uint16_t>(address) << 7) |
                                 count_minus_one);
  }

  static uint16_t wreg(uint8_t address, uint8_t count_minus_one) {
    return static_cast<uint16_t>(0x6000u | (static_cast<uint16_t>(address) << 7) |
                                 count_minus_one);
  }

  void hardwareReset() {
    digitalWrite(kAdcSyncPin, LOW);
    delayMicroseconds(kResetPulseUs);
    digitalWrite(kAdcSyncPin, HIGH);
    delay(kResetWaitMs);
  }

  bool waitForDrdyLow() {
    const uint32_t start = micros();
    while (digitalRead(kAdcDrdyPin) != LOW) {
      if (micros() - start > kDrdyTimeoutUs) {
        return false;
      }
    }
    return true;
  }

  uint32_t transferWord24(uint32_t word) {
    const uint8_t tx0 = static_cast<uint8_t>((word >> 16) & 0xFFu);
    const uint8_t tx1 = static_cast<uint8_t>((word >> 8) & 0xFFu);
    const uint8_t tx2 = static_cast<uint8_t>(word & 0xFFu);

    const uint8_t rx0 = SPI.transfer(tx0);
    const uint8_t rx1 = SPI.transfer(tx1);
    const uint8_t rx2 = SPI.transfer(tx2);

    return (static_cast<uint32_t>(rx0) << 16) | (static_cast<uint32_t>(rx1) << 8) | rx2;
  }

  bool transferFrame(const uint32_t tx[kFrameWords], uint32_t rx[kFrameWords]) {
    if (!waitForDrdyLow()) {
      return false;
    }

    SPI.beginTransaction(SPISettings(kSpiClockHz, MSBFIRST, SPI_MODE1));
    digitalWrite(kAdcCsPin, LOW);
    for (size_t i = 0; i < kFrameWords; ++i) {
      rx[i] = transferWord24(tx[i]);
    }
    digitalWrite(kAdcCsPin, HIGH);
    SPI.endTransaction();
    return true;
  }

  bool writeRegister(uint8_t address, uint16_t value) {
    uint32_t tx[kFrameWords] = {};
    uint32_t rx[kFrameWords] = {};
    tx[0] = static_cast<uint32_t>(wreg(address, 0)) << 8;
    tx[1] = static_cast<uint32_t>(value) << 8;
    if (!transferFrame(tx, rx)) {
      return false;
    }

    uint32_t null_tx[kFrameWords] = {};
    uint32_t null_rx[kFrameWords] = {};
    null_tx[0] = static_cast<uint32_t>(kNullCommand) << 8;
    if (!transferFrame(null_tx, null_rx)) {
      return false;
    }

    const uint16_t ack = static_cast<uint16_t>(null_rx[0] >> 8);
    const uint16_t expected = static_cast<uint16_t>(0x4000u | (static_cast<uint16_t>(address) << 7));
    return ack == expected;
  }

  bool readRegister(uint8_t address, uint16_t &value) {
    uint32_t tx[kFrameWords] = {};
    uint32_t rx[kFrameWords] = {};
    tx[0] = static_cast<uint32_t>(rreg(address, 0)) << 8;
    if (!transferFrame(tx, rx)) {
      return false;
    }

    uint32_t null_tx[kFrameWords] = {};
    uint32_t null_rx[kFrameWords] = {};
    null_tx[0] = static_cast<uint32_t>(kNullCommand) << 8;
    if (!transferFrame(null_tx, null_rx)) {
      return false;
    }

    value = static_cast<uint16_t>(null_rx[0] >> 8);
    return true;
  }

  bool discardFrames(uint8_t count) {
    AdsFrame frame;
    for (uint8_t i = 0; i < count; ++i) {
      if (!readFrame(frame)) {
        return false;
      }
    }
    return true;
  }
};

Ads131m08 adc;
bool adc_ready = false;
InputMode input_mode = InputMode::External;
uint8_t gain_code = 7;
char command_buffer[32];
uint8_t command_length = 0;
uint32_t last_init_attempt_ms = 0;

float codeToUv() {
  const float gain = static_cast<float>(1 << gain_code);
  const float full_scale_volts = 1.2f / gain;
  return (full_scale_volts * 1.0e6f) / 8388608.0f;
}

void discardLine() {
  command_length = 0;
}

void writeLine(const char *text) {
  Serial.print("# ");
  Serial.println(text);
}

void writeStatusLine(const char *label) {
  uint16_t status = 0;
  if (!adc.readStatus(status)) {
    writeLine("status read failed");
    return;
  }
  Serial.print("# ");
  Serial.print(label);
  Serial.print(" status=0x");
  Serial.println(status, HEX);
}

bool setInputModeFromCommand(InputMode mode, const char *name) {
  if (!adc.setInputMode(mode)) {
    writeLine("input mode change failed");
    return false;
  }
  input_mode = mode;
  Serial.print("# mode=");
  Serial.println(name);
  writeStatusLine("mode");
  return true;
}

bool setGainFromValue(uint16_t gain_value) {
  uint8_t code = 0;
  switch (gain_value) {
    case 1: code = 0; break;
    case 2: code = 1; break;
    case 4: code = 2; break;
    case 8: code = 3; break;
    case 16: code = 4; break;
    case 32: code = 5; break;
    case 64: code = 6; break;
    case 128: code = 7; break;
    default:
      writeLine("bad gain");
      return false;
  }
  if (!adc.setGainCode(code)) {
    writeLine("gain change failed");
    return false;
  }
  gain_code = code;
  Serial.print("# gain=");
  Serial.println(gain_value);
  writeStatusLine("gain");
  return true;
}

bool gainClipped(const AdsFrame &frame) {
  for (size_t i = 0; i < kActiveChannels; ++i) {
    if (abs(frame.raw[i]) >= kGainLimitCode) {
      return true;
    }
  }
  return false;
}

bool chooseGain() {
  if (!adc.setInputMode(InputMode::External)) {
    return false;
  }
  input_mode = InputMode::External;
  if (!kAutoGain) {
    if (!adc.setGainCode(kFixedGainCode)) {
      return false;
    }
    gain_code = kFixedGainCode;
    return true;
  }
  for (int8_t code = 7; code >= 0; --code) {
    if (!adc.setGainCode(static_cast<uint8_t>(code))) {
      return false;
    }
    bool clipped = false;
    for (uint8_t i = 0; i < 8; ++i) {
      AdsFrame frame;
      if (!adc.readFrame(frame)) {
        return false;
      }
      if (gainClipped(frame)) {
        clipped = true;
      }
    }
    if (!clipped) {
      gain_code = static_cast<uint8_t>(code);
      return true;
    }
  }
  gain_code = 0;
  return true;
}

void handleCommand(const char *command) {
  if (strcmp(command, "ext") == 0) {
    setInputModeFromCommand(InputMode::External, "ext");
    return;
  }
  if (strcmp(command, "short") == 0) {
    setInputModeFromCommand(InputMode::Shorted, "short");
    return;
  }
  if (strcmp(command, "testp") == 0) {
    setInputModeFromCommand(InputMode::TestPositive, "testp");
    return;
  }
  if (strcmp(command, "testn") == 0) {
    setInputModeFromCommand(InputMode::TestNegative, "testn");
    return;
  }
  if (strncmp(command, "gain ", 5) == 0) {
    setGainFromValue(static_cast<uint16_t>(atoi(command + 5)));
    return;
  }
  if (strcmp(command, "status") == 0) {
    writeStatusLine("status");
    return;
  }
  writeLine("unknown command");
}

void readCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (command_length > 0) {
        command_buffer[command_length] = '\0';
        handleCommand(command_buffer);
        discardLine();
      }
      continue;
    }
    if (command_length + 1 < sizeof(command_buffer)) {
      command_buffer[command_length++] = c;
    } else {
      discardLine();
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaud);
  delay(200);
  writeLine("boot");
}

void loop() {
  readCommands();
  if (!adc_ready) {
    if (millis() - last_init_attempt_ms >= 500) {
      last_init_attempt_ms = millis();
      adc_ready = adc.begin();
      if (adc_ready) {
        if (!chooseGain()) {
          adc_ready = false;
          writeLine("gain select failed");
          delay(10);
          return;
        }
        writeLine("adc ready");
        Serial.print("# gain=");
        Serial.println(1 << gain_code);
        writeStatusLine("boot");
      } else {
        writeLine("adc retry");
      }
    }
    delay(10);
    return;
  }

  AdsFrame frame;
  if (!adc.readFrame(frame)) {
    return;
  }

  if (Serial.availableForWrite() < static_cast<int>(sizeof(TelemetryPacket))) {
    return;
  }

  TelemetryPacket packet{};
  packet.sync = kPacketSync;
  packet.timestamp_us = micros();
  const float code_to_uv = codeToUv();
  for (size_t i = 0; i < kActiveChannels; ++i) {
    packet.ft_uv[i] = static_cast<float>(frame.raw[i]) * code_to_uv;
  }
  packet.checksum = packetChecksum(packet);
  Serial.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
}
