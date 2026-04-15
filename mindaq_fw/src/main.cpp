#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
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

constexpr uint8_t kTftSclkPin = 43;
constexpr uint8_t kTftMisoPin = 44;
constexpr uint8_t kTftMosiPin = 1;
constexpr uint8_t kTftCsPin = 2;
constexpr uint8_t kTftRstPin = 4;
constexpr uint8_t kTftDcPin = 5;
constexpr uint8_t kSdCsPin = 6;
constexpr uint8_t kTftLitPin = 7;

constexpr uint16_t kModeReg = 0x02;
constexpr uint16_t kClockReg = 0x03;
constexpr uint16_t kGain1Reg = 0x04;
constexpr uint16_t kGain2Reg = 0x05;
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
constexpr uint32_t kZeroTimeMs = 1000;
constexpr uint16_t kZeroStepMs = 100;
constexpr uint32_t kDisplayRateHz = 30;
constexpr uint32_t kDisplayPeriodMs = 1000 / kDisplayRateHz;
constexpr BaseType_t kDisplayCore = 0;
constexpr uint32_t kDisplayStackWords = 4096;
constexpr uint32_t kTftSpiHz = 10000000;
constexpr float kDisplayTextStepUv = 1.0f;

constexpr uint16_t kBgColor = ST77XX_BLACK;
constexpr uint16_t kBorderColor = ST77XX_WHITE;
constexpr uint16_t kTrackColor = ST77XX_BLUE;
constexpr uint16_t kPosColor = ST77XX_GREEN;
constexpr uint16_t kNegColor = ST77XX_RED;
constexpr uint16_t kTextColor = ST77XX_WHITE;
constexpr uint16_t kZeroColor = ST77XX_YELLOW;

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

enum class DisplayStatus : uint8_t {
  Boot = 0,
  Retry = 1,
  Zeroing = 2,
  Ready = 3,
};

struct DisplaySnapshot {
  float values[kActiveChannels] = {};
  uint8_t gain_code = 0;
  DisplayStatus status = DisplayStatus::Boot;
  uint16_t zero_progress = 0;
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
    const uint16_t expected =
        static_cast<uint16_t>(0x4000u | (static_cast<uint16_t>(address) << 7));
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
SPIClass tft_spi(HSPI);
Adafruit_ST7789 tft(&tft_spi, kTftCsPin, kTftDcPin, kTftRstPin);

portMUX_TYPE display_lock = portMUX_INITIALIZER_UNLOCKED;
DisplaySnapshot display_snapshot;

bool adc_ready = false;
InputMode input_mode = InputMode::External;
uint8_t gain_code = 7;
char command_buffer[32];
uint8_t command_length = 0;
uint32_t last_init_attempt_ms = 0;
TaskHandle_t display_task_handle = nullptr;
uint32_t zero_start_ms = 0;
uint32_t zero_count = 0;
double zero_sum[kActiveChannels] = {};
float zero_offset[kActiveChannels] = {};
bool zero_done = false;

float codeToUv() {
  const float gain = static_cast<float>(1 << gain_code);
  const float full_scale_volts = 1.2f / gain;
  return (full_scale_volts * 1.0e6f) / 8388608.0f;
}

float displayFullScaleUv(uint8_t code) {
  return 1.2e6f / static_cast<float>(1 << code);
}

void resolveFt(const float gages[kActiveChannels], float ft[kActiveChannels]) {
  const float g0 = gages[0];
  const float g1 = gages[1];
  const float g2 = gages[2];
  const float g3 = gages[3];
  const float g4 = gages[4];
  const float g5 = gages[5];
  ft[0] = 0.5f * (g1 - g3);
  ft[1] = 0.25f * (-g1 - g3 + 2.0f * g5);
  ft[2] = 0.25f * (g0 + g2 + 2.0f * g4);
  ft[3] = 0.5f * (g2 - g0);
  ft[4] = 0.25f * (g0 + g2 - 2.0f * g4);
  ft[5] = 0.25f * (g1 + g3 + 2.0f * g5);
}

void setDisplayStatus(DisplayStatus status, uint16_t zero_progress = 0) {
  portENTER_CRITICAL(&display_lock);
  display_snapshot.status = status;
  display_snapshot.zero_progress = zero_progress;
  display_snapshot.gain_code = gain_code;
  portEXIT_CRITICAL(&display_lock);
}

void publishDisplayValues(const float ft[kActiveChannels], DisplayStatus status, uint16_t zero_progress) {
  portENTER_CRITICAL(&display_lock);
  for (size_t i = 0; i < kActiveChannels; ++i) {
    display_snapshot.values[i] = ft[i];
  }
  display_snapshot.status = status;
  display_snapshot.zero_progress = zero_progress;
  display_snapshot.gain_code = gain_code;
  portEXIT_CRITICAL(&display_lock);
}

void resetZeroing() {
  zero_start_ms = millis();
  zero_count = 0;
  zero_done = false;
  for (size_t i = 0; i < kActiveChannels; ++i) {
    zero_sum[i] = 0.0;
    zero_offset[i] = 0.0f;
  }
  setDisplayStatus(DisplayStatus::Zeroing, 0);
}

void updateDisplayData(const float gages[kActiveChannels]) {
  float ft[kActiveChannels];
  resolveFt(gages, ft);

  if (!zero_done) {
    if (zero_start_ms == 0) {
      resetZeroing();
    }
    for (size_t i = 0; i < kActiveChannels; ++i) {
      zero_sum[i] += ft[i];
    }
    ++zero_count;
    const uint32_t elapsed = millis() - zero_start_ms;
    if (elapsed >= kZeroTimeMs && zero_count > 0) {
      for (size_t i = 0; i < kActiveChannels; ++i) {
        zero_offset[i] = static_cast<float>(zero_sum[i] / static_cast<double>(zero_count));
        ft[i] -= zero_offset[i];
      }
      zero_done = true;
      publishDisplayValues(ft, DisplayStatus::Ready, 1000);
      return;
    }
    const uint16_t progress =
        static_cast<uint16_t>(min<uint32_t>(elapsed, kZeroTimeMs) / kZeroStepMs) * kZeroStepMs;
    setDisplayStatus(DisplayStatus::Zeroing, progress);
    return;
  }

  for (size_t i = 0; i < kActiveChannels; ++i) {
    ft[i] -= zero_offset[i];
  }
  publishDisplayValues(ft, DisplayStatus::Ready, 1000);
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
    case 1:
      code = 0;
      break;
    case 2:
      code = 1;
      break;
    case 4:
      code = 2;
      break;
    case 8:
      code = 3;
      break;
    case 16:
      code = 4;
      break;
    case 32:
      code = 5;
      break;
    case 64:
      code = 6;
      break;
    case 128:
      code = 7;
      break;
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

void drawStatusScreen(const DisplaySnapshot &snapshot) {
  tft.fillScreen(kBgColor);
  tft.setTextColor(kTextColor);
  tft.setTextSize(2);
  tft.setCursor(12, 18);
  switch (snapshot.status) {
    case DisplayStatus::Boot:
      tft.print("Booting...");
      break;
    case DisplayStatus::Retry:
      tft.print("ADC retry");
      break;
    case DisplayStatus::Zeroing:
      tft.print("Zeroing...");
      tft.setTextSize(1);
      tft.setCursor(12, 48);
      tft.print(snapshot.zero_progress / 1000.0f, 2);
      tft.print(" s");
      break;
    case DisplayStatus::Ready:
      break;
  }
  tft.setTextSize(1);
  tft.setCursor(12, 108);
  tft.print("gain x");
  tft.print(1 << snapshot.gain_code);
}

void drawZeroProgress(uint16_t zero_progress) {
  tft.fillRect(12, 48, 72, 10, kBgColor);
  tft.setTextColor(kTextColor);
  tft.setTextSize(1);
  tft.setCursor(12, 48);
  tft.print(zero_progress / 1000.0f, 1);
  tft.print(" s");
}

int16_t barFillForValue(float value, float full_scale) {
  const int16_t track_w = 120 - 8;
  const int16_t half_w = track_w / 2;
  const float clamped = constrain(value, -full_scale, full_scale);
  const int16_t fill =
      static_cast<int16_t>((fabsf(clamped) / full_scale) * static_cast<float>(half_w - 2));
  return clamped >= 0.0f ? fill : -fill;
}

int32_t textValueForDisplay(float value) {
  return static_cast<int32_t>(lroundf(value / kDisplayTextStepUv) * kDisplayTextStepUv);
}

void drawBarCellStatic(int16_t x, int16_t y, int16_t w, int16_t h, const char *name) {
  const int16_t margin = 4;
  const int16_t track_x = x + margin;
  const int16_t track_y = y + 16;
  const int16_t track_w = w - margin * 2;
  const int16_t track_h = 12;
  const int16_t center_x = track_x + track_w / 2;

  tft.drawRect(x, y, w, h, kBorderColor);
  tft.setTextSize(1);
  tft.setTextColor(kTextColor);
  tft.setCursor(x + margin, y + 3);
  tft.print(name);

  tft.drawRect(track_x, track_y, track_w, track_h, kTrackColor);
  tft.drawFastVLine(center_x, track_y + 1, track_h - 2, kZeroColor);
}

void drawBarCellValue(int16_t x, int16_t y, int16_t w, int16_t h, int16_t fill, int32_t text_value) {
  const int16_t margin = 4;
  const int16_t track_x = x + margin;
  const int16_t track_y = y + 16;
  const int16_t track_w = w - margin * 2;
  const int16_t track_h = 12;
  const int16_t center_x = track_x + track_w / 2;

  tft.fillRect(track_x + 1, track_y + 1, track_w - 2, track_h - 2, kBgColor);
  tft.drawFastVLine(center_x, track_y + 1, track_h - 2, kZeroColor);

  if (fill > 0) {
    tft.fillRect(center_x + 1, track_y + 1, fill, track_h - 2, kPosColor);
  } else if (fill < 0) {
    tft.fillRect(center_x + fill, track_y + 1, -fill, track_h - 2, kNegColor);
  }

  tft.fillRect(x + margin, y + 32, w - margin * 2, 10, kBgColor);
  tft.setTextSize(1);
  tft.setTextColor(kTextColor);
  tft.setCursor(x + margin, y + 32);
  tft.print(text_value);
}

void drawBarsScreenFrame(const DisplaySnapshot &snapshot, bool clear_screen) {
  const char *names[kActiveChannels] = {"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"};
  const float full_scale = displayFullScaleUv(snapshot.gain_code);
  if (clear_screen) {
    tft.fillScreen(kBgColor);
    for (size_t i = 0; i < kActiveChannels; ++i) {
      const int16_t col = static_cast<int16_t>(i / 3);
      const int16_t row = static_cast<int16_t>(i % 3);
      drawBarCellStatic(col * 120, row * 45, 120, 45, names[i]);
      drawBarCellValue(col * 120, row * 45, 120, 45, barFillForValue(snapshot.values[i], full_scale),
                       textValueForDisplay(snapshot.values[i]));
    }
  }
}

void displayTask(void *param) {
  (void)param;
  TickType_t last_wake = xTaskGetTickCount();
  DisplayStatus last_status = DisplayStatus::Boot;
  uint16_t last_zero_progress = 0xFFFF;
  bool bars_initialized = false;
  int16_t last_fill[kActiveChannels] = {};
  int32_t last_text[kActiveChannels] = {};
  while (true) {
    DisplaySnapshot snapshot;
    portENTER_CRITICAL(&display_lock);
    snapshot = display_snapshot;
    portEXIT_CRITICAL(&display_lock);

    if (snapshot.status == DisplayStatus::Ready) {
      const bool force_redraw = !bars_initialized || last_status != DisplayStatus::Ready;
      const float full_scale = displayFullScaleUv(snapshot.gain_code);
      if (force_redraw) {
        drawBarsScreenFrame(snapshot, true);
        for (size_t i = 0; i < kActiveChannels; ++i) {
          last_fill[i] = barFillForValue(snapshot.values[i], full_scale);
          last_text[i] = textValueForDisplay(snapshot.values[i]);
        }
      } else {
        for (size_t i = 0; i < kActiveChannels; ++i) {
          const int16_t fill = barFillForValue(snapshot.values[i], full_scale);
          const int32_t text_value = textValueForDisplay(snapshot.values[i]);
          if (fill == last_fill[i] && text_value == last_text[i]) {
            continue;
          }
          const int16_t col = static_cast<int16_t>(i / 3);
          const int16_t row = static_cast<int16_t>(i % 3);
          drawBarCellValue(col * 120, row * 45, 120, 45, fill, text_value);
          last_fill[i] = fill;
          last_text[i] = text_value;
        }
      }
      bars_initialized = true;
    } else {
      if (snapshot.status != last_status) {
        drawStatusScreen(snapshot);
      } else if (snapshot.status == DisplayStatus::Zeroing &&
                 snapshot.zero_progress != last_zero_progress) {
        drawZeroProgress(snapshot.zero_progress);
      }
      bars_initialized = false;
    }

    last_status = snapshot.status;
    last_zero_progress = snapshot.zero_progress;
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kDisplayPeriodMs));
  }
}

void initDisplay() {
  pinMode(kSdCsPin, OUTPUT);
  digitalWrite(kSdCsPin, HIGH);
  pinMode(kTftLitPin, OUTPUT);
  digitalWrite(kTftLitPin, HIGH);

  tft_spi.begin(kTftSclkPin, kTftMisoPin, kTftMosiPin, kTftCsPin);
  tft.init(135, 240);
  tft.setSPISpeed(kTftSpiHz);
  tft.setRotation(1);
  tft.fillScreen(kBgColor);
  drawStatusScreen(display_snapshot);

  xTaskCreatePinnedToCore(displayTask, "display", kDisplayStackWords, nullptr, 1,
                          &display_task_handle, kDisplayCore);
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaud);
  delay(200);
  writeLine("boot");
  setDisplayStatus(DisplayStatus::Boot, 0);
  disableCore0WDT();
  initDisplay();
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
          setDisplayStatus(DisplayStatus::Retry, 0);
          delay(10);
          return;
        }
        resetZeroing();
        writeLine("adc ready");
        Serial.print("# gain=");
        Serial.println(1 << gain_code);
        writeStatusLine("boot");
      } else {
        writeLine("adc retry");
        setDisplayStatus(DisplayStatus::Retry, 0);
      }
    }
    delay(10);
    return;
  }

  AdsFrame frame;
  if (!adc.readFrame(frame)) {
    return;
  }

  TelemetryPacket packet{};
  packet.sync = kPacketSync;
  packet.timestamp_us = micros();
  const float code_to_uv = codeToUv();
  for (size_t i = 0; i < kActiveChannels; ++i) {
    packet.ft_uv[i] = static_cast<float>(frame.raw[i]) * code_to_uv;
  }
  updateDisplayData(packet.ft_uv);

  if (Serial.availableForWrite() < static_cast<int>(sizeof(TelemetryPacket))) {
    return;
  }

  packet.checksum = packetChecksum(packet);
  Serial.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
}
