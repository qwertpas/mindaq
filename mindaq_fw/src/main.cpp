#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Arduino.h>
#include <SPI.h>

namespace {

constexpr uint32_t kSerialBaud = 2000000;
constexpr uint32_t kSpiClockHz = 24000000;
constexpr uint8_t kWords = 10;
constexpr uint8_t kFrameBytes = kWords * 3;
constexpr uint8_t kChannels = 6;
constexpr uint32_t kSync = 0xA55AA55A;
constexpr size_t kSamplesPerBlock = 64;
constexpr size_t kRingSamples = 8192;
constexpr size_t kRingMask = kRingSamples - 1;
constexpr BaseType_t kAdcCore = 0;
constexpr BaseType_t kSerialCore = ARDUINO_RUNNING_CORE;
constexpr BaseType_t kDisplayCore = ARDUINO_RUNNING_CORE;
constexpr UBaseType_t kAdcPriority = 2;
constexpr UBaseType_t kSerialPriority = 1;
constexpr UBaseType_t kDisplayPriority = 1;
constexpr uint32_t kTaskStackBytes = 8192;
constexpr uint32_t kDisplayStackBytes = 8192;
constexpr uint32_t kDisplayRateHz = 10;
constexpr uint32_t kDisplayPeriodMs = 1000 / kDisplayRateHz;
constexpr uint32_t kTftSpiHz = 10000000;
constexpr uint32_t kZeroTimeMs = 1000;
constexpr uint16_t kZeroStepMs = 100;

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
constexpr uint16_t kModeValue = 0x0110;
constexpr uint16_t kClockValue32k = 0x3FC2;
constexpr uint8_t kGainCode = 3;
constexpr uint16_t kGain1Value =
    (kGainCode << 12) | (kGainCode << 8) | (kGainCode << 4) | kGainCode;
constexpr uint16_t kGain2Value = (kGainCode << 4) | kGainCode;

constexpr uint16_t kBgColor = ST77XX_BLACK;
constexpr uint16_t kBorderColor = ST77XX_WHITE;
constexpr uint16_t kTrackColor = ST77XX_BLUE;
constexpr uint16_t kPosColor = ST77XX_GREEN;
constexpr uint16_t kNegColor = ST77XX_RED;
constexpr uint16_t kTextColor = ST77XX_WHITE;
constexpr uint16_t kZeroColor = ST77XX_YELLOW;

constexpr float kAdcCodeToNetGauge = 0.016f;
constexpr float kNetGaugeZero = 32768.0f;
constexpr float kAdcZeroCode[kChannels] = {
    -389970.6f, -69628.7f, -468145.8f, -62856.2f, -320894.1f, -7240.5f,
};
constexpr float kGaugeOffset[kChannels] = {
    23086.0f, 28152.0f, 25212.0f, 29526.0f, 24516.0f, 33182.0f,
};
constexpr float kGaugeScale[kChannels] = {
    0.934640523f, 0.939244663f, 1.036231884f, 1.041894353f, 1.184265010f,
    0.912280702f,
};
constexpr float kFtMatrix[kChannels][kChannels] = {
    {-2.827122413565440e-05f, 5.179287349565690e-04f, -9.575586391247820e-06f,
     -5.034719323341890e-04f, 6.054530989079390e-06f, 2.075418832354940e-05f},
    {8.796242351000440e-06f, -3.247148267647300e-04f, -1.038902452683050e-07f,
     -2.775572866673270e-04f, -2.520127206219610e-05f, 6.348497973460170e-04f},
    {5.993160239162e-04f, 0.0f, 5.434076606576e-04f, 0.0f, 4.838318054700e-04f,
     0.0f},
    {-3.204207898418680e-06f, -2.084386424212550e-06f, 3.124957530622520e-06f,
     -1.381712094274260e-06f, -2.577052836577890e-07f, 3.830766465429980e-06f},
    {2.229410518476670e-06f, -3.070929538864320e-06f, 1.741821139441800e-06f,
     3.219636663939340e-06f, -3.172020201011310e-06f, -4.085733997308220e-07f},
    {-7.705459896268520e-08f, 2.297459226084220e-06f, 2.397902398001370e-08f,
     2.162107308175090e-06f, -9.943845273683800e-08f, 2.240688262268020e-06f},
};
constexpr float kDisplayFullScale[kChannels] = {
    12.0f, 12.0f, 17.0f, 0.12f, 0.12f, 0.12f,
};

enum class DisplayStatus : uint8_t {
  Boot = 0,
  Retry = 1,
  Zeroing = 2,
  Ready = 3,
};

struct __attribute__((packed)) Sample {
  uint32_t seq;
  uint8_t raw[kChannels * 3];
};

struct __attribute__((packed)) Block {
  uint32_t sync;
  uint32_t seq;
  uint8_t raw[kSamplesPerBlock][kChannels * 3];
  uint8_t checksum;
};

struct DisplaySnapshot {
  float values[kChannels] = {};
  uint8_t gain_code = kGainCode;
  DisplayStatus status = DisplayStatus::Boot;
  uint16_t zero_progress = 0;
};

uint8_t tx[kFrameBytes] = {};
uint8_t rx[kFrameBytes] = {};
Sample ring[kRingSamples] = {};
Block block = {};
volatile uint32_t write_index = 0;
volatile uint32_t read_index = 0;
uint32_t seq = 0;
SPIClass tft_spi(HSPI);
Adafruit_ST7789 tft(&tft_spi, kTftCsPin, kTftDcPin, kTftRstPin);
portMUX_TYPE latest_lock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE display_lock = portMUX_INITIALIZER_UNLOCKED;
int32_t latest_raw[kChannels] = {};
bool latest_ready = false;
DisplaySnapshot display_snapshot;
uint32_t zero_start_ms = 0;
uint32_t zero_count = 0;
double zero_sum[kChannels] = {};
float zero_offset[kChannels] = {};
bool zero_done = false;

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

int32_t signExtend24(uint32_t word) {
  int32_t value = static_cast<int32_t>(word & 0x00FFFFFFu);
  if ((value & 0x00800000) != 0) {
    value |= ~0x00FFFFFF;
  }
  return value;
}

uint32_t rawLe24(const uint8_t *raw) {
  return static_cast<uint32_t>(raw[0]) | (static_cast<uint32_t>(raw[1]) << 8) |
         (static_cast<uint32_t>(raw[2]) << 16);
}

bool waitDrdy() {
  const uint32_t start = micros();
  while (digitalRead(kAdcDrdyPin) == LOW) {
    if (micros() - start > 50000) {
      return false;
    }
  }
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
  if ((sample_seq & 0x1F) == 0) {
    portENTER_CRITICAL(&latest_lock);
    for (size_t ch = 0; ch < kChannels; ++ch) {
      latest_raw[ch] = signExtend24(rawLe24(&sample.raw[ch * 3]));
    }
    latest_ready = true;
    portEXIT_CRITICAL(&latest_lock);
  }
}

void adcTask(void *param) {
  (void)param;
  startStreamSpi();
  uint32_t next_sample_us = micros();
  uint16_t sample_phase = 0;
  while (true) {
    const int32_t wait_us = static_cast<int32_t>(next_sample_us - micros());
    if (wait_us > 0) {
      if (wait_us > 1000) {
        vTaskDelay(1);
      } else {
        taskYIELD();
      }
      continue;
    }

    if (!readStreamFrame()) {
      continue;
    }

    if (write_index - read_index >= kRingSamples) {
      read_index = write_index - kRingSamples + kSamplesPerBlock;
    }
    fillSample(ring[write_index & kRingMask], seq);
    ++write_index;
    ++seq;

    next_sample_us += 31;
    sample_phase += 250;
    if (sample_phase >= 1000) {
      ++next_sample_us;
      sample_phase -= 1000;
    }
    if (static_cast<int32_t>(micros() - next_sample_us) > 100000) {
      next_sample_us = micros();
      sample_phase = 0;
    }
  }
}

void serialTask(void *param) {
  (void)param;
  uint32_t blocks_sent = 0;
  while (true) {
    if (Serial.availableForWrite() < static_cast<int>(sizeof(Block))) {
      if (write_index - read_index > kRingSamples / 2) {
        read_index = write_index - kSamplesPerBlock;
      }
      taskYIELD();
      continue;
    }

    const uint32_t available = write_index - read_index;
    if (available < kSamplesPerBlock) {
      taskYIELD();
      continue;
    }

    block.sync = kSync;
    block.seq = ring[read_index & kRingMask].seq;
    for (uint32_t i = 0; i < kSamplesPerBlock; ++i) {
      memcpy(block.raw[i], ring[(read_index + i) & kRingMask].raw, kChannels * 3);
    }
    block.checksum = blockChecksum(block);
    Serial.write(reinterpret_cast<const uint8_t *>(&block), sizeof(block));
    read_index += kSamplesPerBlock;
    ++blocks_sent;
    if ((blocks_sent & 0x0F) == 0) {
      vTaskDelay(1);
    }
  }
}

void resolveFt(const int32_t raw[kChannels], float ft[kChannels]) {
  float gages[kChannels];
  for (size_t i = 0; i < kChannels; ++i) {
    const float net_gage =
        (static_cast<float>(raw[i]) - kAdcZeroCode[i]) * kAdcCodeToNetGauge + kNetGaugeZero;
    gages[i] = (net_gage - kGaugeOffset[i]) * kGaugeScale[i];
  }
  for (size_t row = 0; row < kChannels; ++row) {
    float value = 0.0f;
    for (size_t col = 0; col < kChannels; ++col) {
      value += kFtMatrix[row][col] * gages[col];
    }
    ft[row] = value;
  }
}

void setDisplayStatus(DisplayStatus status, uint16_t zero_progress = 0) {
  portENTER_CRITICAL(&display_lock);
  display_snapshot.status = status;
  display_snapshot.zero_progress = zero_progress;
  display_snapshot.gain_code = kGainCode;
  portEXIT_CRITICAL(&display_lock);
}

void publishDisplayValues(const float ft[kChannels], DisplayStatus status, uint16_t zero_progress) {
  portENTER_CRITICAL(&display_lock);
  for (size_t i = 0; i < kChannels; ++i) {
    display_snapshot.values[i] = ft[i];
  }
  display_snapshot.status = status;
  display_snapshot.zero_progress = zero_progress;
  display_snapshot.gain_code = kGainCode;
  portEXIT_CRITICAL(&display_lock);
}

void resetZeroing() {
  zero_start_ms = millis();
  zero_count = 0;
  zero_done = false;
  for (size_t i = 0; i < kChannels; ++i) {
    zero_sum[i] = 0.0;
    zero_offset[i] = 0.0f;
  }
  setDisplayStatus(DisplayStatus::Zeroing, 0);
}

void updateDisplayData(const float ft_in[kChannels]) {
  float ft[kChannels];
  for (size_t i = 0; i < kChannels; ++i) {
    ft[i] = ft_in[i];
  }

  if (zero_start_ms != 0) {
    for (size_t i = 0; i < kChannels; ++i) {
      zero_sum[i] += ft[i];
    }
    ++zero_count;
    const uint32_t elapsed = millis() - zero_start_ms;
    if (elapsed >= kZeroTimeMs && zero_count > 0) {
      for (size_t i = 0; i < kChannels; ++i) {
        zero_offset[i] = static_cast<float>(zero_sum[i] / static_cast<double>(zero_count));
        ft[i] -= zero_offset[i];
      }
      zero_done = true;
      zero_start_ms = 0;
      publishDisplayValues(ft, DisplayStatus::Ready, 1000);
      return;
    }
    const uint16_t progress =
        static_cast<uint16_t>(min<uint32_t>(elapsed, kZeroTimeMs) / kZeroStepMs) * kZeroStepMs;
    setDisplayStatus(DisplayStatus::Zeroing, progress);
    return;
  }

  for (size_t i = 0; i < kChannels; ++i) {
    if (zero_done) {
      ft[i] -= zero_offset[i];
    }
  }
  publishDisplayValues(ft, DisplayStatus::Ready, 1000);
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

uint8_t valueDecimals(size_t axis) {
  return axis < 3 ? 2 : 3;
}

void drawBarCellStatic(int16_t x, int16_t y, int16_t w, int16_t h, const char *name,
                       const char *unit) {
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
  tft.print(" ");
  tft.print(unit);
  tft.drawRect(track_x, track_y, track_w, track_h, kTrackColor);
  tft.drawFastVLine(center_x, track_y + 1, track_h - 2, kZeroColor);
}

void drawBarCellValue(int16_t x, int16_t y, int16_t w, int16_t h, int16_t fill, float value,
                      size_t axis) {
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
  tft.print(value, valueDecimals(axis));
}

void drawBarsScreenFrame(const DisplaySnapshot &snapshot) {
  const char *names[kChannels] = {"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"};
  const char *units[kChannels] = {"N", "N", "N", "Nm", "Nm", "Nm"};
  tft.fillScreen(kBgColor);
  for (size_t i = 0; i < kChannels; ++i) {
    const int16_t col = static_cast<int16_t>(i / 3);
    const int16_t row = static_cast<int16_t>(i % 3);
    drawBarCellStatic(col * 120, row * 45, 120, 45, names[i], units[i]);
    drawBarCellValue(col * 120, row * 45, 120, 45,
                     barFillForValue(snapshot.values[i], kDisplayFullScale[i]), snapshot.values[i],
                     i);
  }
}

void displayTask(void *param) {
  (void)param;
  TickType_t last_wake = xTaskGetTickCount();
  DisplayStatus last_status = DisplayStatus::Boot;
  uint16_t last_zero_progress = 0xFFFF;
  bool bars_initialized = false;
  int16_t last_fill[kChannels] = {};
  float last_value[kChannels] = {};

  while (true) {
    int32_t raw[kChannels];
    bool ready = false;
    portENTER_CRITICAL(&latest_lock);
    ready = latest_ready;
    for (size_t i = 0; i < kChannels; ++i) {
      raw[i] = latest_raw[i];
    }
    portEXIT_CRITICAL(&latest_lock);

    if (ready) {
      float ft[kChannels];
      resolveFt(raw, ft);
      updateDisplayData(ft);
    }

    DisplaySnapshot snapshot;
    portENTER_CRITICAL(&display_lock);
    snapshot = display_snapshot;
    portEXIT_CRITICAL(&display_lock);

    if (snapshot.status == DisplayStatus::Ready) {
      if (!bars_initialized || last_status != DisplayStatus::Ready) {
        drawBarsScreenFrame(snapshot);
        for (size_t i = 0; i < kChannels; ++i) {
          last_fill[i] = barFillForValue(snapshot.values[i], kDisplayFullScale[i]);
          last_value[i] = snapshot.values[i];
        }
      } else {
        for (size_t i = 0; i < kChannels; ++i) {
          const int16_t fill = barFillForValue(snapshot.values[i], kDisplayFullScale[i]);
          if (fill == last_fill[i] &&
              fabsf(snapshot.values[i] - last_value[i]) < (i < 3 ? 0.01f : 0.001f)) {
            continue;
          }
          const int16_t col = static_cast<int16_t>(i / 3);
          const int16_t row = static_cast<int16_t>(i % 3);
          drawBarCellValue(col * 120, row * 45, 120, 45, fill, snapshot.values[i], i);
          last_fill[i] = fill;
          last_value[i] = snapshot.values[i];
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
  xTaskCreatePinnedToCore(displayTask, "display", kDisplayStackBytes, nullptr, kDisplayPriority,
                          nullptr, kDisplayCore);
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
  initDisplay();
  disableLoopWDT();
  disableCore0WDT();
  disableCore1WDT();
  Serial.println("# mindaq raw stream boot");
  while (!initAdc()) {
    setDisplayStatus(DisplayStatus::Retry, 0);
    Serial.println("# adc init failed");
    delay(500);
  }
  setDisplayStatus(DisplayStatus::Boot, 0);
  Serial.printf("# stream sync=0x%08lX block_bytes=%u samples_per_block=%u rate=32000 channels=%u\n",
                kSync, static_cast<unsigned>(sizeof(Block)),
                static_cast<unsigned>(kSamplesPerBlock), kChannels);
  delay(1000);
  xTaskCreatePinnedToCore(serialTask, "serial", kTaskStackBytes, nullptr, kSerialPriority, nullptr,
                          kSerialCore);
  xTaskCreatePinnedToCore(adcTask, "adc", kTaskStackBytes, nullptr, kAdcPriority, nullptr, kAdcCore);
}

void loop() {
  delay(1000);
}
