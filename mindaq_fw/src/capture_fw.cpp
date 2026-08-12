#include <Arduino.h>
#include <SPI.h>
#include <esp_timer.h>
#include <math.h>

namespace {

constexpr uint32_t kSerialBaud = 2000000;
constexpr uint32_t kSpiClockHz = 24000000;
constexpr uint8_t kWords = 10;
constexpr uint8_t kFrameBytes = kWords * 3;
constexpr uint8_t kChannels = 8;
constexpr uint32_t kSync = 0xA55AA55A;
constexpr size_t kSamplesPerBlock = 64;
constexpr size_t kRingSamples = 8192;
constexpr size_t kRingMask = kRingSamples - 1;
constexpr BaseType_t kAdcCore = 0;
constexpr BaseType_t kSerialCore = ARDUINO_RUNNING_CORE;
constexpr UBaseType_t kAdcPriority = 2;
constexpr UBaseType_t kSerialPriority = 1;
constexpr uint32_t kTaskStackBytes = 8192;
constexpr uint8_t kSettleDiscardFrames = 16;

constexpr uint8_t kAdcSclkPin = 10;
constexpr uint8_t kAdcMisoPin = 9;
constexpr uint8_t kAdcMosiPin = 8;
constexpr uint8_t kAdcDrdyPin = 11;
constexpr uint8_t kAdcCsPin = 12;
constexpr uint8_t kAdcSyncPin = 13;
constexpr uint8_t kTriggerLedPin = 48;

constexpr uint16_t kModeReg = 0x02;
constexpr uint16_t kClockReg = 0x03;
constexpr uint16_t kGain1Reg = 0x04;
constexpr uint16_t kGain2Reg = 0x05;
constexpr uint16_t kModeValue = 0x0110;
constexpr uint16_t kClockValue32k = 0xFFC2;
constexpr uint8_t kDefaultGainCode = 7;
constexpr uint8_t kMaxGainCode = 7;
constexpr int32_t kAdcFullScale = 0x7FFFFF;
constexpr int32_t kAdcWarnCode = static_cast<int32_t>(static_cast<float>(kAdcFullScale) * 0.85f);
constexpr int32_t kAdcClipCode = static_cast<int32_t>(static_cast<float>(kAdcFullScale) * 0.98f);
constexpr uint32_t kTriggerLockoutSamples = 32000;

enum TriggerEdge : uint8_t {
  kTriggerRising = 0,
  kTriggerFalling = 1,
};

struct __attribute__((packed)) Sample {
  uint32_t seq;
  uint8_t raw[kChannels * 3];
  uint8_t gain_code;
  uint8_t warn_flags;
  uint8_t clip_flags;
};

struct __attribute__((packed)) Block {
  uint32_t sync;
  uint32_t seq;
  uint8_t raw[kSamplesPerBlock][kChannels * 3];
  uint8_t gain_code;
  uint8_t warn_flags;
  uint8_t clip_flags;
  uint8_t checksum;
};

struct TriggerConfig {
  bool enabled;
  int8_t pos;
  int8_t neg;
  uint8_t edge;
  uint8_t pin;
  int64_t threshold_uv1000;
  uint32_t delay_us;
  uint32_t width_us;
};

static_assert(sizeof(Block) == 1548);

uint8_t tx[kFrameBytes] = {};
uint8_t rx[kFrameBytes] = {};
Sample ring[kRingSamples] = {};
Block block = {};
volatile uint32_t write_index = 0;
volatile uint32_t read_index = 0;
uint32_t seq = 0;
volatile uint8_t current_gain_code = kDefaultGainCode;
volatile uint8_t requested_gain_code = kDefaultGainCode;
volatile bool output_paused = false;
portMUX_TYPE trigger_lock = portMUX_INITIALIZER_UNLOCKED;
TriggerConfig trigger_config = {};
TriggerConfig adc_trigger = {};
bool adc_trigger_valid = false;
bool trigger_prev_valid = false;
int64_t trigger_prev_value = 0;
uint32_t trigger_last_seq = 0;
bool trigger_last_valid = false;
esp_timer_handle_t trigger_delay_timer = nullptr;
esp_timer_handle_t trigger_width_timer = nullptr;
volatile uint8_t trigger_pin = 0;
volatile uint32_t trigger_config_version = 0;
uint32_t adc_trigger_config_version = 0;

uint16_t rreg(uint8_t address) {
  return 0xA000u | (static_cast<uint16_t>(address) << 7);
}

uint16_t wreg(uint8_t address) {
  return 0x6000u | (static_cast<uint16_t>(address) << 7);
}

uint16_t gain1Value(uint8_t gain_code) {
  return (static_cast<uint16_t>(gain_code) << 12) | (static_cast<uint16_t>(gain_code) << 8) |
         (static_cast<uint16_t>(gain_code) << 4) | gain_code;
}

uint16_t gain2Value(uint8_t gain_code) {
  return (static_cast<uint16_t>(gain_code) << 12) | (static_cast<uint16_t>(gain_code) << 8) |
         (static_cast<uint16_t>(gain_code) << 4) | gain_code;
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

int32_t sampleRaw(const Sample &sample, uint8_t channel) {
  const size_t base = channel * 3;
  uint32_t value = sample.raw[base] | (static_cast<uint32_t>(sample.raw[base + 1]) << 8) |
                   (static_cast<uint32_t>(sample.raw[base + 2]) << 16);
  return signExtend24(value);
}

int64_t rawToUv1000(int32_t raw, uint8_t gain_code) {
  const double uv1000_per_count = (1200000000.0 / static_cast<double>(1u << gain_code)) / 8388608.0;
  return static_cast<int64_t>(llround(static_cast<double>(raw) * uv1000_per_count));
}

int64_t triggerValueUv1000(const Sample &sample, const TriggerConfig &config) {
  int32_t raw = 0;
  if (config.pos >= 0) {
    raw += sampleRaw(sample, static_cast<uint8_t>(config.pos));
  }
  if (config.neg >= 0) {
    raw -= sampleRaw(sample, static_cast<uint8_t>(config.neg));
  }
  return rawToUv1000(raw, sample.gain_code);
}

bool validTriggerPin(int64_t pin) {
  return pin == 43 || pin == 44 || (pin >= 1 && pin <= 7);
}

void stopTriggerOutput() {
  if (trigger_delay_timer != nullptr) {
    esp_timer_stop(trigger_delay_timer);
  }
  if (trigger_width_timer != nullptr) {
    esp_timer_stop(trigger_width_timer);
  }
  const uint8_t pin = trigger_pin;
  if (pin != 0) {
    digitalWrite(pin, LOW);
  }
  digitalWrite(kTriggerLedPin, LOW);
}

void triggerWidthDone(void *arg) {
  (void)arg;
  const uint8_t pin = trigger_pin;
  if (pin != 0) {
    digitalWrite(pin, LOW);
  }
  digitalWrite(kTriggerLedPin, LOW);
}

void triggerDelayDone(void *arg) {
  (void)arg;
  const uint8_t pin = trigger_pin;
  if (pin == 0) {
    return;
  }
  digitalWrite(pin, HIGH);
  if (trigger_width_timer != nullptr) {
    esp_timer_stop(trigger_width_timer);
    esp_timer_start_once(trigger_width_timer, adc_trigger.width_us);
  }
}

void fireTriggerPulse() {
  if (trigger_delay_timer == nullptr || trigger_width_timer == nullptr || adc_trigger.pin == 0) {
    return;
  }
  esp_timer_stop(trigger_delay_timer);
  esp_timer_stop(trigger_width_timer);
  digitalWrite(adc_trigger.pin, LOW);
  digitalWrite(kTriggerLedPin, HIGH);
  trigger_pin = adc_trigger.pin;
  if (adc_trigger.delay_us == 0) {
    triggerDelayDone(nullptr);
  } else {
    esp_timer_start_once(trigger_delay_timer, adc_trigger.delay_us);
  }
}

void resetTriggerState() {
  trigger_prev_valid = false;
  trigger_last_valid = false;
}

void copyTriggerConfig() {
  portENTER_CRITICAL(&trigger_lock);
  const uint32_t version = trigger_config_version;
  if (version != adc_trigger_config_version) {
    adc_trigger = trigger_config;
    adc_trigger_valid = adc_trigger.enabled;
    adc_trigger_config_version = version;
    resetTriggerState();
  }
  portEXIT_CRITICAL(&trigger_lock);
}

void checkTrigger(const Sample &sample) {
  copyTriggerConfig();
  if (!adc_trigger_valid) {
    return;
  }

  const int64_t value = triggerValueUv1000(sample, adc_trigger);
  if (!trigger_prev_valid) {
    trigger_prev_value = value;
    trigger_prev_valid = true;
    return;
  }

  const int64_t threshold = adc_trigger.threshold_uv1000;
  bool crossed = false;
  if (adc_trigger.edge == kTriggerRising) {
    crossed = trigger_prev_value < threshold && value >= threshold;
  } else {
    crossed = trigger_prev_value > threshold && value <= threshold;
  }

  const bool locked_out = trigger_last_valid && sample.seq - trigger_last_seq < kTriggerLockoutSamples;
  if (crossed && !locked_out) {
    fireTriggerPulse();
    trigger_last_seq = sample.seq;
    trigger_last_valid = true;
  }
  trigger_prev_value = value;
}

void initTriggerTimers() {
  const esp_timer_create_args_t delay_args = {
      .callback = &triggerDelayDone,
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "trigger_delay",
      .skip_unhandled_events = true,
  };
  const esp_timer_create_args_t width_args = {
      .callback = &triggerWidthDone,
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "trigger_width",
      .skip_unhandled_events = true,
  };
  esp_timer_create(&delay_args, &trigger_delay_timer);
  esp_timer_create(&width_args, &trigger_width_timer);
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

bool writeGainRegisters(uint8_t gain_code) {
  return writeRegister(kGain1Reg, gain1Value(gain_code)) &&
         writeRegister(kGain2Reg, gain2Value(gain_code));
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

void stopStreamSpi() {
  SPI.endTransaction();
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

uint8_t saturationFlag(int32_t value, int32_t threshold) {
  const int32_t magnitude = value < 0 ? -value : value;
  return magnitude >= threshold ? 1 : 0;
}

void fillSample(Sample &sample, uint32_t sample_seq) {
  sample.seq = sample_seq;
  sample.gain_code = current_gain_code;
  sample.warn_flags = 0;
  sample.clip_flags = 0;
  for (size_t ch = 0; ch < kChannels; ++ch) {
    const uint32_t value = word24(ch + 1);
    sample.raw[ch * 3 + 0] = static_cast<uint8_t>(value & 0xFF);
    sample.raw[ch * 3 + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    sample.raw[ch * 3 + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    const int32_t raw = signExtend24(value);
    sample.warn_flags |= saturationFlag(raw, kAdcWarnCode) << ch;
    sample.clip_flags |= saturationFlag(raw, kAdcClipCode) << ch;
  }
}

bool applyRequestedGain() {
  const uint8_t gain_code = requested_gain_code;
  if (gain_code == current_gain_code) {
    return true;
  }

  output_paused = true;
  vTaskDelay(1);
  stopStreamSpi();
  const bool ok = writeGainRegisters(gain_code);
  startStreamSpi();
  if (ok) {
    current_gain_code = gain_code;
    for (uint8_t i = 0; i < kSettleDiscardFrames; ++i) {
      readStreamFrame();
    }
    write_index = 0;
    read_index = 0;
    seq = 0;
  } else {
    requested_gain_code = current_gain_code;
  }
  output_paused = false;
  resetTriggerState();
  return ok;
}

void setTriggerConfig(const TriggerConfig &config) {
  stopTriggerOutput();
  if (config.enabled) {
    pinMode(config.pin, OUTPUT);
    digitalWrite(config.pin, LOW);
    trigger_pin = config.pin;
  }
  portENTER_CRITICAL(&trigger_lock);
  trigger_config = config;
  ++trigger_config_version;
  portEXIT_CRITICAL(&trigger_lock);
}

void disableTrigger() {
  TriggerConfig config = {};
  setTriggerConfig(config);
}

int64_t nextNumber(char *&text, bool &ok) {
  if (text == nullptr) {
    ok = false;
    return 0;
  }
  char *token = strtok_r(nullptr, " ", &text);
  if (token == nullptr) {
    ok = false;
    return 0;
  }
  char *end = nullptr;
  const int64_t value = strtoll(token, &end, 10);
  if (end == token || *end != '\0') {
    ok = false;
  }
  return value;
}

void handleTriggerCommand(char *command) {
  if (strcmp(command, "trigger off") == 0) {
    disableTrigger();
    return;
  }

  char *save = nullptr;
  strtok_r(command, " ", &save);
  bool ok = true;
  const int64_t pos = nextNumber(save, ok);
  const int64_t neg = nextNumber(save, ok);
  const int64_t edge = nextNumber(save, ok);
  const int64_t pin = nextNumber(save, ok);
  const int64_t threshold = nextNumber(save, ok);
  const int64_t delay_us = nextNumber(save, ok);
  const int64_t width_us = nextNumber(save, ok);
  if (!ok || pos < -1 || pos >= kChannels || neg < -1 || neg >= kChannels ||
      (edge != kTriggerRising && edge != kTriggerFalling) || !validTriggerPin(pin) || delay_us < 0 ||
      delay_us > UINT32_MAX || width_us <= 0 || width_us > UINT32_MAX) {
    return;
  }

  TriggerConfig config = {};
  config.enabled = true;
  config.pos = static_cast<int8_t>(pos);
  config.neg = static_cast<int8_t>(neg);
  config.edge = static_cast<uint8_t>(edge);
  config.pin = static_cast<uint8_t>(pin);
  config.threshold_uv1000 = threshold;
  config.delay_us = static_cast<uint32_t>(delay_us);
  config.width_us = static_cast<uint32_t>(width_us);
  setTriggerConfig(config);
}

void handleCommand(char *command) {
  if (strncmp(command, "trigger ", 8) == 0) {
    handleTriggerCommand(command);
    return;
  }

  if (strncmp(command, "gain ", 5) != 0) {
    return;
  }

  char *end = nullptr;
  const long value = strtol(command + 5, &end, 10);
  if (end == command + 5 || value < 0 || value > kMaxGainCode) {
    return;
  }
  requested_gain_code = static_cast<uint8_t>(value);
}

void pollSerialCommands() {
  static char command[96] = {};
  static uint8_t length = 0;
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      command[length] = '\0';
      if (length > 0) {
        handleCommand(command);
      }
      length = 0;
      continue;
    }
    if (length + 1 < sizeof(command)) {
      command[length++] = c;
    } else {
      length = 0;
    }
  }
}

void adcTask(void *param) {
  (void)param;
  startStreamSpi();
  uint32_t next_sample_us = micros();
  uint16_t sample_phase = 0;
  while (true) {
    if (requested_gain_code != current_gain_code) {
      applyRequestedGain();
      next_sample_us = micros();
      sample_phase = 0;
      continue;
    }

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
    Sample &sample = ring[write_index & kRingMask];
    fillSample(sample, seq);
    checkTrigger(sample);
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
    pollSerialCommands();
    if (output_paused) {
      taskYIELD();
      continue;
    }
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
    block.gain_code = ring[read_index & kRingMask].gain_code;
    block.warn_flags = 0;
    block.clip_flags = 0;
    for (uint32_t i = 0; i < kSamplesPerBlock; ++i) {
      const Sample &sample = ring[(read_index + i) & kRingMask];
      memcpy(block.raw[i], sample.raw, kChannels * 3);
      block.warn_flags |= sample.warn_flags;
      block.clip_flags |= sample.clip_flags;
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
  if (!writeGainRegisters(kDefaultGainCode)) return false;
  current_gain_code = kDefaultGainCode;
  requested_gain_code = kDefaultGainCode;

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
  return mode == kModeValue && clock == kClockValue32k && gain1 == gain1Value(kDefaultGainCode) &&
         gain2 == gain2Value(kDefaultGainCode);
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
  pinMode(kTriggerLedPin, OUTPUT);
  digitalWrite(kTriggerLedPin, LOW);
  initTriggerTimers();
  Serial.println("# mindaq adc stream boot");
  while (!initAdc()) {
    Serial.println("# adc init failed");
    delay(500);
  }
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
