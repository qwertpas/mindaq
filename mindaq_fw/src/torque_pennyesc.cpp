#include <Arduino.h>
#include <SPI.h>
#include <pennyesc_arduino.h>

namespace {

constexpr uint32_t kSerialBaud = 2000000;
constexpr uint32_t kPennyBaud = PENNYESC_BAUD_FAST;
constexpr uint32_t kSpiClockHz = 24000000;
constexpr uint8_t kWords = 10;
constexpr uint8_t kFrameBytes = kWords * 3;
constexpr uint8_t kChannels = 6;
constexpr uint16_t kSync = 0xA55A;
constexpr uint32_t kLoopPeriodUs = 1000;
constexpr uint8_t kAdcAverageSamples = 32;
constexpr uint32_t kPennyStatusPeriodUs = 20000;
constexpr uint32_t kPennyReadTimeoutMs = 10;
constexpr uint32_t kPennyScanTimeoutMs = 120;
constexpr uint32_t kPennyControlTimeoutMs = 120;
constexpr uint32_t kPennyPosVelPeriodUs = 500;
constexpr uint32_t kPennyPosVelTimeoutUs = 1500;
constexpr int16_t kMaxDuty = 799;
constexpr uint8_t kStartMotorId = 1;
constexpr BaseType_t kAdcCore = 0;
constexpr BaseType_t kMotorCore = ARDUINO_RUNNING_CORE;
constexpr UBaseType_t kAdcPriority = 2;
constexpr UBaseType_t kMotorPriority = 1;
constexpr uint32_t kTaskStackBytes = 8192;

constexpr uint8_t kAdcSclkPin = 10;
constexpr uint8_t kAdcMisoPin = 9;
constexpr uint8_t kAdcMosiPin = 8;
constexpr uint8_t kAdcDrdyPin = 11;
constexpr uint8_t kAdcCsPin = 12;
constexpr uint8_t kAdcSyncPin = 13;

constexpr uint8_t kMotorRxPin = 43;
constexpr uint8_t kMotorTxPin = 44;
constexpr uint8_t kMotorGndPin = 3;

constexpr uint16_t kModeReg = 0x02;
constexpr uint16_t kClockReg = 0x03;
constexpr uint16_t kGain1Reg = 0x04;
constexpr uint16_t kGain2Reg = 0x05;
constexpr uint16_t kModeValue = 0x0110;
constexpr uint16_t kClockValue32k = 0x3FC2;
constexpr uint8_t kDefaultGainCode = 3;
constexpr uint8_t kMaxGainCode = 7;
constexpr int32_t kAdcFullScale = 0x7FFFFF;
constexpr int32_t kAdcWarnCode = static_cast<int32_t>(static_cast<float>(kAdcFullScale) * 0.85f);
constexpr int32_t kAdcClipCode = static_cast<int32_t>(static_cast<float>(kAdcFullScale) * 0.98f);
constexpr uint8_t kSettleDiscardFrames = 16;

static_assert(kPennyBaud == 921600u, "PennyESC fast UART baud changed");

constexpr float countToMicrovolt(uint8_t gain_code) {
  return (1.2e6f / static_cast<float>(1u << gain_code)) / 8388608.0f;
}

constexpr float kRawZeroCode[kChannels] = {
    -995095.6f, -358128.7f, -940395.8f, -265481.2f, -836644.1f, 18634.5f,
};
constexpr float kRawZeroMicrovolt[kChannels] = {
    kRawZeroCode[0] * countToMicrovolt(kDefaultGainCode),
    kRawZeroCode[1] * countToMicrovolt(kDefaultGainCode),
    kRawZeroCode[2] * countToMicrovolt(kDefaultGainCode),
    kRawZeroCode[3] * countToMicrovolt(kDefaultGainCode),
    kRawZeroCode[4] * countToMicrovolt(kDefaultGainCode),
    kRawZeroCode[5] * countToMicrovolt(kDefaultGainCode),
};

constexpr float kAdcUvToAtiGauge[kChannels][kChannels] = {
    {0.0f, 0.0f, 0.0f, 0.0f, 1.059662393280649e+00f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 8.162949541379004e-01f},
    {0.0f, 0.0f, 9.272045943442637e-01f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 9.322712858379332e-01f, 0.0f, 0.0f},
    {8.363021832919449e-01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 8.404218980265711e-01f, 0.0f, 0.0f, 0.0f, 0.0f},
};
constexpr float kAtiGaugeToTorqueZ[kChannels] = {
    -9.943845273683800e-08f, 2.240688262268020e-06f, 2.397902398001370e-08f,
    2.162107308175090e-06f,  -7.705459896268520e-08f, 2.297459226084220e-06f,
};

struct __attribute__((packed)) TelemetryPacket {
  uint16_t sync;
  uint32_t seq;
  uint32_t timestamp_us;
  float torque_z_nm;
  float motor_speed_rad_s;
  int16_t set_duty;
  uint8_t motor_id;
  uint16_t scan_mask;
  int16_t penny_clip;
  uint16_t penny_mct_fault_count;
  uint16_t penny_isr_us;
  uint16_t penny_isr_max_us;
  uint16_t penny_i2c_us;
  uint32_t penny_isr_overrun_count;
  uint32_t penny_i2c_timeout_count;
  uint32_t penny_i2c_nack_count;
  uint32_t penny_i2c_recover_count;
  uint32_t penny_uart_overrun_errors;
  uint32_t penny_tmag_sample_count;
  uint16_t penny_tmag_sample_dt_us;
  uint8_t gain_code;
  uint8_t warn_flags;
  uint8_t clip_flags;
  uint8_t checksum;
};

static_assert(sizeof(TelemetryPacket) == 63, "TelemetryPacket must match GUI struct");

uint8_t tx[kFrameBytes] = {};
uint8_t rx[kFrameBytes] = {};
volatile uint8_t current_gain_code = kDefaultGainCode;
volatile uint8_t requested_gain_code = kDefaultGainCode;
portMUX_TYPE latest_lock = portMUX_INITIALIZER_UNLOCKED;
int32_t latest_raw[kChannels] = {};
uint8_t latest_gain_code = kDefaultGainCode;
uint8_t latest_warn_flags = 0;
uint8_t latest_clip_flags = 0;
bool latest_ready = false;

PennyEsc esc(kStartMotorId);
PennyEscEncoderData esc_data;
PennyEscStatus penny_status;
uint32_t telemetry_seq = 0;
uint32_t next_loop_us = 0;
uint32_t next_penny_status_us = 0;
uint32_t next_penny_posvel_us = 0;
uint32_t penny_posvel_request_us = 0;
uint16_t scan_mask = 0;
float motor_speed_rad_s = 0.0f;
volatile int16_t target_duty = 0;
int16_t applied_duty = 0;
volatile int16_t requested_clip = 150;
volatile int8_t requested_id = -1;
int16_t penny_clip = 150;
uint8_t motor_id = kStartMotorId;
volatile bool scan_requested = false;
volatile bool clip_requested = false;
bool penny_posvel_pending = false;
char command_buf[32] = {};
uint8_t command_len = 0;

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
  return (static_cast<uint16_t>(gain_code) << 4) | gain_code;
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

uint8_t saturationFlag(int32_t value, int32_t threshold) {
  const int32_t magnitude = value < 0 ? -value : value;
  return magnitude >= threshold ? 1 : 0;
}

void readAdcRaw(int32_t raw[kChannels], uint8_t &warn_flags, uint8_t &clip_flags) {
  warn_flags = 0;
  clip_flags = 0;
  for (size_t ch = 0; ch < kChannels; ++ch) {
    raw[ch] = signExtend24(word24(ch + 1));
    warn_flags |= saturationFlag(raw[ch], kAdcWarnCode) << ch;
    clip_flags |= saturationFlag(raw[ch], kAdcClipCode) << ch;
  }
}

void publishAdcFrame(const int32_t raw[kChannels], uint8_t warn_flags, uint8_t clip_flags) {
  portENTER_CRITICAL(&latest_lock);
  for (size_t ch = 0; ch < kChannels; ++ch) {
    latest_raw[ch] = raw[ch];
  }
  latest_gain_code = current_gain_code;
  latest_warn_flags = warn_flags;
  latest_clip_flags = clip_flags;
  latest_ready = true;
  portEXIT_CRITICAL(&latest_lock);
}

void publishAveragedAdcFrame(const int64_t sum[kChannels], uint8_t warn_flags, uint8_t clip_flags) {
  int32_t raw[kChannels] = {};
  for (size_t ch = 0; ch < kChannels; ++ch) {
    raw[ch] = static_cast<int32_t>(sum[ch] / kAdcAverageSamples);
  }
  publishAdcFrame(raw, warn_flags, clip_flags);
}

void clearAdcAccumulator(int64_t sum[kChannels], uint8_t &count, uint8_t &warn_flags, uint8_t &clip_flags) {
  for (size_t ch = 0; ch < kChannels; ++ch) {
    sum[ch] = 0;
  }
  count = 0;
  warn_flags = 0;
  clip_flags = 0;
}

void addAdcFrame(int64_t sum[kChannels], uint8_t &count, uint8_t &warn_flags, uint8_t &clip_flags) {
  int32_t raw[kChannels] = {};
  uint8_t frame_warn_flags = 0;
  uint8_t frame_clip_flags = 0;
  readAdcRaw(raw, frame_warn_flags, frame_clip_flags);
  for (size_t ch = 0; ch < kChannels; ++ch) {
    sum[ch] += raw[ch];
  }
  ++count;
  warn_flags |= frame_warn_flags;
  clip_flags |= frame_clip_flags;
  if (count >= kAdcAverageSamples) {
    publishAveragedAdcFrame(sum, warn_flags, clip_flags);
    clearAdcAccumulator(sum, count, warn_flags, clip_flags);
  }
}

bool applyRequestedGain() {
  const uint8_t gain_code = requested_gain_code;
  if (gain_code == current_gain_code) {
    return true;
  }
  stopStreamSpi();
  const bool ok = writeGainRegisters(gain_code);
  startStreamSpi();
  if (ok) {
    current_gain_code = gain_code;
    for (uint8_t i = 0; i < kSettleDiscardFrames; ++i) {
      readStreamFrame();
    }
    portENTER_CRITICAL(&latest_lock);
    latest_ready = false;
    latest_gain_code = gain_code;
    latest_warn_flags = 0;
    latest_clip_flags = 0;
    portEXIT_CRITICAL(&latest_lock);
  } else {
    requested_gain_code = current_gain_code;
  }
  return ok;
}

void adcTask(void *param) {
  (void)param;
  startStreamSpi();
  uint32_t next_sample_us = micros();
  uint16_t sample_phase = 0;
  int64_t adc_sum[kChannels] = {};
  uint8_t adc_count = 0;
  uint8_t adc_warn_flags = 0;
  uint8_t adc_clip_flags = 0;
  while (true) {
    if (requested_gain_code != current_gain_code) {
      applyRequestedGain();
      clearAdcAccumulator(adc_sum, adc_count, adc_warn_flags, adc_clip_flags);
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

    if (readStreamFrame()) {
      addAdcFrame(adc_sum, adc_count, adc_warn_flags, adc_clip_flags);
    }

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

float resolveTorqueZ(const int32_t raw[kChannels], uint8_t gain_code) {
  float adc_uv[kChannels];
  for (size_t i = 0; i < kChannels; ++i) {
    adc_uv[i] = static_cast<float>(raw[i]) * countToMicrovolt(gain_code) - kRawZeroMicrovolt[i];
  }

  float ati_gage[kChannels];
  for (size_t row = 0; row < kChannels; ++row) {
    float value = 0.0f;
    for (size_t col = 0; col < kChannels; ++col) {
      value += kAdcUvToAtiGauge[row][col] * adc_uv[col];
    }
    ati_gage[row] = value;
  }

  float torque_z = 0.0f;
  for (size_t col = 0; col < kChannels; ++col) {
    torque_z += kAtiGaugeToTorqueZ[col] * ati_gage[col];
  }
  return torque_z;
}

uint8_t packetChecksum(const TelemetryPacket &packet) {
  const auto *bytes = reinterpret_cast<const uint8_t *>(&packet);
  uint8_t value = 0;
  for (size_t i = 0; i + 1 < sizeof(packet); ++i) {
    value ^= bytes[i];
  }
  return value;
}

void setPennyClip(int16_t clip) {
  clip = static_cast<int16_t>(constrain(static_cast<long>(clip), 0L, static_cast<long>(kMaxDuty)));
  penny_posvel_pending = false;
  esc.clearRx();
  if (esc.setControl(0.0f, 0.0f, 0.0f, applied_duty, clip, nullptr, kPennyControlTimeoutMs)) {
    penny_clip = clip;
  }
}

void stopMotor() {
  target_duty = 0;
  if (esc.sendDuty(0)) {
    applied_duty = 0;
  }
}

void setPennyId(uint8_t id) {
  stopMotor();
  penny_posvel_pending = false;
  esc.clearRx();
  motor_id = static_cast<uint8_t>(id & 0x0F);
  esc.setAddress(motor_id);
  penny_status = PennyEscStatus{};
  esc_data = PennyEscEncoderData{};
  motor_speed_rad_s = 0.0f;
  next_penny_status_us = 0;
  next_penny_posvel_us = 0;
  penny_posvel_pending = false;
  esc.sendDuty(0);
  setPennyClip(penny_clip);
}

bool scanPennyId(uint8_t id) {
  penny_posvel_pending = false;
  esc.clearRx();
  esc.setAddress(id);
  PennyEscStatus status;
  return esc.getStatus(status, kPennyScanTimeoutMs) && status.valid;
}

void scanMotorIds() {
  stopMotor();
  scan_mask = 0;
  for (uint8_t id = 0; id < 16; ++id) {
    if (scanPennyId(id)) {
      scan_mask |= static_cast<uint16_t>(1u << id);
      if (scan_mask == static_cast<uint16_t>(1u << id)) {
        setPennyId(id);
      }
    }
  }
  esc.setAddress(motor_id);
}

void handleCommand(const char *cmd) {
  if (cmd[0] == 'S' || cmd[0] == 's') {
    scan_requested = true;
    return;
  }

  if (cmd[0] == 'I' || cmd[0] == 'i') {
    char *end = nullptr;
    const long id = strtol(cmd + 1, &end, 10);
    if (end != cmd + 1) {
      requested_id = static_cast<int8_t>(constrain(id, 0L, 15L));
    }
    return;
  }

  if (cmd[0] == 'D' || cmd[0] == 'd') {
    char *end = nullptr;
    const long value = strtol(cmd + 1, &end, 10);
    if (end != cmd + 1) {
      target_duty = static_cast<int16_t>(
          constrain(value, static_cast<long>(-kMaxDuty), static_cast<long>(kMaxDuty)));
    }
    return;
  }

  if (cmd[0] == 'C' || cmd[0] == 'c') {
    char *end = nullptr;
    const long value = strtol(cmd + 1, &end, 10);
    if (end != cmd + 1) {
      requested_clip =
          static_cast<int16_t>(constrain(value, 0L, static_cast<long>(kMaxDuty)));
      clip_requested = true;
    }
    return;
  }

  if (strncmp(cmd, "gain ", 5) == 0) {
    char *end = nullptr;
    const long value = strtol(cmd + 5, &end, 10);
    if (end != cmd + 5 && value >= 0 && value <= kMaxGainCode) {
      requested_gain_code = static_cast<uint8_t>(value);
    }
  }
}

void readUsbCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (command_len > 0) {
        command_buf[command_len] = '\0';
        handleCommand(command_buf);
        command_len = 0;
      }
      continue;
    }
    if (command_len + 1 < sizeof(command_buf)) {
      command_buf[command_len++] = c;
    } else {
      command_len = 0;
    }
  }
}

void updateMotor() {
  if (target_duty != applied_duty) {
    if (esc.sendDuty(target_duty)) {
      applied_duty = target_duty;
    }
  }

  const uint32_t now = micros();

  if (penny_posvel_pending) {
    if (esc.readPosVelAvailable(esc_data) && esc_data.valid) {
      motor_speed_rad_s = esc_data.velocityRadS();
      penny_posvel_pending = false;
      next_penny_posvel_us = now + kPennyPosVelPeriodUs;
    } else if (static_cast<uint32_t>(now - penny_posvel_request_us) > kPennyPosVelTimeoutUs) {
      penny_posvel_pending = false;
      next_penny_posvel_us = now + kPennyPosVelPeriodUs;
    }
  }

  if (!penny_posvel_pending && static_cast<int32_t>(now - next_penny_status_us) >= 0) {
    next_penny_status_us = now + kPennyStatusPeriodUs;
    esc.clearRx();
    if (esc.getStatus(penny_status, kPennyReadTimeoutMs) && penny_status.valid) {
      motor_speed_rad_s = penny_status.velocityRadS();
    }
  }

  if (!penny_posvel_pending && static_cast<int32_t>(now - next_penny_posvel_us) >= 0) {
    if (esc.requestPosVel()) {
      esc.serial().flush();
      penny_posvel_pending = true;
      penny_posvel_request_us = now;
    }
  }
}

void updateMotorFast() {
  if (penny_posvel_pending && esc.readPosVelAvailable(esc_data) && esc_data.valid) {
    motor_speed_rad_s = esc_data.velocityRadS();
    penny_posvel_pending = false;
    next_penny_posvel_us = micros() + kPennyPosVelPeriodUs;
  }
}

void motorTask(void *param) {
  (void)param;
  TickType_t last_wake = xTaskGetTickCount();
  while (true) {
    if (scan_requested) {
      scan_requested = false;
      scanMotorIds();
    }

    if (requested_id >= 0) {
      const uint8_t id = static_cast<uint8_t>(requested_id);
      requested_id = -1;
      setPennyId(id);
    }

    if (clip_requested) {
      const int16_t clip = requested_clip;
      clip_requested = false;
      setPennyClip(clip);
    }

    updateMotor();
    for (uint8_t i = 0; i < 20; ++i) {
      updateMotorFast();
      delayMicroseconds(50);
    }
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
  }
}

void sendTelemetry(uint32_t timestamp_us) {
  int32_t raw[kChannels] = {};
  uint8_t gain_code = current_gain_code;
  uint8_t warn_flags = 0;
  uint8_t clip_flags = 0;
  bool ready = false;
  portENTER_CRITICAL(&latest_lock);
  ready = latest_ready;
  for (size_t i = 0; i < kChannels; ++i) {
    raw[i] = latest_raw[i];
  }
  gain_code = latest_gain_code;
  warn_flags = latest_warn_flags;
  clip_flags = latest_clip_flags;
  portEXIT_CRITICAL(&latest_lock);

  TelemetryPacket packet{};
  packet.sync = kSync;
  packet.seq = telemetry_seq++;
  packet.timestamp_us = timestamp_us;
  packet.torque_z_nm = ready ? resolveTorqueZ(raw, gain_code) : NAN;
  packet.motor_speed_rad_s = motor_speed_rad_s;
  packet.set_duty = applied_duty;
  packet.motor_id = motor_id;
  packet.scan_mask = scan_mask;
  packet.penny_clip = penny_clip;
  if (penny_status.valid) {
    packet.penny_mct_fault_count = penny_status.mct_fault_count;
    packet.penny_isr_us = penny_status.isr_us;
    packet.penny_isr_max_us = penny_status.isr_max_us;
    packet.penny_i2c_us = penny_status.i2c_us;
    packet.penny_isr_overrun_count = penny_status.isr_overrun_count;
    packet.penny_i2c_timeout_count = penny_status.i2c_timeout_count;
    packet.penny_i2c_nack_count = penny_status.i2c_nack_count;
    packet.penny_i2c_recover_count = penny_status.i2c_recover_count;
    packet.penny_uart_overrun_errors = penny_status.uart_overrun_errors;
    packet.penny_tmag_sample_count = penny_status.tmag_sample_count;
    packet.penny_tmag_sample_dt_us = penny_status.tmag_sample_dt_us;
  }
  packet.gain_code = gain_code;
  packet.warn_flags = warn_flags;
  packet.clip_flags = clip_flags;
  packet.checksum = packetChecksum(packet);

  if (Serial.availableForWrite() >= static_cast<int>(sizeof(packet))) {
    Serial.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
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

  pinMode(kMotorGndPin, OUTPUT);
  digitalWrite(kMotorGndPin, LOW);
  esc.begin(Serial1, kMotorRxPin, kMotorTxPin);
  esc.setAddress(motor_id);
  stopMotor();
  setPennyClip(penny_clip);

  Serial.println("# mindaq torque pennyesc boot");
  while (!initAdc()) {
    Serial.println("# adc init failed");
    delay(500);
  }
  Serial.printf(
      "# telemetry sync=0x%04X packet_bytes=%u rate=1000 torque=Tz motor_baud=%lu motor_rx=%u motor_tx=%u\n",
      kSync, static_cast<unsigned>(sizeof(TelemetryPacket)), static_cast<unsigned long>(kPennyBaud),
      kMotorRxPin, kMotorTxPin);

  xTaskCreatePinnedToCore(adcTask, "adc", kTaskStackBytes, nullptr, kAdcPriority, nullptr, kAdcCore);
  xTaskCreatePinnedToCore(motorTask, "motor", kTaskStackBytes, nullptr, kMotorPriority, nullptr,
                          kMotorCore);
  next_loop_us = micros() + kLoopPeriodUs;
}

void loop() {
  readUsbCommands();
  const uint32_t now = micros();
  if (static_cast<int32_t>(now - next_loop_us) < 0) {
    return;
  }
  next_loop_us += kLoopPeriodUs;
  if (static_cast<int32_t>(micros() - next_loop_us) >= 0) {
    next_loop_us = micros() + kLoopPeriodUs;
  }

  sendTelemetry(micros());
}
