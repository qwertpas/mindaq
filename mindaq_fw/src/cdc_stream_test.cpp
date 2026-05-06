#include <Arduino.h>

namespace {

constexpr uint32_t kRateHz = 32000;
constexpr uint32_t kSync = 0xA55AA55A;
constexpr uint8_t kChannels = 6;
constexpr uint8_t kBlockPackets = 32;
constexpr uint8_t kBlockCount = 16;
constexpr uint32_t kBlockPeriodUs = (1000000UL * kBlockPackets) / kRateHz;

#if ARDUINO_RUNNING_CORE == 0
constexpr BaseType_t kAdcCore = 1;
#else
constexpr BaseType_t kAdcCore = 0;
#endif

constexpr UBaseType_t kAdcPriority = 1;
constexpr uint32_t kAdcStackBytes = 8192;

struct __attribute__((packed)) Packet {
  uint32_t sync;
  uint32_t seq;
  uint8_t raw[kChannels * 3];
  uint8_t checksum;
};

struct Block {
  Packet packets[kBlockPackets];
};

Block blocks[kBlockCount];
QueueHandle_t free_blocks = nullptr;
QueueHandle_t ready_blocks = nullptr;
volatile bool streaming = false;
volatile uint32_t work_sink = 0;

uint8_t checksum(const Packet &packet) {
  const auto *bytes = reinterpret_cast<const uint8_t *>(&packet);
  uint8_t value = 0;
  for (size_t i = 0; i + 1 < sizeof(packet); ++i) {
    value ^= bytes[i];
  }
  return value;
}

void simulateAdcWork(uint32_t seq) {
  uint32_t value = seq;
  for (uint8_t i = 0; i < 10; ++i) {
    value = value * 1664525u + 1013904223u;
    work_sink ^= value;
  }
}

void fillPacket(Packet &packet, uint32_t seq) {
  packet.sync = kSync;
  packet.seq = seq;
  for (uint8_t i = 0; i < kChannels; ++i) {
    const uint32_t value = 0x00101010u + static_cast<uint32_t>(i) * 0x00010101u;
    packet.raw[i * 3 + 0] = static_cast<uint8_t>(value & 0xFFu);
    packet.raw[i * 3 + 1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
    packet.raw[i * 3 + 2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
  }
  packet.checksum = checksum(packet);
}

void resetQueues() {
  xQueueReset(free_blocks);
  xQueueReset(ready_blocks);
  for (uint8_t i = 0; i < kBlockCount; ++i) {
    xQueueSend(free_blocks, &i, 0);
  }
}

void adcTask(void *param) {
  (void)param;
  uint32_t seq = 0;
  uint32_t next_block_us = micros();

  while (true) {
    if (!streaming) {
      seq = 0;
      vTaskDelay(1);
      next_block_us = micros();
      continue;
    }

    const uint32_t now_us = micros();
    if (static_cast<int32_t>(now_us - next_block_us) < 0) {
      vTaskDelay(1);
      continue;
    }
    next_block_us += kBlockPeriodUs;

    uint8_t block_index = 0;
    if (xQueueReceive(free_blocks, &block_index, 0) != pdTRUE) {
      seq += kBlockPackets;
      continue;
    }

    Block &block = blocks[block_index];
    for (uint8_t i = 0; i < kBlockPackets; ++i) {
      simulateAdcWork(seq);
      fillPacket(block.packets[i], seq);
      ++seq;
    }

    if (xQueueSend(ready_blocks, &block_index, 0) != pdTRUE) {
      xQueueSend(free_blocks, &block_index, 0);
    }
  }
}

}  // namespace

void setup() {
  Serial.setTxBufferSize(8192);
  Serial.begin(2000000);
  free_blocks = xQueueCreate(kBlockCount, sizeof(uint8_t));
  ready_blocks = xQueueCreate(kBlockCount, sizeof(uint8_t));
  resetQueues();
  xTaskCreatePinnedToCore(adcTask, "adc", kAdcStackBytes, nullptr, kAdcPriority, nullptr, kAdcCore);
}

void loop() {
  static bool was_connected = false;
  const bool connected = Serial;

  if (!connected) {
    streaming = false;
    was_connected = false;
    uint8_t block_index = 0;
    while (xQueueReceive(ready_blocks, &block_index, 0) == pdTRUE) {
      xQueueSend(free_blocks, &block_index, 0);
    }
    delay(1);
    return;
  }

  if (!was_connected) {
    streaming = false;
    resetQueues();
    streaming = true;
    was_connected = true;
    delay(2);
    return;
  }

  uint8_t block_index = 0;
  if (xQueueReceive(ready_blocks, &block_index, pdMS_TO_TICKS(1)) != pdTRUE) {
    return;
  }

  if (!connected) {
    xQueueSend(free_blocks, &block_index, 0);
    return;
  }

  const size_t bytes = sizeof(blocks[block_index]);
  const uint8_t *data = reinterpret_cast<const uint8_t *>(&blocks[block_index]);
  size_t written = 0;
  while (written < bytes && Serial) {
    written += Serial.write(data + written, bytes - written);
    if (written < bytes) {
      delay(1);
    }
  }
  xQueueSend(free_blocks, &block_index, 0);
}
