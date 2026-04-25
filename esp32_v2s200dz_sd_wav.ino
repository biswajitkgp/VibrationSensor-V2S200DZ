#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "driver/i2s.h"

// -------------------------
// User configuration
// -------------------------
static constexpr int SENSOR_CLK_PIN = 26;   // V2S200DZ CLK
static constexpr int SENSOR_DT_PIN  = 27;   // V2S200DZ DT
static constexpr int SD_CS_PIN      = 5;    // SD card CS

static constexpr i2s_port_t I2S_PORT = I2S_NUM_0;

// 16 kHz is a good general-purpose vibration logging rate for this sensor family.
static constexpr uint32_t SAMPLE_RATE_HZ = 16000;
static constexpr uint16_t BITS_PER_SAMPLE = 16;
// Two-sensor PDM capture on one data line (SEL/LR strapped opposite):
// sensor A SEL -> GND (Left), sensor B SEL -> 3V3 (Right)
static constexpr uint16_t CHANNEL_COUNT = 2;

// Record length in seconds. Increase as needed.
static constexpr uint32_t RECORD_SECONDS = 20;
static constexpr size_t I2S_READ_BYTES = 2048;

const char* WAV_PATH = "/vibration.wav";

File wavFile;
uint32_t totalAudioBytes = 0;

void writeLE16(File &f, uint16_t v) {
  uint8_t b[2] = {
    static_cast<uint8_t>(v & 0xFF),
    static_cast<uint8_t>((v >> 8) & 0xFF)
  };
  f.write(b, sizeof(b));
}

void writeLE32(File &f, uint32_t v) {
  uint8_t b[4] = {
    static_cast<uint8_t>(v & 0xFF),
    static_cast<uint8_t>((v >> 8) & 0xFF),
    static_cast<uint8_t>((v >> 16) & 0xFF),
    static_cast<uint8_t>((v >> 24) & 0xFF)
  };
  f.write(b, sizeof(b));
}

void writeWavHeaderPlaceholder(File &f, uint32_t sampleRate, uint16_t bitsPerSample, uint16_t channels) {
  uint32_t byteRate = sampleRate * channels * bitsPerSample / 8;
  uint16_t blockAlign = channels * bitsPerSample / 8;

  // RIFF chunk
  f.write((const uint8_t*)"RIFF", 4);
  writeLE32(f, 36);  // Placeholder: 36 + dataSize
  f.write((const uint8_t*)"WAVE", 4);

  // fmt chunk
  f.write((const uint8_t*)"fmt ", 4);
  writeLE32(f, 16);                    // PCM chunk size
  writeLE16(f, 1);                     // Audio format: PCM
  writeLE16(f, channels);
  writeLE32(f, sampleRate);
  writeLE32(f, byteRate);
  writeLE16(f, blockAlign);
  writeLE16(f, bitsPerSample);

  // data chunk
  f.write((const uint8_t*)"data", 4);
  writeLE32(f, 0);                     // Placeholder for data size
}

bool patchWavHeader(File &f, uint32_t dataSize) {
  if (!f.seek(4)) return false;   // RIFF size
  writeLE32(f, 36 + dataSize);

  if (!f.seek(40)) return false;  // data chunk size
  writeLE32(f, dataSize);
  return true;
}

bool initSD() {
  // Use default VSPI pins on ESP32:
  // SCK=18, MISO=19, MOSI=23, CS=SD_CS_PIN
  // This matches the reference initialization that is known to work
  // on common ESP32-WROOM dev boards.
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("[ERR] SD.begin failed (check SD wiring/format/power)");
    return false;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("[ERR] No SD card attached");
    return false;
  }

  Serial.print("[OK] SD card type: ");
  switch (cardType) {
    case CARD_MMC:  Serial.println("MMC"); break;
    case CARD_SD:   Serial.println("SDSC"); break;
    case CARD_SDHC: Serial.println("SDHC/SDXC"); break;
    default:        Serial.println("UNKNOWN"); break;
  }

  uint64_t cardSizeMB = SD.cardSize() / (1024ULL * 1024ULL);
  Serial.printf("[OK] SD size: %llu MB\n", cardSizeMB);
  return true;
}

bool initI2SPDM() {
  i2s_config_t i2s_config = {};
  i2s_config.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
  i2s_config.sample_rate = SAMPLE_RATE_HZ;
  i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  i2s_config.dma_buf_count = 8;
  i2s_config.dma_buf_len = 256;
  i2s_config.use_apll = false;
  i2s_config.tx_desc_auto_clear = false;
  i2s_config.fixed_mclk = 0;

  i2s_pin_config_t pin_config = {};
  pin_config.bck_io_num = I2S_PIN_NO_CHANGE;
  pin_config.ws_io_num = SENSOR_CLK_PIN;   // PDM clock output to sensor
  pin_config.data_out_num = I2S_PIN_NO_CHANGE;
  pin_config.data_in_num = SENSOR_DT_PIN;  // PDM data from sensor

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, nullptr);
  if (err != ESP_OK) {
    Serial.printf("[ERR] i2s_driver_install failed: %d\n", err);
    return false;
  }

  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("[ERR] i2s_set_pin failed: %d\n", err);
    return false;
  }

  err = i2s_set_clk(I2S_PORT, SAMPLE_RATE_HZ, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  if (err != ESP_OK) {
    Serial.printf("[ERR] i2s_set_clk failed: %d\n", err);
    return false;
  }

  i2s_zero_dma_buffer(I2S_PORT);
  Serial.println("[OK] I2S PDM initialized");
  return true;
}

bool openWav() {
  if (SD.exists(WAV_PATH)) {
    SD.remove(WAV_PATH);
  }

  wavFile = SD.open(WAV_PATH, FILE_WRITE);
  if (!wavFile) {
    Serial.println("[ERR] Failed to open WAV file");
    return false;
  }

  writeWavHeaderPlaceholder(wavFile, SAMPLE_RATE_HZ, BITS_PER_SAMPLE, CHANNEL_COUNT);
  totalAudioBytes = 0;
  return true;
}

void recordWav(uint32_t seconds) {
  uint32_t targetBytes = SAMPLE_RATE_HZ * (BITS_PER_SAMPLE / 8) * CHANNEL_COUNT * seconds;
  uint8_t buffer[I2S_READ_BYTES];

  Serial.printf("[REC] Recording %lu s to %s\n", static_cast<unsigned long>(seconds), WAV_PATH);

  while (totalAudioBytes < targetBytes) {
    size_t bytesRead = 0;
    esp_err_t err = i2s_read(I2S_PORT, buffer, sizeof(buffer), &bytesRead, portMAX_DELAY);
    if (err != ESP_OK) {
      Serial.printf("[ERR] i2s_read failed: %d\n", err);
      break;
    }

    if (bytesRead > 0) {
      size_t written = wavFile.write(buffer, bytesRead);
      totalAudioBytes += written;

      if (written != bytesRead) {
        Serial.println("[ERR] SD write underrun");
        break;
      }
    }
  }

  wavFile.flush();
  if (!patchWavHeader(wavFile, totalAudioBytes)) {
    Serial.println("[ERR] Failed to patch WAV header");
  }
  wavFile.close();

  Serial.printf("[OK] Done. Audio bytes: %lu\n", static_cast<unsigned long>(totalAudioBytes));
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("\nESP32 + V2S200DZ -> SD WAV recorder");

  if (!initSD()) {
    Serial.println("[FATAL] SD init failed");
    while (true) delay(1000);
  }

  if (!initI2SPDM()) {
    Serial.println("[FATAL] I2S init failed");
    while (true) delay(1000);
  }

  if (!openWav()) {
    Serial.println("[FATAL] Cannot create WAV");
    while (true) delay(1000);
  }

  recordWav(RECORD_SECONDS);
  Serial.println("[INFO] Recording complete. Reset board to record again.");
}

void loop() {
  delay(1000);
}
