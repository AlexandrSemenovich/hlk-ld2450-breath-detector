#include <Arduino.h>
#include <HardwareSerial.h>

#include "ld2450_parser.h"
#include "radar_bridge.h"

// Hardware config
static constexpr uint8_t  LD2450_RX_PIN = 32;
static constexpr uint8_t  LD2450_TX_PIN = 33;
static constexpr uint32_t LD2450_BAUD   = 256000;
static constexpr uint32_t PC_BAUD       = 921600;

// Radar reports at ~10 Hz (datasheet). Used only as a coarse resync guard:
// if the byte stream goes silent, a partial frame can get stuck in the buffer.
static constexpr uint32_t FRAME_TIMEOUT_MS = 1000;

HardwareSerial radarSerial(2);

static ld2450::Parser parser;
static uint32_t       last_frame_ms = 0;
static uint32_t       frame_counter = 0;

static void pollRadar(uint32_t now) {
  if (parser.size() > 0 && (now - last_frame_ms) > FRAME_TIMEOUT_MS) {
    parser.reset();
  }

  while (radarSerial.available()) {
    parser.feed(radarSerial.read());
  }

  ld2450::Frame f;
  while (parser.drain(f)) {
    last_frame_ms = now;
    bridge::sendTargets(f, now, ++frame_counter);
  }
}

void setup() {
  bridge::begin(PC_BAUD);
  radarSerial.begin(LD2450_BAUD, SERIAL_8N1, LD2450_RX_PIN, LD2450_TX_PIN);
  delay(3000);
}

void loop() {
  pollRadar(millis());  
}
