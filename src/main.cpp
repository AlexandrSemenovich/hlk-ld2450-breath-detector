#include <Arduino.h>

#define RXD2 32
#define TXD2 33

// Data buffer
byte buffer[64];
int pos = 0;

// Diagnostic counters
unsigned long byte_count = 0;
unsigned long packet_count = 0;
unsigned long valid_packet_count = 0;

// Mode flags
bool raw_mode = false;
bool teleplot_mode = false;
bool debug_mode = false;
bool breath_mode = true;

// Breathing detection variables
int breath_count = 0;
int last_y = 0;
bool initialized = false;
int min_y = 0;
int max_y = 0;
unsigned long last_breath_time = 0;
unsigned long detection_start_time = 0;

// Person detection
bool person_present = false;
unsigned long last_person_detected = 0;
unsigned long person_stability_start = 0;
bool person_stable = false;

// Improved breathing detection
bool inhale_detected = false;
bool exhale_detected = false;
int breath_threshold = 20;
unsigned long last_breath_cycle_start = 0;

// Timing for breath detection
unsigned long last_peak_time = 0;
unsigned long last_trough_time = 0;

// Teleplot timing
unsigned long last_teleplot_send = 0;

// Breathing analysis window
#define BREATH_WINDOW_SIZE 20
int y_history[BREATH_WINDOW_SIZE] = {0};
int history_index = 0;

// Function declarations
void printRawData(byte* p);
void sendTeleplotData(byte* p);
void printDebugData(byte* p);
void detectBreathing(byte* p);
void updatePersonDetection(byte* p);
int calculateCurrentAmplitude();

void setup() {
  Serial.begin(115200);
  Serial2.begin(256000, SERIAL_8N1, RXD2, TXD2);
  Serial.println("--- HLK-LD2450 Breathing Detector ---");
  Serial.println("Commands:");
  Serial.println("  raw     - Raw data mode");
  Serial.println("  teleplot - Teleplot mode");
  Serial.println("  debug   - Debug mode");
  Serial.println("  breath  - Breathing detection mode (default)");
  Serial.println("  status  - System status");
  Serial.println("  reset   - Reset breath count");
  Serial.println("----------------------------");
}

void loop() {
  // Process serial commands
  while (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    
    if (command == "raw") {
      raw_mode = true;
      teleplot_mode = false;
      debug_mode = false;
      breath_mode = false;
      Serial.println("Raw mode: ON");
    } 
    else if (command == "teleplot") {
      raw_mode = false;
      teleplot_mode = true;
      debug_mode = false;
      breath_mode = false;
      Serial.println("Teleplot mode: ON");
    }
    else if (command == "debug") {
      raw_mode = false;
      teleplot_mode = false;
      debug_mode = true;
      breath_mode = false;
      Serial.println("Debug mode: ON");
    }
    else if (command == "breath") {
      raw_mode = false;
      teleplot_mode = false;
      debug_mode = false;
      breath_mode = true;
      Serial.println("Breathing detection mode: ON");
    }
    else if (command == "status") {
      Serial.println("--- System Status ---");
      Serial.print("Bytes: ");
      Serial.print(byte_count);
      Serial.print(" | Packets: ");
      Serial.print(packet_count);
      Serial.print(" | Valid: ");
      Serial.println(valid_packet_count);
      Serial.print("Person present: ");
      Serial.print(person_present ? "YES" : "NO");
      Serial.print(" | Stable: ");
      Serial.println(person_stable ? "YES" : "NO");
      Serial.print("Breath count: ");
      Serial.println(breath_count);
      Serial.println("-------------------");
    }
    else if (command == "reset") {
      breath_count = 0;
      initialized = false;
      inhale_detected = false;
      exhale_detected = false;
      person_present = false;
      person_stable = false;
      Serial.println("Counters reset");
    }
  }

  // Process radar data
  while (Serial2.available()) {
    byte b = Serial2.read();
    byte_count++;
    buffer[pos++] = b;

    // Synchronize with packet header 0xAA 0xFF 0x03 0x00
    if (pos >= 4 && buffer[pos-4] == 0xAA && buffer[pos-3] == 0xFF && buffer[pos-2] == 0x03 && buffer[pos-1] == 0x00) {
      pos = 4;
      packet_count++;
    }

    // Process when full packet received (30 bytes)
    if (pos == 30) {
      if (buffer[28] == 0x55 && buffer[29] == 0xCC) {
        valid_packet_count++;
        
        // Always update person detection
        updatePersonDetection(buffer);
        
        if (raw_mode) {
          printRawData(buffer);
        } 
        else if (teleplot_mode) {
          sendTeleplotData(buffer);
        }
        else if (debug_mode) {
          printDebugData(buffer);
        }
        else if (breath_mode) {
          detectBreathing(buffer);
        }
      }
      pos = 0;
    }

    if (pos >= 60) pos = 0; // Overflow protection
  }
  
  // Periodic status in breath mode
  static unsigned long last_status = 0;
  if (breath_mode && millis() - last_status > 5000) {
    Serial.print("Person: ");
    Serial.print(person_present ? "PRESENT" : "ABSENT");
    Serial.print(" | Stable: ");
    Serial.print(person_stable ? "YES" : "NO");
    Serial.print(" | Breath: ");
    Serial.println(breath_count);
    last_status = millis();
  }
}

void printRawData(byte* p) {
  Serial.print("RAW:");
  for(int i = 0; i < 30; i++) {
    if(p[i] < 0x10) Serial.print("0");
    Serial.print(p[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void sendTeleplotData(byte* p) {
  // Limit teleplot data rate to prevent overload
  if (millis() - last_teleplot_send < 50) {
    return;
  }
  last_teleplot_send = millis();
  
  unsigned long timestamp = millis();
  
  // Target data
  int16_t x_coord = (int16_t)(p[4] | (p[5] << 8));
  int16_t y_coord = (int16_t)(p[6] | (p[7] << 8));
  int16_t speed = (int16_t)(p[8] | (p[9] << 8));
  uint16_t quality = (uint16_t)(p[10] | (p[11] << 8));
  
  // Send data in proper Teleplot format: >variable:timestamp:value
  Serial.print(">y_position:");
  Serial.print(timestamp);
  Serial.print(":");
  Serial.println(y_coord);
  
  Serial.print(">x_position:");
  Serial.print(timestamp);
  Serial.print(":");
  Serial.println(x_coord);
  
  Serial.print(">speed:");
  Serial.print(timestamp);
  Serial.print(":");
  Serial.println(speed);
  
  Serial.print(">quality:");
  Serial.print(timestamp);
  Serial.print(":");
  Serial.println(quality);
  
  Serial.print(">person_present:");
  Serial.print(timestamp);
  Serial.print(":");
  Serial.println(person_present ? 1 : 0);
  
  Serial.print(">breath_count:");
  Serial.print(timestamp);
  Serial.print(":");
  Serial.println(breath_count);
  
  // Send current breathing amplitude for visualization
  int current_amplitude = calculateCurrentAmplitude();
  Serial.print(">breathing_amplitude:");
  Serial.print(timestamp);
  Serial.print(":");
  Serial.println(current_amplitude);
  
  // Send min/max for visualization
  Serial.print(">min_y:");
  Serial.print(timestamp);
  Serial.print(":");
  Serial.println(min_y);
  
  Serial.print(">max_y:");
  Serial.print(timestamp);
  Serial.print(":");
  Serial.println(max_y);
}

void printDebugData(byte* p) {
  int16_t x_coord = (int16_t)(p[4] | (p[5] << 8));
  int16_t y_coord = (int16_t)(p[6] | (p[7] << 8));
  int16_t speed = (int16_t)(p[8] | (p[9] << 8));
  uint16_t quality = (uint16_t)(p[10] | (p[11] << 8));
  uint8_t target_count = p[3];
  
  Serial.print("TARGETS:");
  Serial.print(target_count);
  Serial.print(" | X:");
  Serial.print(x_coord);
  Serial.print(" | Y:");
  Serial.print(y_coord);
  Serial.print(" | SPD:");
  Serial.print(speed);
  Serial.print(" | Q:");
  Serial.print(quality);
  Serial.print(" | Person:");
  Serial.println(person_present ? "YES" : "NO");
}

void updatePersonDetection(byte* p) {
  uint16_t quality = (uint16_t)(p[10] | (p[11] << 8));
  int16_t y_coord = (int16_t)(p[6] | (p[7] << 8));
  int16_t x_coord = (int16_t)(p[4] | (p[5] << 8));
  uint8_t target_count = p[3];
  
  // Relaxed person detection criteria
  bool valid_target = (
    quality > 15 &&              // Minimum quality
    quality < 2000 &&            // Not saturated
    abs(y_coord) > 20000 &&      // Reasonable distance (negative values)
    abs(x_coord) < 10000 &&      // Reasonable horizontal position
    target_count > 0             // At least one target
  );
  
  if (valid_target) {
    last_person_detected = millis();
    if (!person_present) {
      person_present = true;
      person_stability_start = millis();
      Serial.println("PERSON DETECTED");
    }
    
    // Person considered stable after 1 second
    if (millis() - person_stability_start > 1000) {
      person_stable = true;
    }
  } else {
    // No person detected for more than 5 seconds
    if (millis() - last_person_detected > 5000) {
      if (person_present) {
        Serial.println("PERSON LOST");
        person_present = false;
        person_stable = false;
        // Reset breath count when person leaves
        breath_count = 0;
        inhale_detected = false;
        exhale_detected = false;
      }
    }
  }
}

int calculateCurrentAmplitude() {
  // Calculate amplitude over recent history
  if (history_index < 5) return 0; // Not enough data
  
  int local_min = y_history[0];
  int local_max = y_history[0];
  
  for (int i = 0; i < BREATH_WINDOW_SIZE && i < history_index; i++) {
    if (y_history[i] < local_min) local_min = y_history[i];
    if (y_history[i] > local_max) local_max = y_history[i];
  }
  
  return local_max - local_min;
}

void detectBreathing(byte* p) {
  int16_t y_coord = (int16_t)(p[6] | (p[7] << 8));
  uint16_t quality = (uint16_t)(p[10] | (p[11] << 8));
  
  // Add to history
  y_history[history_index % BREATH_WINDOW_SIZE] = y_coord;
  if (history_index < BREATH_WINDOW_SIZE) history_index++;
  
  // Relaxed quality check
  if (quality < 10) {
    return; // Very poor signal quality
  }
  
  if (!initialized) {
    last_y = y_coord;
    min_y = y_coord;
    max_y = y_coord;
    initialized = true;
    detection_start_time = millis();
    last_breath_cycle_start = millis();
    return;
  }
  
  // Track min/max values continuously
  if (y_coord < min_y) min_y = y_coord;
  if (y_coord > max_y) max_y = y_coord;
  
  // Improved peak detection for breathing with timing constraints
  static bool was_increasing = false;
  int delta = y_coord - last_y;
  
  // Minimum time between breath cycles (prevent rapid detections)
  if (millis() - last_breath_cycle_start < 1500) {
    // Too soon for another breath, but still track min/max
  } else {
    // Ready for next breath detection
    
    // Detect direction change with hysteresis
    if (!was_increasing && delta > 4) {
      // Start of inhale (moving toward sensor)
      was_increasing = true;
      inhale_detected = true;
      last_peak_time = millis();
    }
    else if (was_increasing && delta < -4) {
      // Start of exhale (moving away from sensor)
      was_increasing = false;
      exhale_detected = true;
      last_trough_time = millis();
    }
    
    // Count complete breath cycle (inhale + exhale) with timing constraint
    if (inhale_detected && exhale_detected) {
      // Check if this is a significant movement
      int amplitude = max_y - min_y;
      if (amplitude > breath_threshold) {
        // Additional timing check to prevent double counting
        if (millis() - last_breath_time > 2000) {
          // Valid breath cycle detected
          breath_count++;
          last_breath_time = millis();
          last_breath_cycle_start = millis();
          
          Serial.print("BREATH #");
          Serial.print(breath_count);
          Serial.print(" | Amp: ");
          Serial.print(amplitude);
          Serial.print(" | Range: ");
          Serial.print(min_y);
          Serial.print("..");
          Serial.println(max_y);
        }
      }
      
      // Reset for next cycle only after significant time
      if (millis() - last_breath_time > 5000) {
        inhale_detected = false;
        exhale_detected = false;
        min_y = y_coord;
        max_y = y_coord;
      }
    }
  }
  
  last_y = y_coord;
  
  // Reset if no breathing detected for too long
  if (millis() - last_breath_time > 30000) {
    min_y = y_coord;
    max_y = y_coord;
    inhale_detected = false;
    exhale_detected = false;
    last_breath_cycle_start = millis();
  }
}