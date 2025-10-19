// Install: Library Manager -> "VL53L1X" by Pololu
#include <Wire.h>
#include <VL53L1X.h>

constexpr int PIN_SDA = 8;
constexpr int PIN_SCL = 9;

// Your XSHUT pins (one per sensor)
constexpr uint8_t XSHUT_PINS[] = { 2, 1, 37, 36, 35 };
constexpr uint8_t NUM_SENSORS  = sizeof(XSHUT_PINS);

// New unique 7-bit I2C addresses to assign (avoid default 0x29)
constexpr uint8_t NEW_ADDRS[NUM_SENSORS] = { 0x30, 0x31, 0x32, 0x33, 0x34 };

VL53L1X tof[NUM_SENSORS];

void holdAllInReset() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
  }
}

bool bringUpOne(uint8_t i) {
  // release just this sensor
  digitalWrite(XSHUT_PINS[i], HIGH);
  delay(5); // boot time

  tof[i].setBus(&Wire);
  tof[i].setAddress(0x29);          // talk to default address first (optional but explicit)
  if (!tof[i].init()) {             // init at 0x29
    Serial.printf("Sensor %u @XSHUT %u: init failed\n", i, XSHUT_PINS[i]);
    return false;
  }

  // change to a unique address (persists only until power-cycle)
  tof[i].setAddress(NEW_ADDRS[i]);

  // basic config (optional)
  tof[i].setDistanceMode(VL53L1X::Long);      // Short/Medium/Long
  tof[i].setMeasurementTimingBudget(50000);   // us (e.g., 50 ms)
  tof[i].startContinuous(50);                 // 50 ms per reading

  Serial.printf("Sensor %u up: XSHUT %u -> I2C 0x%02X\n", i, XSHUT_PINS[i], NEW_ADDRS[i]);
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }
  Serial.println("\nVL53L1X (Pololu) multi-sensor bring-up");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000); // start at 100 kHz

  holdAllInReset();
  delay(10);

  // bring them up one-by-one to avoid 0x29 collisions
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (!bringUpOne(i)) {
      Serial.printf(">> Skipping sensor %u due to init error\n", i);
    }
    delay(10);
  }
}

void loop() {
  static uint32_t t_last = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint16_t mm = tof[i].read();
    Serial.printf("[0x%02X] %u mm  ", NEW_ADDRS[i], mm);
  }
  Serial.println();

  // gentle print rate
  while (millis() - t_last < 200) { delay(1); }
  t_last = millis();
}