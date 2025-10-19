#include <Wire.h>
#include "Adafruit_VL53L1X.h"

// ==== Board pins (ESP32-C3 Mini) ====
constexpr int PIN_SDA   = 8;
constexpr int PIN_SCL   = 9;
constexpr int PIN_XSHUT = -1;   // set to a real GPIO if wired
constexpr int PIN_INT   = -1;   // set to a real GPIO if wired (active low)

// ==== Ranging config ====
constexpr uint16_t TIMING_BUDGET_MS = 200;  // 200 ms = ~5 Hz
constexpr uint32_t POLL_INTERVAL_MS = 20;   // poll every 20 ms

Adafruit_VL53L1X vl53(PIN_XSHUT, PIN_INT);

// ---- Non-blocking init / retry state ----
enum class SensorState : uint8_t { STARTUP, TRY_INIT, RUNNING, ERROR_RETRY };
SensorState state = SensorState::STARTUP;
uint32_t t_next_action = 0;      // millis() timestamp for next state action
uint32_t retry_backoff_ms = 500; // grows up to a cap

// ---- Optional interrupt support ----
volatile bool vl53_data_ready_irq = false;
#if (PIN_INT >= 0)
void IRAM_ATTR vl53_isr() { vl53_data_ready_irq = true; }
#endif

// ---- 5-sample moving average buffer ----
constexpr size_t AVG_N = 5;
int16_t avg_buf[AVG_N] = {0};
size_t  avg_idx = 0;
size_t  avg_count = 0;   // number of valid samples in buffer
int32_t avg_sum = 0;     // running sum of samples

inline void avg_reset() {
  avg_idx = 0;
  avg_count = 0;
  avg_sum = 0;
  for (size_t i = 0; i < AVG_N; ++i) avg_buf[i] = 0;
}

inline void avg_push(int16_t mm) {
  if (avg_count == AVG_N) {
    avg_sum -= avg_buf[avg_idx];   // remove oldest
  } else {
    avg_count++;
  }
  avg_buf[avg_idx] = mm;
  avg_sum += mm;
  avg_idx = (avg_idx + 1) % AVG_N;
}

inline float avg_value() {
  return (avg_count > 0) ? (float)avg_sum / (float)avg_count : NAN;
}

// ---- Helpers ----
bool beginVL53Nonblocking() {
  if (!vl53.begin(0x29, &Wire)) {
    return false;
  }
  vl53.setTimingBudget(TIMING_BUDGET_MS);
  vl53.startRanging();
  avg_reset();
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.setPins(PIN_SDA, PIN_SCL);
  Wire.begin();

#if (PIN_INT >= 0)
  pinMode(PIN_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_INT), vl53_isr, FALLING);
#endif

  state = SensorState::TRY_INIT;
  t_next_action = millis();
  Serial.println("Booting VL53L1X (non-blocking)...");
}

void loop() {
  const uint32_t now = millis();

  switch (state) {
    case SensorState::STARTUP:
      state = SensorState::TRY_INIT;
      t_next_action = now;
      break;

    case SensorState::TRY_INIT:
      if (now >= t_next_action) {
        if (beginVL53Nonblocking()) {
          Serial.println("VL53L1X ready");
          state = SensorState::RUNNING;
          retry_backoff_ms = 500;
        } else {
          Serial.print("Init failed (status=");
          Serial.print(vl53.vl_status);
          Serial.println("). Will retry.");
          state = SensorState::ERROR_RETRY;
          t_next_action = now + retry_backoff_ms;
          retry_backoff_ms = min<uint32_t>(retry_backoff_ms * 2, 8000);
        }
      }
      break;

    case SensorState::RUNNING: {
      bool data_ready = false;

#if (PIN_INT >= 0)
      if (vl53_data_ready_irq) {
        vl53_data_ready_irq = false;
        data_ready = true;
      } else
#endif
      {
        static uint32_t t_last_poll = 0;
        if (now - t_last_poll >= POLL_INTERVAL_MS) {
          t_last_poll = now;
          data_ready = vl53.dataReady();
        }
      }

      if (data_ready) {
        int16_t mm = vl53.distance();   // -1 on error/out of range
        vl53.clearInterrupt();

        if (mm >= 0) {
          avg_push(mm);
          float mean_mm = avg_value();

          Serial.print("Raw: ");
          Serial.print(mm);
          Serial.print(" mm   |   Avg(");
          Serial.print(avg_count);
          Serial.print("): ");
          Serial.print(mean_mm, 1);
          Serial.println(" mm");
        } else {
          Serial.print("Read error/OOR, status=");
          Serial.println(vl53.vl_status);
        }
      }
      break;
    }

    case SensorState::ERROR_RETRY:
      if (now >= t_next_action) {
        state = SensorState::TRY_INIT;
      }
      break;
  }
}
