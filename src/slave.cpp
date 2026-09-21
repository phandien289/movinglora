#include <Arduino.h>
#include "E32LoRaDriver.h"

/**
 * ============================================================================
 * ESP32-S3 SLAVE NODE - BỘ ĐIỀU KHIỂN ROBOT 3 BÁNH OMNI QUA LORA E32
 * ============================================================================
 * - Tối ưu phản hồi siêu tốc (< 50ms):
 *     + Xử lý gói tin rút gọn 2 bytes ("F", "B", "L", "R", "C", "W", "S", "H").
 *     + Core 0 quét buffer UART mỗi 1ms.
 *     + Gia tốc S-curve tăng tốc/giảm tốc cực nhanh (~35ms).
 * - Cảnh báo mất kết nối thông minh:
 *     + Nếu quá 500ms không nhận được bất kỳ tín hiệu nào (kể cả Heartbeat):
 *       -> Phanh dừng xe an toàn lập tức.
 *       -> NHẤP NHÁY ĐÈN LED RGB ĐỎ chu kỳ 250ms báo mất sóng.
 *     + Khi có sóng và đang đứng yên: Đèn sáng màu XANH LAM (Blue) báo sẵn sàng.
 *     + Khi di chuyển: Đèn đổi màu theo từng hướng chuyển động.
 *
 * PHÂN BỔ CHÂN ĐỘNG CƠ (GIỮ NGUYÊN 100% TỪ moving850):
 *   + Motor 1 (90°):  PU- = GPIO 4,  DR- = GPIO 5,  MF- = GPIO 6
 *   + Motor 2 (210°): PU- = GPIO 7,  DR- = GPIO 15, MF- = GPIO 16
 *   + Motor 3 (330°): PU- = GPIO 17, DR- = GPIO 18, MF- = GPIO 8
 *
 * PHÂN BỔ CHÂN LORA E32 (TRÁNH TOÀN BỘ CHÂN ĐÃ DÙNG):
 *   + RX1: GPIO 1 | TX1: GPIO 2 | AUX: GPIO 10 | M0: GPIO 11 | M1: GPIO 12
 * ============================================================================
 */

// Cấu trúc chân động cơ bước
struct StepperMotor {
  uint8_t pinPulse; // PU-
  uint8_t pinDir;   // DR-
  uint8_t pinEna;   // MF-
};

// 3 Động cơ bánh Omni
StepperMotor motors[3] = {
  {4,  5,  6 }, // Motor 1 (Bánh 1 - 90°)
  {7,  15, 16}, // Motor 2 (Bánh 2 - 210°)
  {17, 18, 8 }  // Motor 3 (Bánh 3 - 330°)
};

// Cấu hình chân LoRa E32 trên Slave
#define SLAVE_LORA_RX_PIN  1  // RX1 (nối TXD của LoRa)
#define SLAVE_LORA_TX_PIN  2  // TX1 (nối RXD của LoRa)
#define SLAVE_LORA_AUX_PIN 10 // AUX
#define SLAVE_LORA_M0_PIN  11 // M0
#define SLAVE_LORA_M1_PIN  12 // M1

// Cấu hình LED RGB WS2812 trên bo mạch ESP32-S3 DevKit
#ifdef RGB_BUILTIN
#define RGB_LED_PIN RGB_BUILTIN
#else
#define RGB_LED_PIN 48 // Mặc định GPIO 48 trên ESP32-S3 DevKit (hoặc GPIO 38)
#endif

// Bộ lọc trạng thái LED tránh gọi neopixelWrite liên tục gây xung đột
static uint8_t curR = 255, curG = 255, curB = 255;
inline void setRgb(uint8_t r, uint8_t g, uint8_t b) {
  if (r != curR || g != curG || b != curB) {
    curR = r; curG = g; curB = b;
    neopixelWrite(RGB_LED_PIN, r, g, b);
  }
}

E32LoRaDriver lora(&Serial1, 
                   SLAVE_LORA_RX_PIN, 
                   SLAVE_LORA_TX_PIN, 
                   SLAVE_LORA_AUX_PIN, 
                   SLAVE_LORA_M0_PIN, 
                   SLAVE_LORA_M1_PIN);

const unsigned long PULSES_PER_REV = 8000;

// Biến mục tiêu vận tốc từ Master (-1.0 đến 1.0)
volatile float targetVx = 0.0f; // Trục X (Trái / Phải)
volatile float targetVy = 0.0f; // Trục Y (Tiến / Lùi)
volatile float targetW  = 0.0f; // Xoay tròn (CW / CCW)
volatile int   targetRpm = 75;  // Tốc độ tối đa (RPM)
volatile bool  motorsEnabled = true;
volatile bool  invertEnPolarity = false;
volatile unsigned long lastCommandTime = 0;

// WATCHDOG AN TOÀN: Sau 650ms không nhận gói tin (hoặc heartbeat) -> Mất kết nối!
const unsigned long SAFETY_TIMEOUT_MS = 650;

// Gia tốc làm mịn chuyển động:
volatile float accelRate = 0.002f;    // Tăng tốc êm ái
volatile float decelRate = 0.007f;    // Hãm phanh từ từ
volatile float steeringTrim = 0.0f;   // Bù lệch bánh khi chạy thẳng

void applyEnPinState() {
  for (int i = 0; i < 3; i++) {
    bool shouldDisable = (!invertEnPolarity) ? !motorsEnabled : motorsEnabled;
    if (shouldDisable) {
      pinMode(motors[i].pinEna, OUTPUT);
      digitalWrite(motors[i].pinEna, LOW);
    } else {
      pinMode(motors[i].pinEna, INPUT);
    }
  }
}

// ============================================================================
// TASK CORE 1: PHÁT XUNG ĐỘNG CƠ & GIÁM SÁT MẤT KẾT NỐI (MOTOR TASK)
// ============================================================================
void motorTask(void *pvParameters) {
  uint32_t nextStepUs[3] = {0, 0, 0};
  uint32_t stepIntervalUs[3] = {0, 0, 0};
  bool motorActive[3] = {false, false, false};

  float currentVx = 0.0f;
  float currentVy = 0.0f;
  float currentW  = 0.0f;

  const float SQRT3_2 = 0.866025404f;

  while (true) {
    unsigned long nowMs = millis();
    bool isConnected = (nowMs - lastCommandTime <= SAFETY_TIMEOUT_MS);

    // XỬ LÝ KHI MẤT KẾT NỐI (TIMEOUT > 650ms):
    if (!isConnected) {
      // 1. Dập tắt ngay lập tức vận tốc vi bước (dừng phát xung lập tức, không trôi quán tính)
      currentVx = 0.0f;
      currentVy = 0.0f;
      currentW  = 0.0f;
      targetVx  = 0.0f;
      targetVy  = 0.0f;
      targetW   = 0.0f;

      // 2. Ngắt hẳn phần cứng động cơ (Disable Driver ENA pin)
      if (motorsEnabled) {
        motorsEnabled = false;
        applyEnPinState();
        Serial.println("[FAILSAFE] Mat tin hieu -> NGAT DONG CO, KHONG CHAY NUA!");
      }

      // 3. NHẤP NHÁY LED ĐỎ CẢNH BÁO MẤT SÓNG (chu kỳ 250ms)
      if ((nowMs / 250) % 2 == 0) {
        setRgb(200, 0, 0); // Đỏ sáng
      } else {
        setRgb(0, 0, 0);   // Tắt
      }
    }

    // Làm mịn vận tốc S-Curve Ramp
    auto rampValue = [](float current, float target, float accel, float decel) -> float {
      if (fabs(target) > fabs(current)) {
        if (target > current) {
          current += accel;
          if (current > target) current = target;
        } else {
          current -= accel;
          if (current < target) current = target;
        }
      } else {
        if (target > current) {
          current += decel;
          if (current > target) current = target;
        } else {
          current -= decel;
          if (current < target) current = target;
        }
        if (target == 0.0f && fabs(current) < 0.02f) {
          current = 0.0f;
        }
      }
      return current;
    };

    currentVx = rampValue(currentVx, targetVx, accelRate, decelRate);
    currentVy = rampValue(currentVy, targetVy, accelRate, decelRate);
    currentW  = rampValue(currentW,  targetW,  accelRate, decelRate);

    // Nếu xe đã dừng hẳn hoặc bị Disable
    if (!motorsEnabled || (currentVx == 0.0f && currentVy == 0.0f && currentW == 0.0f)) {
      vTaskDelay(pdMS_TO_TICKS(4));
      continue;
    }

    // Động học 3 bánh Omni:
    float vx = -currentVx; // Đảo dấu trục X
    float vy = currentVy;
    float w  = -(currentW + (steeringTrim * vy));

    float v1 = -vx + w;
    float v2 =  0.5f * vx - SQRT3_2 * vy + w;
    float v3 =  0.5f * vx + SQRT3_2 * vy + w;

    float speeds[3] = {v1, v2, v3};
    uint32_t now = micros();

    for (int i = 0; i < 3; i++) {
      float spd = speeds[i];
      if (fabs(spd) < 0.03f) {
        motorActive[i] = false;
      } else {
        motorActive[i] = true;
        digitalWrite(motors[i].pinDir, (spd >= 0) ? HIGH : LOW);

        float freq = fabs(spd) * ((float)targetRpm * (float)PULSES_PER_REV) / 60.0f;
        if (freq < 80.0f) freq = 80.0f;
        stepIntervalUs[i] = (uint32_t)(1000000.0f / freq);
      }
    }

    // Vòng lặp phát xung thời gian thực vi giây (~2.5ms mỗi chu kỳ)
    for (int loop = 0; loop < 80; loop++) {
      now = micros();
      bool steppedAny = false;

      for (int i = 0; i < 3; i++) {
        if (motorActive[i] && (int32_t)(now - nextStepUs[i]) >= 0) {
          nextStepUs[i] += stepIntervalUs[i];
          if ((int32_t)(now - nextStepUs[i]) > (int32_t)stepIntervalUs[i]) {
            nextStepUs[i] = now + stepIntervalUs[i];
          }
          digitalWrite(motors[i].pinPulse, LOW);
          steppedAny = true;
        }
      }

      if (steppedAny) {
        delayMicroseconds(6);
        for (int i = 0; i < 3; i++) {
          digitalWrite(motors[i].pinPulse, HIGH);
        }
      }

      delayMicroseconds(10);
    }
  }
}

// ============================================================================
// TASK CORE 0: NHẬN VÀ GIẢI MÃ LỆNH LORA TẦN SỐ CAO (LORA TASK)
// ============================================================================
void loraTask(void *pvParameters) {
  char rxMsg[64];

  while (true) {
    if (lora.readPacket(rxMsg, sizeof(rxMsg))) {
      // Khi có tín hiệu lại từ Master: bật lại động cơ nếu trước đó bị ngắt an toàn
      if (!motorsEnabled) {
        motorsEnabled = true;
        applyEnPinState();
        Serial.println("[RECONNECT] Da co tin hieu tro lai -> Kich hoat lai dong co!");
      }

      // Hỗ trợ cả lệnh rút gọn 1 ký tự ("F", "B", "L", "R", "C", "W", "S", "H")
      // lẫn định dạng cũ ("CMD:FWD", "CMD:BACK", v.v.)
      char firstChar = rxMsg[0];

      if (firstChar == 'F' || strstr(rxMsg, "FWD") != nullptr) {
        // Phím S1: Tiến -> Đèn Xanh Lá (Green)
        targetVx = 0.0f;
        targetVy = 1.0f;
        targetW  = 0.0f;
        lastCommandTime = millis();
        setRgb(0, 180, 0);
        Serial.println("[RX] -> TIEN TOI (F) [LED: XANH LA]");
      } 
      else if (firstChar == 'B' || strstr(rxMsg, "BACK") != nullptr) {
        // Phím S2: Lùi -> Đèn Vàng Cam (Yellow/Orange)
        targetVx = 0.0f;
        targetVy = -1.0f;
        targetW  = 0.0f;
        lastCommandTime = millis();
        setRgb(180, 80, 0);
        Serial.println("[RX] -> LUI LAI (B) [LED: VANG CAM]");
      } 
      else if (firstChar == 'L' || strstr(rxMsg, "LEFT") != nullptr) {
        // Phím S3: Ngang trái -> Đèn Xanh Lơ (Cyan)
        targetVx = -1.0f;
        targetVy = 0.0f;
        targetW  = 0.0f;
        lastCommandTime = millis();
        setRgb(0, 180, 180);
        Serial.println("[RX] -> NGANG TRAI (L) [LED: CYAN]");
      } 
      else if (firstChar == 'R' || strstr(rxMsg, "RIGHT") != nullptr) {
        // Phím S4: Ngang phải -> Đèn Tím Hồng (Magenta)
        targetVx = 1.0f;
        targetVy = 0.0f;
        targetW  = 0.0f;
        lastCommandTime = millis();
        setRgb(180, 0, 180);
        Serial.println("[RX] -> NGANG PHAI (R) [LED: MAGENTA]");
      } 
      else if (firstChar == 'C' || strstr(rxMsg, "ROT_L") != nullptr) {
        // Phím S5: Xoay trái (CCW) -> Đèn Xanh Dương (Blue)
        targetVx = 0.0f;
        targetVy = 0.0f;
        targetW  = -1.0f;
        lastCommandTime = millis();
        setRgb(0, 50, 240);
        Serial.println("[RX] -> XOAY TRAI (C) [LED: BLUE]");
      } 
      else if (firstChar == 'W' || strstr(rxMsg, "ROT_R") != nullptr) {
        // Phím S6: Xoay phải (CW) -> Đèn Trắng Sáng (White)
        targetVx = 0.0f;
        targetVy = 0.0f;
        targetW  = 1.0f;
        lastCommandTime = millis();
        setRgb(160, 160, 160);
        Serial.println("[RX] -> XOAY PHAI (W) [LED: WHITE]");
      } 
      else if (firstChar == 'S' || strstr(rxMsg, "STOP") != nullptr) {
        // Nhả phím: Dừng phanh ngay -> Đèn Xanh Lam sẵn sàng (Solid Blue)
        targetVx = 0.0f;
        targetVy = 0.0f;
        targetW  = 0.0f;
        lastCommandTime = millis();
        setRgb(0, 0, 90);
        Serial.println("[RX] -> DUNG PHANH (S) [LED: XANH SAN SANG]");
      }
      else if (firstChar == 'V') {
        // Lệnh cài đặt tốc độ từ Master (gói "V<RPM>", ví dụ "V75", "V80")
        int newRpm = atoi(rxMsg + 1);
        if (newRpm >= 20 && newRpm <= 200) {
          targetRpm = newRpm;
          lastCommandTime = millis();
          Serial.printf("[RX] -> CAI DAT TOC DO MOI: %d RPM\n", targetRpm);
        }
      }
      else if (firstChar == 'H' || strstr(rxMsg, "PING") != nullptr) {
        // Heartbeat báo sống từ Master -> Làm mới thời gian giữ kết nối
        lastCommandTime = millis();
        if (targetVx == 0.0f && targetVy == 0.0f && targetW == 0.0f) {
          setRgb(0, 0, 90); // Giữ đèn Xanh Lam sẵn sàng
        }
      }
    }

    // Quét cổng UART mỗi 1ms để đảm bảo độ trễ phản hồi < 50ms
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup() {
  Serial.begin(115200);

  // Timeout tránh kẹt Serial CDC khi chạy nguồn pin ngoài (User Global Rule)
  unsigned long start = millis();
  while (!Serial && (millis() - start < 3000)) { 
    delay(10); 
  }

  // Vô hiệu hóa Task Watchdog trên Core 1 để motorTask chạy trơn tru
  disableCore1WDT();

  Serial.println("\n========================================================");
  Serial.println(" ESP32-S3 SLAVE: ROBOT 3 BANH OMNI (LOW-LATENCY <50MS)  ");
  Serial.println("========================================================");

  // Cấu hình các chân điều khiển 3 động cơ bước
  for (int i = 0; i < 3; i++) {
    pinMode(motors[i].pinPulse, OUTPUT);
    pinMode(motors[i].pinDir,   OUTPUT);

    digitalWrite(motors[i].pinPulse, HIGH);
    digitalWrite(motors[i].pinDir,   HIGH);
  }
  applyEnPinState(); // Khởi tạo chân EN ở chế độ trở kháng cao (Hi-Z)

  Serial.println("[SETUP] Da cau hinh 9 chan dong co (GPIO 4,5,6 | 7,15,16 | 17,18,8)");

  // Khởi tạo LoRa E32-433T20D với Air Data Rate 19.2 kbps
  lora.begin(9600, true);
  Serial.println("[SETUP] Da khoi tao LoRa E32 (RX:1, TX:2, AUX:10, M0:11, M1:12 | 19.2 kbps)");

  // Task LoRa chạy trên Core 0 (độ ưu tiên cao, chu kỳ 1ms)
  xTaskCreatePinnedToCore(
    loraTask,
    "LoraTask",
    4096,
    NULL,
    3,
    NULL,
    0  // CORE 0
  );

  // Task Động cơ chạy độc quyền trên Core 1
  xTaskCreatePinnedToCore(
    motorTask,
    "MotorTask",
    4096,
    NULL,
    2,
    NULL,
    1  // CORE 1
  );

  Serial.println("[OK] Da khoi tao song song 2 Core: Core 0 (LoRa) & Core 1 (Motor)!");
  
  // Khởi đầu đặt lastCommandTime = 0 để nếu chưa bật Master, Slave sẽ nhấp nháy đỏ ngay
  lastCommandTime = 0;
  Serial.println("Robot 3 banh Omni san sang! (Neu chua ket noi Master: LED Do nhap nhay)");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(100));
}
