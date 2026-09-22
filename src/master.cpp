#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_BNO055.h"
#include "E32LoRaDriver.h"

/**
 * ============================================================================
 * ESP32-S3 MASTER NODE - BỘ ĐIỀU KHIỂN CỬ CHỈ BÀN TAY BẰNG DFROBOT BNO055
 * ============================================================================
 * - Giao tiếp I2C với cảm biến 9 trục DFRobot BNO055:
 *     + SDA: GPIO 4
 *     + SCL: GPIO 5
 * - Giao tiếp UART1 với module LoRa Ebyte E32-433T20D:
 *     + RX:  GPIO 16 (Serial1 RX)
 *     + TX:  GPIO 17 (Serial1 TX)
 *     + AUX: GPIO 18
 *     + M0:  GPIO 7
 *     + M1:  GPIO 15
 *
 * - ÁNH XẠ CỬ CHỈ BÀN TAY (ĐÃ HOÁN ĐỔI PITCH VÀ ROLL THEO HƯỚNG LẮP ĐẶT THỰC TẾ):
 *     1. Bàn tay để phẳng (Neutral Deadzone < 45°): DỪNG XE [S]
 *     2. Trục Roll đảm nhận TIẾN / LÙI:
 *        - Nghiêng chúc tay về trước (> 45°):       TIẾN [F]
 *        - Nghiêng ngửa tay ra sau   (> 45°):       LÙI [B]
 *     3. Trục Pitch đảm nhận TRÁI / PHẢI:
 *        - Nghiêng lật tay sang trái (> 45°):       NGANG TRÁI [L]
 *        - Nghiêng lật tay sang phải (> 45°):       NGANG PHẢI [R]
 *     4. Trục Z (Yaw) đảm nhận XOAY TẠI CHỖ:
 *        - Xoay cổ tay thuận KĐH     (> 35°):       XOAY PHẢI (CW) [W]
 *        - Xoay cổ tay ngược KĐH     (> 35°):       XOAY TRÁI (CCW) [C]
 * ============================================================================
 */

// Cấu hình chân I2C cho DFRobot BNO055
#define I2C_SDA_PIN  4
#define I2C_SCL_PIN  5
#define BNO055_ADDR  0x28 // Địa chỉ I2C mặc định của DFRobot BNO055 (hoặc 0x29)

// Cấu hình chân LoRa Ebyte E32-433T20D trên Master (ESP32-S3)
#define LORA_RX_PIN  16 // ESP32-S3 RX1 (nối TXD của LoRa E32)
#define LORA_TX_PIN  17 // ESP32-S3 TX1 (nối RXD của LoRa E32)
#define LORA_AUX_PIN 18 // ESP32-S3 Input (nối AUX của LoRa E32)
#define LORA_M0_PIN  7  // ESP32-S3 Output (nối M0 của LoRa E32)
#define LORA_M1_PIN  15 // ESP32-S3 Output (nối M1 của LoRa E32)

// Khởi tạo đối tượng BNO055 từ thư viện DFRobot
typedef DFRobot_BNO055_IIC BNO;
BNO bno(&Wire, BNO055_ADDR);

// Khởi tạo driver LoRa E32
E32LoRaDriver lora(&Serial1, LORA_RX_PIN, LORA_TX_PIN, LORA_AUX_PIN, LORA_M0_PIN, LORA_M1_PIN);

// ============================================================================
// CÀI ĐẶT NGƯỠNG GÓC KÍCH HOẠT (SETPOINTS) & ĐẢO CHIỀU NẾU CẦN
// ============================================================================
const float SETPOINT_FWD_BACK   = 45.0f; // Ngưỡng kích hoạt Tiến / Lùi (>= 45°)
const float SETPOINT_LEFT_RIGHT = 45.0f; // Ngưỡng kích hoạt Trái / Phải (>= 45°)
const float SETPOINT_Z_YAW      = 35.0f; // Ngưỡng kích hoạt Xoay CW / CCW (>= 35°)

// Cờ đảo chiều (Đổi true / false nếu chiều chuyển động bị ngược):
const bool INVERT_FWD_BACK   = false; // Đổi true nếu chúc tay trước mà xe lại lùi
const bool INVERT_LEFT_RIGHT = true;  // Đảo chiều: Lật tay trái -> Sang Trái (L), Lật tay phải -> Sang Phải (R)
const bool INVERT_Z_YAW      = false; // Đổi true nếu xoay tay phải mà xe xoay trái

// Quản lý mốc tham chiếu trục Z (Yaw Tare / Zeroing)
float baseYaw = 0.0f;
bool isBaseYawSet = false;
unsigned long neutralStartTime = 0;
const unsigned long AUTO_TARE_DELAY_MS = 1500; // Để phẳng tay liên tục 1.5s mới tự động cân lại mốc Yaw

// Quản lý truyền LoRa
const char* lastSentCmd = "S";
unsigned long lastSendTime = 0;
unsigned long lastHeartbeatTime = 0;
const unsigned long SEND_INTERVAL_MS = 70;       // Gửi lặp lại mỗi 70ms khi đang điều khiển
const unsigned long HEARTBEAT_INTERVAL_MS = 200; // Gửi heartbeat mỗi 200ms khi đứng yên
const unsigned long LOG_INTERVAL_MS = 200;       // In log Serial mỗi 200ms
unsigned long lastLogTime = 0;

// Hàm chuẩn hóa hiệu số 2 góc Yaw (-180° đến +180°)
float normalizeAngleDiff(float angle, float reference) {
    float diff = angle - reference;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

void setup() {
    Serial.begin(115200);

    // Timeout tránh treo khi chạy nguồn ngoài/pin (User Global Rule)
    unsigned long start = millis();
    while (!Serial && (millis() - start < 3000)) { 
        delay(10); 
    }

    Serial.println("\n========================================================");
    Serial.println(" ESP32-S3 MASTER: GESTURE CONTROLLER (DFROBOT BNO055)  ");
    Serial.println("========================================================");

    // 1. Khởi tạo bus I2C trên GPIO 4 (SDA) và GPIO 5 (SCL)
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
    Serial.printf("[I2C] Khoi tao bus I2C tren SDA=%d, SCL=%d (400kHz)\n", I2C_SDA_PIN, I2C_SCL_PIN);

    // 2. Khởi tạo cảm biến DFRobot BNO055
    bno.reset();
    delay(50);

    bool bnoReady = false;
    for (int retry = 0; retry < 5; retry++) {
        if (bno.begin() == BNO::eStatusOK) {
            bnoReady = true;
            break;
        }
        Serial.printf("[WARN] BNO055 chua phan hoi o dia chi 0x%02X, thu lai lan %d...\n", BNO055_ADDR, retry + 1);
        delay(200);
    }

    if (bnoReady) {
        // Sử dụng chế độ IMU (Con quay hồi chuyển + Gia tốc kế):
        // KHÔNG phụ thuộc vào từ kế, phản hồi trục Z (Heading) tức thì từ lúc bật nguồn!
        bno.setOprMode(BNO::eOprModeImu);
        Serial.println("[OK] DFRobot BNO055 san sang o che do IMU Fusion (Phan hoi truc Z tuc thi)");
    } else {
        Serial.println("[ERROR] KHONG TIM THAY BNO055! Kiem tra day noi SDA (4), SCL (5), VCC, GND.");
    }

    // 3. Khởi tạo LoRa E32-433T20D với Air Data Rate 19.2 kbps
    lora.begin(9600, true);
    Serial.println("[OK] LoRa E32-433T20D san sang (Air Data Rate: 19.2 kbps)");

    // Lấy mốc Yaw đầu tiên sau khi cảm biến ổn định
    delay(200);
    BNO::sEulAnalog_t initEul = bno.getEul();
    baseYaw = initEul.head;
    isBaseYawSet = true;

    Serial.println("--------------------------------------------------------");
    Serial.println("HUONG DAN CU CHI BAN TAY (SETPOINT 45° / Z: 35°):");
    Serial.println(" - De phang tay (<45 do)         -> Dung xe (S)");
    Serial.println(" - Nghieng chuc tay truoc >45 do -> Tien (F) [Theo Roll]");
    Serial.println(" - Nghieng ngua tay sau   >45 do -> Lui (B) [Theo Roll]");
    Serial.println(" - Nghieng lat tay trai   >45 do -> Ngang Trai (L) [Theo Pitch]");
    Serial.println(" - Nghieng lat tay phai   >45 do -> Ngang Phai (R) [Theo Pitch]");
    Serial.println(" - Xoay co tay thuan KDH  >35 do -> Xoay Phai (W) [Theo Z]");
    Serial.println(" - Xoay co tay nguoc KDH  >35 do -> Xoay Trai (C) [Theo Z]");
    Serial.println("========================================================\n");
}

void loop() {
    unsigned long now = millis();

    // 1. Đọc góc Euler từ cảm biến BNO055
    BNO::sEulAnalog_t eul = bno.getEul();

    // Kiểm tra mất tín hiệu (tuột dây hoặc lỗi giao tiếp I2C)
    bool bnoSignalLost = (bno.lastOperateStatus != BNO::eStatusOK);

    // ========================================================================
    // NẾU MẤT TÍN HIỆU BNO055 -> NGẮT DỪNG NGAY ĐỘNG CƠ
    // ========================================================================
    if (bnoSignalLost) {
        // Gửi ngay lệnh DỪNG (S) sang Slave để dừng xe lập tức
        if (strcmp(lastSentCmd, "S") != 0 || (now - lastSendTime >= 100)) {
            lastSentCmd = "S";
            lora.sendPacket("S", 40);
            lastSendTime = now;
        }

        // KHÔNG gửi Heartbeat 'H' để sau 650ms Watchdog trên Slave tự động ngắt ENA động cơ!

        if (now - lastLogTime >= 500) {
            lastLogTime = now;
            Serial.println("[FAILSAFE] MAT TIN HIEU BNO055 -> DA GUI LENH DUNG (S) DE NGAT DONG CO!");
        }

        vTaskDelay(pdMS_TO_TICKS(50));
        return; // Dừng vòng lặp tại đây khi mất tín hiệu
    }

    float rawPitch = eul.pitch;
    float rawRoll  = eul.roll;
    float rawYaw   = eul.head; // 0° đến 360°

    // Hoán đổi Pitch và Roll theo phản hồi thực tế:
    // - Roll điều khiển trục TIẾN / LÙI
    // - Pitch điều khiển trục TRÁI / PHẢI
    float fwdBackAngle   = (INVERT_FWD_BACK)   ? -rawRoll  : rawRoll;
    float leftRightAngle = (INVERT_LEFT_RIGHT) ? -rawPitch : rawPitch;

    // Tính độ lệch góc xoay Z so với mốc xuất phát
    float deltaYaw = normalizeAngleDiff(rawYaw, baseYaw);
    if (INVERT_Z_YAW) {
        deltaYaw = -deltaYaw;
    }

    // 2. Kiểm tra trạng thái phẳng (Neutral):
    // CHỈ TỰ ĐỘNG CÂN LẠI MỐC YAW KHI CẢ 3 TRỤC ĐỀU NẰM TRONG VÙNG NGHỈ (< 20°)
    // Tránh tình trạng đang xoay cổ tay mà bị auto-tare xóa mất góc lệch!
    bool isTrulyAtRest = (fabs(fwdBackAngle) < 20.0f && fabs(leftRightAngle) < 20.0f && fabs(deltaYaw) < 18.0f);
    if (isTrulyAtRest) {
        if (neutralStartTime == 0) {
            neutralStartTime = now;
        } else if (now - neutralStartTime >= AUTO_TARE_DELAY_MS) {
            // Khi giữ tay phẳng ổn định > 1.5s, cập nhật lại mốc Yaw mới
            baseYaw = rawYaw;
            deltaYaw = 0.0f;
        }
    } else {
        neutralStartTime = 0; // Đang xoay hoặc nghiêng tay -> Khóa ngay Auto-tare
    }

    // 3. Thuật toán nhận diện cử chỉ bàn tay (Gesture Detection)
    const char* currentCmd = "S";

    // ƯU TIÊN 1: Xoay cổ tay (Trục Z / Yaw >= 35°)
    if (fabs(deltaYaw) >= SETPOINT_Z_YAW) {
        if (deltaYaw > 0) {
            currentCmd = "W"; // Xoay phải (CW)
        } else {
            currentCmd = "C"; // Xoay trái (CCW)
        }
    } 
    // ƯU TIÊN 2: Nghiêng Tiến/Lùi hoặc Trái/Phải (>= 45°)
    else {
        bool fwdActive = (fabs(fwdBackAngle) >= SETPOINT_FWD_BACK);
        bool sideActive = (fabs(leftRightAngle) >= SETPOINT_LEFT_RIGHT);

        if (fwdActive || sideActive) {
            // So sánh xem trục nào nghiêng rõ hơn để lấy hướng ưu tiên
            if (fabs(fwdBackAngle) >= fabs(leftRightAngle)) {
                if (fwdBackAngle < -SETPOINT_FWD_BACK) {
                    currentCmd = "F"; // Tiến
                } else if (fwdBackAngle > SETPOINT_FWD_BACK) {
                    currentCmd = "B"; // Lùi
                }
            } else {
                if (leftRightAngle < -SETPOINT_LEFT_RIGHT) {
                    currentCmd = "L"; // Ngang trái
                } else if (leftRightAngle > SETPOINT_LEFT_RIGHT) {
                    currentCmd = "R"; // Ngang phải
                }
            }
        }
    }

    // 4. Phát lệnh sang Slave qua LoRa E32
    bool isCommandActive = (strcmp(currentCmd, "S") != 0);

    if (strcmp(currentCmd, lastSentCmd) != 0) {
        // Có sự thay đổi cử chỉ: Phát lệnh mới NGAY LẬP TỨC
        lastSentCmd = currentCmd;
        lora.sendPacket(currentCmd, 60);
        lastSendTime = now;
        lastHeartbeatTime = now;

        Serial.printf(">>> [CU CHI MOI] -> %s (TienLui: %.1f | TraiPhai: %.1f | dZ: %.1f)\n", 
                      currentCmd, fwdBackAngle, leftRightAngle, deltaYaw);
    } 
    else if (isCommandActive) {
        // Đang duy trì cử chỉ di chuyển: Phát lặp mỗi 70ms để nuôi Watchdog trên Slave
        if (now - lastSendTime >= SEND_INTERVAL_MS) {
            lora.sendPacket(currentCmd, 60);
            lastSendTime = now;
            lastHeartbeatTime = now;
        }
    } 
    else {
        // Tay đang ở trạng thái Dừng (S): Phát Heartbeat mỗi 200ms báo sóng ổn định
        if (now - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
            lora.sendPacket("H", 40);
            lastHeartbeatTime = now;
        }
    }

    // 5. In Serial log chu kỳ 200ms để theo dõi trực quan
    if (now - lastLogTime >= LOG_INTERVAL_MS) {
        lastLogTime = now;
        Serial.printf("Tien/Lui: %5.1f | Trai/Phai: %5.1f | Xoay Z: %5.1f (dZ: %5.1f) | Lenh: [%s]\n",
                      fwdBackAngle, leftRightAngle, rawYaw, deltaYaw, currentCmd);
    }

    // Chu kỳ quét cử chỉ 20ms (50Hz)
    vTaskDelay(pdMS_TO_TICKS(20));
}
