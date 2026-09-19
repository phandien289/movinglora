#include <Arduino.h>
#include "TM1638Driver.h"
#include "E32LoRaDriver.h"

/**
 * ============================================================================
 * ESP32-S3 MASTER NODE - BỘ ĐIỀU KHIỂN ROBOT OMNI QUA LORA E32 & TM1638
 * ============================================================================
 * - Hiển thị trên 8 LED 7 đoạn TM1638:
 *     + 4 LED bên trái (Digits 0..3): Hướng di chuyển ("FOr ", "bACK", "LEFt", "rIGt", "t-LF", "t-rt", "StOP")
 *     + 4 LED bên phải (Digits 4..7): Tốc độ cài đặt (Ví dụ: "S 75", "S 80", "S100")
 * - Phím bấm S1 -> S8:
 *     + S1: Tiến [F]       | S2: Lùi [B]
 *     + S3: Ngang trái [L] | S4: Ngang phải [R]
 *     + S5: Xoay trái [C]  | S6: Xoay phải [W]
 *     + S7: TĂNG TỐC ĐỘ (+5 RPM)
 *     + S8: GIẢM TỐC ĐỘ (-5 RPM)
 * ============================================================================
 */

// Cấu hình chân TM1638 trên Master (ESP32-S3)
#define TM_STB_PIN   4
#define TM_CLK_PIN   5
#define TM_DIO_PIN   6

// Cấu hình chân LoRa Ebyte E32-433T20D trên Master (ESP32-S3)
#define LORA_RX_PIN  16 // ESP32-S3 RX1 (nối TXD của LoRa E32)
#define LORA_TX_PIN  17 // ESP32-S3 TX1 (nối RXD của LoRa E32)
#define LORA_AUX_PIN 18 // ESP32-S3 Input (nối AUX của LoRa E32)
#define LORA_M0_PIN  7  // ESP32-S3 Output (nối M0 của LoRa E32)
#define LORA_M1_PIN  15 // ESP32-S3 Output (nối M1 của LoRa E32)

TM1638Driver tm1638(TM_STB_PIN, TM_CLK_PIN, TM_DIO_PIN);
E32LoRaDriver lora(&Serial1, LORA_RX_PIN, LORA_TX_PIN, LORA_AUX_PIN, LORA_M0_PIN, LORA_M1_PIN);

// Biến quản lý tốc độ xe (mặc định 75 RPM, khoảng 25 - 150 RPM)
int currentSpeed = 75;
bool s7Handled = false; // Chống lặp phím S7 (Tăng tốc)
bool s8Handled = false; // Chống lặp phím S8 (Giảm tốc)

// Biến quản lý trạng thái chuyển động & chống rung
uint8_t activeMoveKey = 0;           // Phím di chuyển S1..S6 hiện tại
const uint8_t* activeDirSegs = DIR_READY; // Con trỏ mảng 4 ký tự hiển thị hướng hiện tại
unsigned long lastReleaseTimer = 0;  // Mốc thời gian khi phát hiện phím chuyển động = 0
const unsigned long RELEASE_DEBOUNCE_MS = 25; // Cần giữ phím = 0 liên tục 25ms mới tính là nhả

// Biến quản lý thời gian phát lệnh
unsigned long lastSendTime = 0;
unsigned long lastHeartbeatTime = 0;
const unsigned long SEND_INTERVAL_MS = 75;       // Chu kỳ phát lệnh khi giữ phím (75ms)
const unsigned long HEARTBEAT_INTERVAL_MS = 200; // Chu kỳ phát heartbeat khi đứng yên (200ms)
bool isMoving = false;

// Hàm phụ trợ lấy lệnh, vị trí LED và mảng 4 ký tự hướng từ phím bấm S1..S6
const char* getMoveCommand(uint8_t moveKey, int &ledIdx, const uint8_t* &dirSegs) {
    if (moveKey & 0x01) { ledIdx = 0; dirSegs = DIR_FWD;   return "F"; } // S1: Tiến
    if (moveKey & 0x02) { ledIdx = 1; dirSegs = DIR_BACK;  return "B"; } // S2: Lùi
    if (moveKey & 0x04) { ledIdx = 2; dirSegs = DIR_LEFT;  return "L"; } // S3: Ngang trái
    if (moveKey & 0x08) { ledIdx = 3; dirSegs = DIR_RIGHT; return "R"; } // S4: Ngang phải
    if (moveKey & 0x10) { ledIdx = 4; dirSegs = DIR_ROT_L; return "C"; } // S5: Xoay trái
    if (moveKey & 0x20) { ledIdx = 5; dirSegs = DIR_ROT_R; return "W"; } // S6: Xoay phải
    ledIdx = -1;
    dirSegs = DIR_STOP;
    return nullptr;
}

void setup() {
    Serial.begin(115200);

    // Timeout tránh treo khi chạy nguồn ngoài/pin (User Global Rule)
    unsigned long start = millis();
    while (!Serial && (millis() - start < 3000)) { 
        delay(10); 
    }

    Serial.println("\n========================================================");
    Serial.println(" ESP32-S3 MASTER: CONTROLLER ROBOT OMNI (DUAL DISPLAY) ");
    Serial.println("========================================================");

    // 1. Khởi tạo TM1638
    tm1638.begin(7); // Độ sáng tối đa (7)
    
    // Hiệu ứng khởi động kiểm tra màn hình và 8 LED
    for (int i = 0; i < 8; i++) {
        tm1638.setLed(i, true);
        delay(20);
    }
    delay(60);
    tm1638.clearAllLeds();

    // Hiển thị khởi đầu: 4 LED trái "rdY ", 4 LED phải "S 75"
    activeDirSegs = DIR_READY;
    tm1638.showReady(currentSpeed);
    Serial.printf("[OK] TM1638 san sang (Trai: 'rdY ' | Phai: 'S %d')\n", currentSpeed);

    // 2. Khởi tạo LoRa E32-433T20D với Air Data Rate 19.2 kbps
    lora.begin(9600, true);
    Serial.println("[OK] LoRa E32-433T20D san sang (Air Data Rate: 19.2 kbps)");

    // Đồng bộ tốc độ khởi đầu sang Slave
    char spdCmd[16];
    snprintf(spdCmd, sizeof(spdCmd), "V%d", currentSpeed);
    lora.sendPacket(spdCmd, 60);

    Serial.println("--------------------------------------------------------");
    Serial.println("HUONG DAN PHIM BAM:");
    Serial.println(" - S1: Tien [F]       | S2: Lui [B]");
    Serial.println(" - S3: Ngang trai [L] | S4: Ngang phai [R]");
    Serial.println(" - S5: Xoay trai [C]  | S6: Xoay phai [W]");
    Serial.println(" - S7: TANG TOC DO (+5 RPM)");
    Serial.println(" - S8: GIAM TOC DO (-5 RPM)");
    Serial.println(" - Tha phim: Dung xe [S]");
    Serial.println("========================================================");
}

void loop() {
    uint8_t rawKey = tm1638.readButtons();
    unsigned long now = millis();

    // Tách riêng phím di chuyển (S1..S6: bit 0..5) và phím tốc độ (S7..S8: bit 6..7)
    uint8_t moveKey  = rawKey & 0x3F; // Bit 0..5 (S1..S6)
    uint8_t speedKey = rawKey & 0xC0; // Bit 6..7 (S7..S8)

    // ========================================================================
    // 1. XỬ LÝ PHÍM TỐC ĐỘ: S7 (TĂNG TỐC) & S8 (GIẢM TỐC)
    // ========================================================================
    if (speedKey & 0x40) { // Phím S7: TĂNG TỐC ĐỘ
        if (!s7Handled) {
            s7Handled = true;
            if (currentSpeed < 150) {
                currentSpeed += 5; // Tăng 5 RPM mỗi lần bấm
                char spdPacket[16];
                snprintf(spdPacket, sizeof(spdPacket), "V%d", currentSpeed);
                lora.sendPacket(spdPacket, 60);

                // Cập nhật 4 LED phải hiển thị tốc độ mới
                tm1638.updateDirectionAndSpeed(activeDirSegs, currentSpeed);
                tm1638.setLed(6, true); // Bật LED trên nút S7 phản hồi thị giác
                Serial.printf("[MASTER] S7 -> TANG TOC: %d RPM\n", currentSpeed);
            }
        }
    } else {
        if (s7Handled) {
            s7Handled = false;
            tm1638.setLed(6, false); // Tắt LED khi nhả nút S7
        }
    }

    if (speedKey & 0x80) { // Phím S8: GIẢM TỐC ĐỘ
        if (!s8Handled) {
            s8Handled = true;
            if (currentSpeed > 25) {
                currentSpeed -= 5; // Giảm 5 RPM mỗi lần bấm
                char spdPacket[16];
                snprintf(spdPacket, sizeof(spdPacket), "V%d", currentSpeed);
                lora.sendPacket(spdPacket, 60);

                // Cập nhật 4 LED phải hiển thị tốc độ mới
                tm1638.updateDirectionAndSpeed(activeDirSegs, currentSpeed);
                tm1638.setLed(7, true); // Bật LED trên nút S8 phản hồi thị giác
                Serial.printf("[MASTER] S8 -> GIAM TOC: %d RPM\n", currentSpeed);
            }
        }
    } else {
        if (s8Handled) {
            s8Handled = false;
            tm1638.setLed(7, false); // Tắt LED khi nhả nút S8
        }
    }

    // ========================================================================
    // 2. XỬ LÝ PHÍM DI CHUYỂN: S1..S6 (TIẾN, LÙI, TRÁI, PHẢI, XOAY)
    // ========================================================================
    if (moveKey != 0) {
        lastReleaseTimer = now;

        if (moveKey != activeMoveKey) {
            // === BẤM PHÍM DI CHUYỂN MỚI HOẶC ĐỔI HƯỚNG (0ms DELAY) ===
            activeMoveKey = moveKey;
            int ledIdx = -1;
            const uint8_t* dirSegs = nullptr;
            const char* cmd = getMoveCommand(activeMoveKey, ledIdx, dirSegs);

            if (cmd != nullptr) {
                activeDirSegs = dirSegs;

                // Cập nhật 4 LED 7 đoạn bên trái (Hướng) và giữ nguyên tốc độ bên phải
                tm1638.updateDirectionAndSpeed(activeDirSegs, currentSpeed);
                tm1638.setSingleLed(ledIdx);

                // Phát sóng tức thì ra LoRa
                lora.sendPacket(cmd, 60);
                lastSendTime = now;
                lastHeartbeatTime = now;
                isMoving = true;

                Serial.printf("[MASTER] HUONG: %s | TOC DO: %d RPM\n", cmd, currentSpeed);
            }
        } else {
            // === ĐANG NHẤN GIỮ PHÍM DI CHUYỂN (LẶP MỖI 75ms NUÔI WATCHDOG) ===
            if (now - lastSendTime >= SEND_INTERVAL_MS) {
                int ledIdx = -1;
                const uint8_t* dirSegs = nullptr;
                const char* cmd = getMoveCommand(activeMoveKey, ledIdx, dirSegs);
                if (cmd != nullptr) {
                    lora.sendPacket(cmd, 60);
                    lastSendTime = now;
                    lastHeartbeatTime = now;
                }
            }
        }
    } else {
        // === KHÔNG CÓ PHÍM DI CHUYỂN NÀO ĐƯỢC NHẤN ===
        if (activeMoveKey != 0) {
            // Chống rung nhả phím: chỉ khi phím = 0 liên tục >= 25ms mới tính là thả thật
            if (now - lastReleaseTimer >= RELEASE_DEBOUNCE_MS) {
                // === XÁC NHẬN NHẢ TAY -> DỪNG PHANH XE ===
                Serial.println("[MASTER] NHA PHIM -> DUNG PHANH XE (S)");
                activeMoveKey = 0;
                isMoving = false;
                activeDirSegs = DIR_STOP;

                // Cập nhật 4 LED trái thành "StOP", 4 LED phải giữ nguyên tốc độ
                tm1638.showStop(currentSpeed);
                tm1638.clearMoveLeds();

                // Gửi lệnh DỪNG "S" đồng bộ qua LoRa
                lora.sendPacketWaitDone("S", 80);
                lora.sendPacketWaitDone("S", 80);

                lastSendTime = now;
                lastHeartbeatTime = now;
            }
        } else {
            // === ĐỨNG YÊN (IDLE) -> PHÁT HEARTBEAT MỖI 200ms ===
            if (now - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
                lora.sendPacket("H", 40);
                lastHeartbeatTime = now;
            }
        }
    }

    // Quét phím chu kỳ 2ms
    vTaskDelay(pdMS_TO_TICKS(2));
}

