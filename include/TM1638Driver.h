#ifndef TM1638_DRIVER_H
#define TM1638_DRIVER_H

#include <Arduino.h>

/**
 * ============================================================================
 * Driver điều khiển module TM1638 (8 LED 7 đoạn + 8 LED đơn + 8 Phím bấm S1-S8)
 * ============================================================================
 * - Tối ưu cho ESP32-S3 với xung nhịp 240MHz (chèn delayMicroseconds 4us).
 * - Quét chính xác 8 phím S1 đến S8 với định dạng bitmask:
 *     S1: 0x01 | S2: 0x02 | S3: 0x04 | S4: 0x08 | S5: 0x10 | S6: 0x20 | S7: 0x40 | S8: 0x80
 * - Hỗ trợ các kiểu hiển thị chuyển động trực quan cho Robot Omni:
 *     "  FOr   ", "  bACK  ", "  LEFt  ", "  rIGt  ", "  t-LF  ", "  t-rt  ", "  StOP  ", "  rEAdY "
 */

// Bảng mã các ký tự 7 thanh thông dụng (A-B-C-D-E-F-G-DP)
#define SEG_BLANK 0x00
#define SEG_DASH  0x40 // Thanh ngang giữa G
#define SEG_U     0x3E // B, C, D, E, F
#define SEG_P     0x73 // A, B, E, F, G
#define SEG_D     0x5E // B, C, D, E, G (chữ d)
#define SEG_O     0x5C // C, D, E, G    (chữ o)
#define SEG_N     0x54 // C, E, G       (chữ n)
#define SEG_L     0x38 // D, E, F
#define SEG_E     0x79 // A, D, E, F, G
#define SEG_F     0x71 // A, E, F, G
#define SEG_T     0x78 // D, E, F, G    (chữ t)
#define SEG_R     0x50 // E, G          (chữ r)
#define SEG_I     0x06 // B, C          (chữ I hoa)
#define SEG_G     0x6F // A, B, C, D, F, G (chữ 9/g)
#define SEG_S     0x6D // A, C, D, F, G (chữ S)
#define SEG_A     0x77 // A, B, C, E, F, G (chữ A)
#define SEG_B     0x7C // C, D, E, F, G (chữ b)
#define SEG_C     0x39 // A, D, E, F (chữ C)
#define SEG_K     0x75 // A, C, E, F, G (chữ K dạng 7 đoạn)
#define SEG_Y     0x6E // B, C, D, F, G (chữ Y)

// Bảng mã các số 0 - 9 cho LED 7 đoạn
static const uint8_t DIGIT_MAP[10] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};

// Mảng 4 ký tự hiển thị hướng (4 LED 7 đoạn bên trái: Digits 0, 1, 2, 3)
static const uint8_t DIR_STOP[4]  = { SEG_S, SEG_T, SEG_O, SEG_P };       // "StOP"
static const uint8_t DIR_READY[4] = { SEG_R, SEG_D, SEG_Y, SEG_BLANK };   // "rdY "
static const uint8_t DIR_FWD[4]   = { SEG_F, SEG_O, SEG_R, SEG_BLANK };   // "FOr "
static const uint8_t DIR_BACK[4]  = { SEG_B, SEG_A, SEG_C, SEG_K };       // "bACK"
static const uint8_t DIR_LEFT[4]  = { SEG_L, SEG_E, SEG_F, SEG_T };       // "LEFt"
static const uint8_t DIR_RIGHT[4] = { SEG_R, SEG_I, SEG_G, SEG_T };       // "rIGt"
static const uint8_t DIR_ROT_L[4] = { SEG_T, SEG_DASH, SEG_L, SEG_F };   // "t-LF"
static const uint8_t DIR_ROT_R[4] = { SEG_T, SEG_DASH, SEG_R, SEG_T };   // "t-rt"

class TM1638Driver {
private:
    uint8_t _stbPin;
    uint8_t _clkPin;
    uint8_t _dioPin;
    uint8_t _lastReportedKey;
    uint8_t _digits[8];
    uint8_t _leds[8];

    void sendCommand(uint8_t cmd) {
        digitalWrite(_stbPin, LOW);
        writeByte(cmd);
        digitalWrite(_stbPin, HIGH);
    }

    void writeByte(uint8_t data) {
        pinMode(_dioPin, OUTPUT);
        for (uint8_t i = 0; i < 8; i++) {
            digitalWrite(_clkPin, LOW);
            digitalWrite(_dioPin, (data & (1 << i)) ? HIGH : LOW);
            delayMicroseconds(4);
            digitalWrite(_clkPin, HIGH);
            delayMicroseconds(4);
        }
    }

    uint8_t readByte() {
        uint8_t data = 0;
        for (uint8_t i = 0; i < 8; i++) {
            digitalWrite(_clkPin, LOW);
            delayMicroseconds(4);
            if (digitalRead(_dioPin) == HIGH) {
                data |= (1 << i);
            }
            digitalWrite(_clkPin, HIGH);
            delayMicroseconds(4);
        }
        return data;
    }

public:
    TM1638Driver(uint8_t stb, uint8_t clk, uint8_t dio)
        : _stbPin(stb), _clkPin(clk), _dioPin(dio), _lastReportedKey(0) {
        memset(_digits, 0, sizeof(_digits));
        memset(_leds, 0, sizeof(_leds));
    }

    void begin(uint8_t brightness = 7) {
        pinMode(_stbPin, OUTPUT);
        pinMode(_clkPin, OUTPUT);
        pinMode(_dioPin, OUTPUT);
        digitalWrite(_stbPin, HIGH);
        digitalWrite(_clkPin, HIGH);

        if (brightness > 7) brightness = 7;
        sendCommand(0x88 | brightness);
        clear();
    }

    void clear() {
        memset(_digits, 0, sizeof(_digits));
        memset(_leds, 0, sizeof(_leds));
        sendCommand(0x40); // Auto-increment address
        digitalWrite(_stbPin, LOW);
        writeByte(0xC0);   // Bắt đầu từ địa chỉ 0
        for (uint8_t i = 0; i < 16; i++) {
            writeByte(0x00);
        }
        digitalWrite(_stbPin, HIGH);
    }

    // Gửi toàn bộ 8 thanh 7 đoạn và 8 đèn LED đơn ra TM1638
    void flushDisplay() {
        sendCommand(0x40); // Auto-increment
        digitalWrite(_stbPin, LOW);
        writeByte(0xC0);
        for (uint8_t i = 0; i < 8; i++) {
            writeByte(_digits[i]); // LED 7 đoạn
            writeByte(_leds[i]);   // LED đơn tương ứng
        }
        digitalWrite(_stbPin, HIGH);
    }

    /**
     * Cập nhật đồng thời:
     * - 4 LED 7 đoạn bên trái (Digits 0..3): Hướng di chuyển (DIR_FWD, DIR_STOP, v.v.)
     * - 4 LED 7 đoạn bên phải (Digits 4..7): Tốc độ (Ví dụ: "S 75", "S100")
     */
    void updateDirectionAndSpeed(const uint8_t dirSegs[4], int speedRpm) {
        // 4 LED bên trái: Hướng
        if (dirSegs != nullptr) {
            for (int i = 0; i < 4; i++) {
                _digits[i] = dirSegs[i];
            }
        }

        // 4 LED bên phải: Tốc độ
        if (speedRpm < 0) speedRpm = 0;
        if (speedRpm > 999) speedRpm = 999;

        _digits[4] = SEG_S; // Ký tự 'S' (Speed)
        if (speedRpm >= 100) {
            _digits[5] = DIGIT_MAP[(speedRpm / 100) % 10];
            _digits[6] = DIGIT_MAP[(speedRpm / 10) % 10];
            _digits[7] = DIGIT_MAP[speedRpm % 10];
        } else {
            _digits[5] = SEG_BLANK;
            _digits[6] = DIGIT_MAP[(speedRpm / 10) % 10];
            _digits[7] = DIGIT_MAP[speedRpm % 10];
        }

        flushDisplay();
    }

    // Bật/tắt đèn LED đơn (ledIndex 0 -> 7, state: true/false)
    void setLed(uint8_t ledIndex, bool state) {
        if (ledIndex > 7) return;
        _leds[ledIndex] = state ? 0x01 : 0x00;
        sendCommand(0x44); // Fixed address
        digitalWrite(_stbPin, LOW);
        writeByte(0xC1 + (ledIndex * 2));
        writeByte(_leds[ledIndex]);
        digitalWrite(_stbPin, HIGH);
    }

    // Bật chỉ 1 đèn LED chỉ hướng (ledIndex 0..5), giữ nguyên trạng thái LED 6, 7 của phím tốc độ
    void setSingleLed(int ledIndex) {
        for (int i = 0; i < 6; i++) {
            _leds[i] = (i == ledIndex) ? 0x01 : 0x00;
        }
        flushDisplay();
    }

    // Tắt toàn bộ 6 đèn LED đơn chỉ hướng (giữ nguyên LED 6, 7 nếu có)
    void clearMoveLeds() {
        for (uint8_t i = 0; i < 6; i++) {
            _leds[i] = 0x00;
        }
        flushDisplay();
    }

    // Tắt tất cả 8 đèn LED đơn
    void clearAllLeds() {
        memset(_leds, 0, sizeof(_leds));
        flushDisplay();
    }

    // Các hàm trợ giúp hiển thị hướng kèm tốc độ:
    void showForward(int speed)     { updateDirectionAndSpeed(DIR_FWD, speed); }
    void showBackward(int speed)    { updateDirectionAndSpeed(DIR_BACK, speed); }
    void showLeft(int speed)        { updateDirectionAndSpeed(DIR_LEFT, speed); }
    void showRight(int speed)       { updateDirectionAndSpeed(DIR_RIGHT, speed); }
    void showRotateLeft(int speed)  { updateDirectionAndSpeed(DIR_ROT_L, speed); }
    void showRotateRight(int speed) { updateDirectionAndSpeed(DIR_ROT_R, speed); }
    void showStop(int speed)        { updateDirectionAndSpeed(DIR_STOP, speed); }
    void showReady(int speed)       { updateDirectionAndSpeed(DIR_READY, speed); }

    /**
     * Quét trực tiếp 4 byte trạng thái phím từ TM1638:
     * - S1..S4 nằm ở byte 0..3 (bit 0)
     * - S5..S8 nằm ở byte 0..3 (bit 4)
     * Trả về bitmask (S1 = bit 0, S6 = bit 5, v.v.)
     */
    uint8_t readButtons() {
        digitalWrite(_stbPin, LOW);
        writeByte(0x42); // Read key-scan data
        pinMode(_dioPin, INPUT_PULLUP);
        delayMicroseconds(5);

        uint8_t b[4];
        for (uint8_t i = 0; i < 4; i++) {
            b[i] = readByte();
        }
        digitalWrite(_stbPin, HIGH);

        uint8_t keys = 0;
        for (uint8_t i = 0; i < 4; i++) {
            if (b[i] & 0x01) keys |= (1 << i);       // S1..S4 (bit 0..3)
            if (b[i] & 0x10) keys |= (1 << (i + 4)); // S5..S8 (bit 4..7)
        }
        return keys;
    }

    /**
     * Đọc trạng thái liên tục đã qua bộ lọc chống rung (debounced)
     * Rất quan trọng khi điều khiển xe (nhấn giữ để chạy, thả tay là dừng).
     */
    uint8_t readButtonsContinuous() {
        uint8_t k1 = readButtons();
        if (k1 == 0) return 0;
        delayMicroseconds(800);
        uint8_t k2 = readButtons();
        return (k1 == k2) ? k1 : 0;
    }
};

#endif // TM1638_DRIVER_H
