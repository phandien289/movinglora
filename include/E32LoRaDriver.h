#ifndef E32_LORA_DRIVER_H
#define E32_LORA_DRIVER_H

#include <Arduino.h>

/**
 * ============================================================================
 * Driver điều khiển module LoRa Ebyte E32-433T20D cho ESP32-S3 qua HardwareSerial.
 * ============================================================================
 * - Hoạt động ở Mode 0 (Normal / Transparent Transmission) với baudrate 9600 bps.
 * - Quản lý chân AUX để đồng bộ gửi nhận tin cậy, có timeout tránh treo loop.
 * - Khởi tạo linh hoạt tùy chọn chân UART và chân điều khiển M0, M1, AUX.
 */

class E32LoRaDriver {
private:
    HardwareSerial* _serial;
    uint8_t _rxPin;
    uint8_t _txPin;
    uint8_t _auxPin;
    uint8_t _m0Pin;
    uint8_t _m1Pin;

    static const size_t RX_BUF_SIZE = 64;
    char _rxBuffer[RX_BUF_SIZE];
    size_t _rxIndex;

public:
    E32LoRaDriver(HardwareSerial* serialPort,
                  uint8_t rx,
                  uint8_t tx,
                  uint8_t aux,
                  uint8_t m0,
                  uint8_t m1)
        : _serial(serialPort), _rxPin(rx), _txPin(tx), _auxPin(aux),
          _m0Pin(m0), _m1Pin(m1), _rxIndex(0) {
        memset(_rxBuffer, 0, sizeof(_rxBuffer));
    }

    void begin(uint32_t baudRate = 9600, bool enableHighSpeed = true) {
        // Cấu hình chân AUX với điện trở kéo lên nội
        pinMode(_auxPin, INPUT_PULLUP);

        // Chân điều khiển chế độ M0, M1
        pinMode(_m0Pin, OUTPUT);
        pinMode(_m1Pin, OUTPUT);

        // Khởi tạo cổng HardwareSerial của ESP32-S3
        _serial->begin(baudRate, SERIAL_8N1, _rxPin, _txPin);

        if (enableHighSpeed) {
            // Cấu hình tốc độ truyền không dây lên 19.2 kbps (giảm thời gian phát từ 60ms xuống ~8ms)
            setHighSpeedAirRate(baudRate);
        } else {
            // Kéo M0, M1 xuống LOW để chọn Mode 0 (Normal / Transparent mode)
            digitalWrite(_m0Pin, LOW);
            digitalWrite(_m1Pin, LOW);
            waitAuxFree(300);
        }
    }

    /**
     * Tự động cấu hình E32 lên Air Data Rate = 19.2 kbps (tối đa của E32).
     * Chuyển tạm thời sang Mode 3 (Sleep / Config) để nạp tham số SPED rồi quay về Mode 0.
     */
    bool setHighSpeedAirRate(uint32_t uartBaud = 9600) {
        // 1. Chuyển sang Mode 3 (Sleep/Program): M0 = HIGH, M1 = HIGH
        digitalWrite(_m0Pin, HIGH);
        digitalWrite(_m1Pin, HIGH);
        waitAuxFree(200);
        delay(10);

        // Xóa sạch bộ đệm UART trước khi gửi lệnh
        while (_serial->available() > 0) {
            _serial->read();
        }

        // Cấu hình E32:
        // Byte 0: 0xC0 (Lưu tham số vào EEPROM không mất khi mất nguồn)
        // Byte 1: 0x00 (ADDH)
        // Byte 2: 0x00 (ADDL)
        // Byte 3: 0x1D (SPED: 8N1=00, 9600=011, Air Data Rate 19.2 kbps=101 -> 0b00011101 = 0x1D)
        // Byte 4: 0x17 (Kênh 23: 410M + 23 = 433.0 MHz)
        // Byte 5: 0x44 (Option: Transparent=0, Push-pull=1, 250ms wakeup=000, FEC on=1, 20dBm=00)
        uint8_t cfgCmd[6] = {0xC0, 0x00, 0x00, 0x1D, 0x17, 0x44};
        _serial->write(cfgCmd, 6);
        _serial->flush();

        // Chờ module phản hồi 6 byte xác nhận
        uint32_t start = millis();
        int bytesRead = 0;
        uint8_t response[6] = {0};
        while ((millis() - start < 300) && bytesRead < 6) {
            if (_serial->available() > 0) {
                response[bytesRead++] = (uint8_t)_serial->read();
            } else {
                delay(2);
            }
        }

        waitAuxFree(200);
        delay(10);

        // 2. Chuyển trở lại Mode 0 (Normal): M0 = LOW, M1 = LOW
        digitalWrite(_m0Pin, LOW);
        digitalWrite(_m1Pin, LOW);
        waitAuxFree(200);
        delay(20);

        bool ok = (bytesRead == 6 && response[0] == 0xC0);
        return ok;
    }

    bool isReady() {
        return (digitalRead(_auxPin) == HIGH);
    }

    void waitAuxFree(uint32_t timeoutMs = 200) {
        uint32_t start = millis();
        // Chờ chân AUX lên HIGH (rỗi), tối đa timeoutMs để tránh kẹt
        while (digitalRead(_auxPin) == LOW && (millis() - start < timeoutMs)) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        delayMicroseconds(200);
    }

    /**
     * Gửi gói tin có kiểm soát luồng phần cứng qua chân AUX.
     * Chờ module E32 rỗi (AUX = HIGH) tối đa timeoutMs để tránh xung đột buffer.
     * Khi hết timeout (hoặc chân AUX không nối), vẫn đẩy dữ liệu ra UART để bảo đảm không đứt kết nối.
     */
    bool sendPacket(const char* packet, uint32_t timeoutMs = 60) {
        if (!packet) return false;

        // Chờ module E32 rỗi (AUX = HIGH) tối đa timeoutMs
        uint32_t start = millis();
        while (digitalRead(_auxPin) == LOW && (millis() - start < timeoutMs)) {
            delayMicroseconds(200);
        }

        // Module đã rỗi hoặc hết thời gian chờ -> Đẩy gói tin ra HardwareSerial
        _serial->println(packet);
        _serial->flush(); // Đẩy toàn bộ dữ liệu từ ESP32-S3 sang E32
        return true;
    }

    /**
     * Gửi gói tin quan trọng (như lệnh DỪNG STOP) và chờ module phát sóng xong.
     */
    bool sendPacketWaitDone(const char* packet, uint32_t timeoutMs = 100) {
        if (!sendPacket(packet, timeoutMs)) return false;
        
        // Đợi một khoảng ngắn cho AUX chuyển sang LOW (bắt đầu phát)
        delayMicroseconds(500);

        // Đợi cho đến khi phát xong hoàn toàn (AUX trở lại HIGH)
        uint32_t start = millis();
        while (digitalRead(_auxPin) == LOW && (millis() - start < timeoutMs)) {
            delayMicroseconds(200);
        }
        return true;
    }

    /**
     * Đọc chuỗi ký tự nhận được từ LoRa.
     * Tách gói tin hoàn chỉnh khi gặp ký tự xuống dòng '\n' hoặc '\r'.
     */
    bool readPacket(char* outMsg, size_t maxLen) {
        while (_serial->available() > 0) {
            char c = (char)_serial->read();

            if (c == '\n' || c == '\r') {
                if (_rxIndex > 0) {
                    _rxBuffer[_rxIndex] = '\0';
                    strncpy(outMsg, _rxBuffer, maxLen - 1);
                    outMsg[maxLen - 1] = '\0';
                    _rxIndex = 0;
                    return true;
                }
            } else {
                if (_rxIndex < (RX_BUF_SIZE - 1)) {
                    _rxBuffer[_rxIndex++] = c;
                } else {
                    _rxIndex = 0; // Tràn buffer thì reset
                }
            }
        }
        return false;
    }
};

#endif // E32_LORA_DRIVER_H
