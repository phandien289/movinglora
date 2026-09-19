# Hệ Thống Điều Khiển Robot 3 Bánh Omni Qua Sóng LoRa E32 & Module TM1638 (ESP32-S3)

Dự án điều khiển robot di động 3 bánh Omni toàn hướng (Omnidirectional Robot) thời gian thực sử dụng vi điều khiển **ESP32-S3**, module truyền nhận vô tuyến tầm xa **LoRa Ebyte E32-433T20D** và bàn phím kiêm hiển thị **TM1638**.

Hệ thống được thiết kế theo kiến trúc **Master - Slave**, tối ưu phản hồi siêu tốc (< 35ms), tích hợp tính năng điều khiển tốc độ linh hoạt và cơ chế tự động ngắt động cơ an toàn (Failsafe Auto-Cutoff) khi mất sóng.

---

## 1. CÁC TÍNH NĂNG NỔI BẬT

* **Phản hồi thời gian thực siêu tốc (< 35ms):**
  * Tốc độ truyền sóng vô tuyến (Air Data Rate) được nâng lên mức tối đa **19.2 kbps** (rút ngắn thời gian phát từ ~60ms xuống còn ~3ms – 8ms/gói).
  * Xử lý gói tin rút gọn 1–4 bytes (`F`, `B`, `L`, `R`, `C`, `W`, `S`, `H`, `V80`).
  * Phản hồi cạnh bấm phím tức thì **0ms delay** (bấm nút là xe chạy ngay, không có độ trễ trù trừ).
  * Bộ lọc chống rung tiếp điểm thông minh **25ms** khi nhả phím (loại bỏ hiện tượng nhả phím ảo gây giật cục xe).
* **Hiển thị kép chuyên biệt trên 8 LED 7 đoạn (TM1638):**
  * **4 LED 7 đoạn bên trái:** Hiển thị trực quan hướng di chuyển (`FOr `, `bACK`, `LEFt`, `rIGt`, `t-LF`, `t-rt`, `StOP`, `rdY `).
  * **4 LED 7 đoạn bên phải:** Hiển thị tốc độ cài đặt theo thời gian thực (định dạng `S 75`, `S 80`, `S100` RPM).
* **Điều khiển tốc độ động (On-the-fly Dynamic Speed Control):**
  * Phím **S7**: Tăng tốc độ (+5 RPM mỗi lần nhấn).
  * Phím **S8**: Giảm tốc độ (-5 RPM mỗi lần nhấn).
  * Dải điều chỉnh: **25 – 150 RPM** (mặc định khởi động 75 RPM).
  * Tách biệt hoàn toàn luồng xử lý phím: có thể vừa nhấn giữ chạy xe vừa bấm S7/S8 tăng giảm tốc mà xe không bị dừng hay giật.
* **Cơ chế tự động ngắt động cơ khi mất kết nối (Failsafe Auto-Cutoff):**
  * Master định kỳ phát nhịp tim (Heartbeat `"H"`) mỗi 200ms khi xe đứng yên.
  * Nếu Slave không nhận được bất kỳ tín hiệu nào quá **650ms** (do mất nguồn Master, ngoài vùng phủ sóng, nhiễu tần số):
    * Lập tức **triệt tiêu toàn bộ xung bước về 0** (dừng ngay, không trôi quán tính).
    * Kéo chân phần cứng **`MF-` (ENA)** để ngắt Driver, động cơ mất lực kéo hoàn toàn, xe dừng hẳn và không chạy nữa.
    * Đèn LED RGB trên ESP32-S3 Slave **nhấp nháy ĐỎ** chu kỳ 250ms cảnh báo.
    * Tự động phục hồi và bật lại Driver khi có sóng kết nối trở lại.
* **Kiến trúc FreeRTOS Dual-Core (ESP32-S3 @ 240MHz):**
  * **Core 0:** Giao tiếp vô tuyến LoRa E32, quét đệm UART chu kỳ 1ms, giải mã lệnh, quản lý nhịp tim.
  * **Core 1:** Giải thuật động học 3 bánh Omni, nội suy đường cong gia tốc S-curve dứt khoát (~35ms đạt full tốc, ~30ms hãm dừng), phát xung thời gian thực chính xác đến từng microsecond ($\mu\text{s}$).

---

## 2. HƯỚNG DẪN ĐẤU NỐI PHẦN CỨNG (PINOUT)

### 2.1. Node Master (Tay cầm điều khiển TM1638 + LoRa E32)

| Tên Thiết Bị | Chân Module | Chân ESP32-S3 | Chức Năng / Ghi Chú |
| :--- | :--- | :--- | :--- |
| **LoRa E32-433T20D** | TXD | **GPIO 1** | RX1 của ESP32-S3 (Nhận dữ liệu) |
| | RXD | **GPIO 2** | TX1 của ESP32-S3 (Gửi dữ liệu) |
| | AUX | **GPIO 10** | Báo trạng thái bận/rảnh của LoRa |
| | M0 | **GPIO 11** | Chuyển chế độ hoạt động |
| | M1 | **GPIO 12** | Chuyển chế độ hoạt động |
| | VCC / GND | 5V / GND | Nguồn nuôi module (nên dùng nguồn ổn định) |
| **Bàn phím TM1638** | STB | **GPIO 4** | Chân chọn chip (Strobe) |
| | CLK | **GPIO 5** | Chân xung đồng hồ (Clock) |
| | DIO | **GPIO 6** | Chân truyền nhận dữ liệu (Data I/O) |
| | VCC / GND | 3.3V hoặc 5V / GND | Nguồn nuôi module |

---

### 2.2. Node Slave (Robot 3 bánh Omni + LoRa E32)

#### A. Kết nối Module LoRa E32
| Chân LoRa E32 | Chân ESP32-S3 | Chức Năng |
| :--- | :--- | :--- |
| **TXD** | **GPIO 1** | RX1 của ESP32-S3 |
| **RXD** | **GPIO 2** | TX1 của ESP32-S3 |
| **AUX** | **GPIO 10** | Giám sát trạng thái buffer vô tuyến |
| **M0** | **GPIO 11** | Chuyển chế độ |
| **M1** | **GPIO 12** | Chuyển chế độ |
| **VCC / GND** | 5V / GND | Nguồn nuôi module |

#### B. Kết nối 3 Driver Động Cơ Bước (Ví dụ: DM542 / TB6600)
> **Lưu ý:** Các chân điều khiển đấu theo kiểu **Common Anode (Dương chung)**: Các chân `PU+`, `DR+`, `MF+` nối chung vào nguồn logic (5V hoặc 3.3V). Các chân âm `PU-`, `DR-`, `MF-` đấu vào GPIO của ESP32-S3:

| Động Cơ | Vị Trí / Góc | PU- (Pulse / Step) | DR- (Direction) | MF- (Enable / Ngắt Driver) |
| :--- | :--- | :--- | :--- | :--- |
| **Motor 1** | Bánh 1 (90°) | **GPIO 4** | **GPIO 5** | **GPIO 6** |
| **Motor 2** | Bánh 2 (210°) | **GPIO 7** | **GPIO 15** | **GPIO 16** |
| **Motor 3** | Bánh 3 (330°) | **GPIO 17** | **GPIO 18** | **GPIO 8** |

* **Đèn LED RGB WS2812 tích hợp:** Sử dụng chân **GPIO 48** (hoặc chân `RGB_BUILTIN` mặc định của bo ESP32-S3).

---

## 3. HƯỚNG DẪN HIỂN THỊ LED VÀ BÀN PHÍM

### 3.1. Hiển thị 8 LED 7 đoạn trên Master (TM1638)

```
       [ 4 LED BÊN TRÁI ]              [ 4 LED BÊN PHẢI ]
    ┌───┬───┬───┬───┐               ┌───┬───┬───┬───┐
    │ F │ O │ r │   │               │ S │   │ 7 │ 5 │
    └───┴───┴───┴───┘               └───┴───┴───┴───┘
   (Hướng di chuyển)                (Tốc độ cài đặt: RPM)
```

| Trạng Thái Thao Tác | 4 LED Trái (Digits 0..3) | 4 LED Phải (Digits 4..7) |
| :--- | :--- | :--- |
| **Khởi động / Sẵn sàng** | `rdY ` (Ready) | `S 75` (Tốc độ mặc định 75 RPM) |
| **Đứng yên / Nhả phím** | `StOP` | `S 75` (hoặc tốc độ hiện thời) |
| **Bấm S1 (Tiến)** | `FOr ` (Forward) | Giữ nguyên tốc độ |
| **Bấm S2 (Lùi)** | `bACK` (Back) | Giữ nguyên tốc độ |
| **Bấm S3 (Ngang trái)** | `LEFt` (Left) | Giữ nguyên tốc độ |
| **Bấm S4 (Ngang phải)** | `rIGt` (Right) | Giữ nguyên tốc độ |
| **Bấm S5 (Xoay trái CCW)** | `t-LF` (Turn Left) | Giữ nguyên tốc độ |
| **Bấm S6 (Xoay phải CW)** | `t-rt` (Turn Right) | Giữ nguyên tốc độ |
| **Bấm S7 (Tăng tốc)** | Giữ nguyên hướng hiện tại | Tăng +5 RPM (`S 80`, `S 85`, `S100`...) |
| **Bấm S8 (Giảm tốc)** | Giữ nguyên hướng hiện tại | Giảm -5 RPM (`S 70`, `S 65`, `S 50`...) |

### 3.2. Cụm 8 Đèn LED Đơn trên TM1638 (Master)
* **LED 1 đến LED 6:** Sáng tương ứng với phím di chuyển đang được nhấn (S1..S6). Khi thả phím, LED tắt.
* **LED 7 (trên nút S7):** Bật sáng tức thì khi bạn bấm nút S7 để phản hồi thị giác việc tăng tốc độ.
* **LED 8 (trên nút S8):** Bật sáng tức thì khi bạn bấm nút S8 để phản hồi thị giác việc giảm tốc độ.

### 3.3. Đèn LED RGB WS2812 trên Robot (Slave)
Đèn RGB trên bo mạch ESP32-S3 Slave phản ánh trực tiếp trạng thái vận hành của robot:
* **Xanh Lam (Solid Blue):** Đang kết nối tốt với Master, xe đứng yên sẵn sàng nhận lệnh.
* **Xanh Lá (Green):** Xe đang tiến tới (Forward).
* **Vàng Cam (Yellow/Orange):** Xe đang lùi lại (Backward).
* **Xanh Lơ (Cyan):** Xe đang dịch ngang sang trái (Left).
* **Tím Hồng (Magenta):** Xe đang dịch ngang sang phải (Right).
* **Xanh Dương (Blue):** Xe đang xoay tròn ngược chiều kim đồng hồ (CCW).
* **Trắng Sáng (White):** Xe đang xoay tròn thuận chiều kim đồng hồ (CW).
* **Nhấp Nháy ĐỎ (Blinking Red - 250ms):** **MẤT TÍN HIỆU VÔ TUYẾN / KÍCH HOẠT TỰ NGẮT AN TOÀN**.

---

## 4. BẢNG THÔNG SỐ VÔ TUYẾN & ĐỘ TRỄ

| Thông Số | Giá Trị Kỹ Thuật |
| :--- | :--- |
| **Băng tần hoạt động** | 433.0 MHz (Kênh 23 / 0x17) |
| **Công suất phát** | 20 dBm (100 mW) |
| **Tốc độ truyền vô tuyến (Air Rate)** | **19.2 kbps** (Tối đa) |
| **Tốc độ UART** | 9600 bps |
| **Độ trễ toàn hệ thống (End-to-End)** | **~15 – 35 ms** |
| **Thời gian truyền sóng (Air Time)** | ~2.5 – 8 ms / gói |
| **Khoảng cách kết nối tối đa** | 2.0 – 3.0 km (Ngoài trời, Line-of-Sight); 150 – 400 m (Trong nhà/xưởng) |
| **Khoảng cách kết nối tối thiểu** | Khuyến nghị **≥ 0.5 m** (Tránh bão hòa mạch thu LNA ở vùng trường gần) |
| **Thời gian Watchdog tự ngắt** | **650 ms** không có tín hiệu |

---

## 5. HƯỚNG DẪN BIÊN DỊCH VÀ NẠP FIRMWARE (PLATFORMIO)

Dự án được cấu hình bằng PlatformIO với 2 môi trường riêng biệt trong [platformio.ini](platformio.ini):

### 5.1. Nạp Firmware cho Master (Tay Cầm)
1. Cắm cáp USB vào cổng UART/Native USB của ESP32-S3 Master.
2. Mở terminal tại thư mục dự án và chạy:
   ```bash
   pio run -e master -t upload
   ```
3. Mở Serial Monitor để theo dõi log:
   ```bash
   pio device monitor -b 115200
   ```

### 5.2. Nạp Firmware cho Slave (Robot Omni)
1. Cắm cáp USB vào cổng ESP32-S3 Slave trên robot.
2. Mở terminal tại thư mục dự án và chạy:
   ```bash
   pio run -e slave -t upload
   ```
3. Mở Serial Monitor:
   ```bash
   pio device monitor -b 115200
   ```

---

## 6. CẤU TRÚC THƯ MỤC DỰ ÁN

```
movinglora/
├── include/
│   ├── E32LoRaDriver.h     # Driver điều khiển module LoRa E32 (Tốc độ 19.2 kbps, quản lý AUX)
│   └── TM1638Driver.h      # Driver quét phím & điều khiển hiển thị 8 LED 7 đoạn TM1638
├── src/
│   ├── master.cpp          # Mã nguồn bộ phát Master (TM1638 + LoRa E32)
│   └── slave.cpp           # Mã nguồn bộ nhận Slave (Kinematics 3 Omni + Failsafe Auto-Cutoff)
├── .gitignore              # Loại trừ file build và cache
├── platformio.ini          # Cấu hình PlatformIO cho cả 2 môi trường master & slave
└── README.md               # Tài liệu hướng dẫn chi tiết
```
