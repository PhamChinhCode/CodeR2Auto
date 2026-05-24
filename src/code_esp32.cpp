#include <Wire.h>
#include <Preferences.h>
#include "Arduino.h"
#include "PCF8574.h"
#include "Adafruit_VL53L0X.h"

// ──────────────────────────────────────────────
// I2C → Pico slave
// ──────────────────────────────────────────────
#define SDA_PIN 8
#define SCL_PIN 9
#define SLAVE_ADDRESS_PICO 0x42

// ──────────────────────────────────────────────
// PCF8574
// ──────────────────────────────────────────────
PCF8574 pcf8574(0x20, &Wire1);

// ──────────────────────────────────────────────
// MOTOR COMMANDS
// ──────────────────────────────────────────────
#define START_BYTE 0x80
#define STOP_BYTE 0x81
#define GET_BYTE 0x82
#define SET_BYTE 0x83
// Physical motor
#define MECANUM_MOTOR 0x40
#define DC_NANG_HA 0x41
#define DC_TIENLUI 0x42
#define DC_XOAY 0x43
#define DC_KEO 0x44

#define MOTOR_FORWARD 0x52
#define MOTOR_KEO 0x52
#define MOTOR_NANG 0x52
#define MOTOR_TIEN 0x52
#define MOTOR_LEN 0x52

#define MOTOR_THA 0x51
#define MOTOR_LUI 0x51
#define MOTOR_HA 0x51
#define MOTOR_XUONG 0x51
#define MOTOR_BACKWARD 0x51

#define MOTOR_STOP 0x50

#define STOP 0x01
#define UP 0x02
#define DOWN 0x03
#define LEFT 0x04
#define RIGHT 0x05
#define RO_LEFT 0x06
#define RO_RIGHT 0x07

#define UP_UNLIMIT 0x10
#define DOWN_UNLIMIT 0x11
#define LEFT_UNLIMIT 0x12
#define RIGHT_UNLIMIT 0x13
#define UP_LEFT_UNLIMIT 0x14
#define UP_RIGHT_UNLIMIT 0x15
#define RO_LEFT_UNLIMIT 0x16
#define RO_RIGHT_UNLIMIT 0x17

// ──────────────────────────────────────────────
// LỆNH ĐI CHÉO CÓ KHOẢNG CÁCH VÀ KO KHOẢNG CÁCH
#define CROSS_UP_LEFT 0X20
#define CROSS_UP_RIGHT 0X21
#define CROSS_DOWN_LEFT 0X22
#define CROSS_DOWN_RIGHT 0X23
#define CROSS_UP_LEFT_UNLIMIT 0X24
#define CROSS_UP_RIGHT_UNLIMIT 0X25
#define CROSS_DOWN_LEFT_UNLIMIT 0X26
#define CROSS_DOWN_RIGHT_UNLIMIT 0X27

// ──────────────────────────────────────────────
// nút chọn chế độ
// ──────────────────────────────────────────────
#define START_BUTTON 4 // xanh lá cây
#define SAN_BUTTON 2   // xanh dương
#define TASK_BUTTON 1  // trắng

// ──────────────────────────────────────────────
// chân cảm biến quang
// ──────────────────────────────────────────────
#define CB_QUANG_PIN_1 39 // Cảm biến quang kích HIGH
#define CB_QUANG_PIN_2 40 // Cảm biến quang kích HIGH
// ──────────────────────────────────────────────
//  chân CTHT CỤM KẸP ĐẦU VŨ KHÍ
// ──────────────────────────────────────────────
#define PIN_CTHT_TREN_DONGCO_KEP 21  // Chân cảm biến CTHT trên động cơ kẹp đầu
#define PIN_CTHT_DUOI_DONGCO_KEP 47  // Chân cảm biến CTHT dưới động cơ kẹp đầu
#define PIN_CTHT_TREN_DONGCO_NANG 35 // Chân cảm biến CTHT trên động cơ nâng ROBOT
#define PIN_CTHT_DUOI_DONGCO_NANG 48 // Chân cảm biến CTHT dưới động cơ nâng ROBOT
#define PIN_CTHT_TREN_DONGCO_KEO 37  // Chân cảm biến CTHT 1 (dùng để xác định vị trí gắp )
#define PIN_CTHT_DUOI_DONGCO_KEO 36  // Chân cảm biến CTHT 2 (dùng để xác định vị trí gắp )
#define PIN_CTHT_1 38                // Chân cảm biến CTHT 3 (dùng để xác định vị trí gắp )
// #define PIN_CTHT 37 // Chân cảm biến CTHT chua dung
// #define PIN_CTHT 38 // Chân cảm biến CTHT chua dung

// ──────────────────────────────────────────────
// chân UART2 DỮ LIỆU CAMERA
// ──────────────────────────────────────────────
#define UART_TX2 15 // Chân truyền dữ liệu UART
#define UART_RX2 16 // Chân nhận dữ liệu UART

// ──────────────────────────────────────────────
//  chân CẢM BIẾN SIÊU ÂM
// ──────────────────────────────────────────────
#define SONAR_UP 0x72
#define SONAR_DOWN 0x73
#define SONAR_LEFT 0x74

#define SONAR_UP_TRIG_PIN 13
#define SONAR_UP_ECHO_PIN 12
#define SONAR_DOWN_TRIG_PIN 10
#define SONAR_DOWN_ECHO_PIN 46
#define SONAR_LEFT_TRIG_PIN 17 // DUNG  CHÂN UART 1
#define SONAR_LEFT_ECHO_PIN 18 // DUNG  CHÂN UART 1

// ──────────────────────────────────────────────
//  chân CẢM BIẾN laser
// ──────────────────────────────────────────────
#define LASER_LEFT 0x30
#define LASER_RIGHT 0x31

#define XSHUT_LEFT_PIN 41
#define XSHUT_RIGHT_PIN 42

#define WIRE_SDA 6
#define WIRE_SCL 7

// xilanh

#define XILANH_KEP 0
#define XILANH_DAY 1

// ──────────────────────────────────────────────
// gửi dữ liệu qua UART2
// ──────────────────────────────────────────────
#define PKT_LEN 14
static uint8_t _uartBuf[PKT_LEN];
static uint8_t _uartIdx = 0;

// ──────────────────────────────────────────────

struct ControlCMD
{
    uint8_t physicMotor;
    uint8_t direct;
    uint16_t speed;
    uint16_t distance;
};

void readUartFromPC();
void onTagReceived(uint8_t tag_id, float dist, float x, float z, float pitch);
void int_pcf8574();
void int_GPIO();
void initLaser();

int getDisLaser(uint8_t cb_id);
int getSonar(uint8_t cb_id);

void sendControlCMD(ControlCMD cmd, uint8_t cmdType = SET_BYTE);
ControlCMD getControlCMD(uint8_t motor);

// hàm hỗ trợ di chuyển
void horizonAlign();
void verticalAlign();

void autoSanDo();

// flat for run code
bool auto_start = false;
bool auto_done = false;
bool robotReady = false;
bool tagReceived = false; // Biến toàn cục để lưu thông tin tag nhận được

// Chế độ sân: 0 = chưa chọn, 1 = sân đỏ, 2= sân xanh
int cheDoSan = 0;
int startstop = 0;
int gapvk_gaphop = 0;

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, UART_RX2, UART_TX2);
    Wire1.begin(SDA_PIN, SCL_PIN);

    Wire1.setClock(100000);
    Wire1.setTimeout(50); // ← tăng từ 5 lên 50ms
    int_pcf8574();
    int_GPIO();
}
// ──────────────────────────────────────────────
// LOOP
// ──────────────────────────────────────────────
bool lastCTHTKeoDuoi = 1;
bool lastCTHTKeoTren = 1;

void loop()
{
    readUartFromPC(); // Nhận dữ liệu Tag từ Camera (Serial2)
    // kiemtratrangthainut();

    if (auto_done)
    {
        auto_done = false;
        robotReady = false;
        Serial.println("XONG! Nhan START de chay lai.");
    }
}
void sendControlCMD(ControlCMD cmd, uint8_t cmdType)
{
    uint8_t data[8];
    data[0] = START_BYTE;
    data[1] = cmd.physicMotor;
    data[2] = cmd.direct;
    data[3] = (cmd.speed >> 8) & 0xFF;
    data[4] = cmd.speed & 0xFF;
    data[5] = (cmd.distance >> 8) & 0xFF;
    data[6] = cmd.distance & 0xFF;
    data[7] = cmdType;

    Wire1.beginTransmission(SLAVE_ADDRESS_PICO);
    Wire1.write(data, 8);
    uint8_t error = Wire1.endTransmission();

    if (error == 0)
    {
        // Serial.printf("Motor send: 0x%02X  speed: %d  dist: %d\n", cmd.physicMotor, cmd.speed, cmd.distance);
    }
    else
    {
        Serial.print("I2C send CMD Error: ");
        Serial.println(error);
    }
}
ControlCMD getControlCMD(uint8_t motor)
{
    ControlCMD cmd;
    cmd.physicMotor = motor;
    sendControlCMD(cmd, GET_BYTE); // Gửi lệnh GET để yêu cầu Pico gửi lại trạng thái hiện tại

    uint8_t data[8];
    Wire1.requestFrom(SLAVE_ADDRESS_PICO, 8);
    if (Wire1.available() >= 8)
    {
        for (int i = 0; i < 8; i++)
            data[i] = Wire1.read();
        if (data[0] == START_BYTE)
        {
            cmd.physicMotor = data[1];
            cmd.direct = data[2];
            cmd.speed = ((uint16_t)data[3] << 8) | data[4];
            cmd.distance = ((uint16_t)data[5] << 8) | data[6];
        }
    }
    return cmd;
}
void int_GPIO()
{

    // Chân cảm biến & CTHT
    pinMode(PIN_CTHT_TREN_DONGCO_KEP, INPUT);
    pinMode(PIN_CTHT_DUOI_DONGCO_KEP, INPUT);
    pinMode(PIN_CTHT_TREN_DONGCO_NANG, INPUT);
    pinMode(PIN_CTHT_DUOI_DONGCO_NANG, INPUT);
    pinMode(PIN_CTHT_TREN_DONGCO_KEO, INPUT);
    pinMode(PIN_CTHT_DUOI_DONGCO_KEO, INPUT);
    pinMode(PIN_CTHT_1, INPUT);

    // Chân nút chọn sân và START
    pinMode(START_BUTTON, INPUT_PULLUP); // Nút START AUTO
    pinMode(SAN_BUTTON, INPUT_PULLUP);   // Nút chọn sân
    pinMode(TASK_BUTTON, INPUT_PULLUP);

    // Chân cảm biến quang
    pinMode(CB_QUANG_PIN_1, INPUT); // Cảm biến quang kích HIGH
    pinMode(CB_QUANG_PIN_2, INPUT); // Cảm biến quang kích HIGH

    // Cảm biến siêu âm
    pinMode(SONAR_UP_TRIG_PIN, OUTPUT);
    pinMode(SONAR_UP_ECHO_PIN, INPUT);

    pinMode(SONAR_DOWN_TRIG_PIN, OUTPUT);
    pinMode(SONAR_DOWN_ECHO_PIN, INPUT);

    pinMode(SONAR_LEFT_TRIG_PIN, OUTPUT);
    pinMode(SONAR_LEFT_ECHO_PIN, INPUT);

    // Kích hoạt cảm biến siêu âm ở trạng thái LOW ban đầu
    digitalWrite(SONAR_UP_TRIG_PIN, LOW);
    digitalWrite(SONAR_DOWN_TRIG_PIN, LOW);
    digitalWrite(SONAR_LEFT_TRIG_PIN, LOW);
}
void int_pcf8574()
{
    Serial.print("Initializing PCF8574...");
    if (!pcf8574.begin())
    {
        Serial.println("PCF8574 not found");
    }
    else
    {
        Serial.println("Done");
    }
}
void readUartFromPC()
{
    while (Serial2.available())
    {
        uint8_t b = (uint8_t)Serial2.read();

        if (_uartIdx == 0 && b != 0xAB)
            continue;
        if (_uartIdx == 1 && b != 0xCD)
        {
            _uartIdx = 0;
            continue;
        }

        _uartBuf[_uartIdx++] = b;
        if (_uartIdx < PKT_LEN)
            continue;
        _uartIdx = 0;

        uint8_t chk = 0;
        for (int i = 0; i < 12; i++)
            chk ^= _uartBuf[i];
        if (chk != _uartBuf[12])
        {
            Serial.println("[UART] Checksum loi");
            continue;
        }

        // Unpack
        uint8_t tag_id = _uartBuf[2];
        uint16_t dist_raw = ((uint16_t)_uartBuf[3] << 8) | _uartBuf[4];
        int16_t x_raw = ((int16_t)_uartBuf[5] << 8) | _uartBuf[6];
        int16_t z_raw = ((int16_t)_uartBuf[7] << 8) | _uartBuf[8];
        int16_t p_raw = ((int16_t)_uartBuf[9] << 8) | _uartBuf[10];

        float dist = dist_raw / 10.0f;
        float x = x_raw / 10.0f;
        float z = z_raw / 10.0f;
        float pitch = p_raw / 10.0f;

        onTagReceived(tag_id, dist, x, z, pitch);
    }
}
void onTagReceived(uint8_t tag_id, float dist, float x, float z, float pitch)
{
    Serial.printf("[TAG] ID=%d dist=%.1f x=%.1f z=%.1f pitch=%.1f\n", tag_id, dist, x, z, pitch);
    tagReceived = true;
}
void initLaser()
{

    pinMode(XSHUT_LEFT_PIN, OUTPUT);
    pinMode(XSHUT_RIGHT_PIN, OUTPUT);

    digitalWrite(XSHUT_LEFT_PIN, LOW);
    digitalWrite(XSHUT_RIGHT_PIN, LOW);
    digitalWrite(XSHUT_LEFT_PIN, LOW);
    digitalWrite(XSHUT_RIGHT_PIN, LOW);
    delay(10);
    // all unreset
    digitalWrite(XSHUT_LEFT_PIN, HIGH);
    digitalWrite(XSHUT_RIGHT_PIN, HIGH);
    delay(10);

    // activating LOX1 and resetting LOX2
    digitalWrite(XSHUT_LEFT_PIN, HIGH);
    digitalWrite(XSHUT_RIGHT_PIN, LOW);

    // initing LOX1
    if (!lox1.begin(LASER_LEFT))
    {
        Serial.println(F("Failed to boot first VL53L0X"));
    }
    delay(10);

    // activating LOX2
    digitalWrite(XSHUT_RIGHT_PIN, HIGH);
    delay(10);

    // initing LOX2
    if (!lox2.begin(LASER_RIGHT))
    {
        Serial.println(F("Failed to boot second VL53L0X"));
    }
}
void autoSanDo() // Hàm thực hiện quy trình tự động cho sân đỏ
{
    sendControlCMD(ControlCMD{MECANUM_MOTOR, DOWN_UNLIMIT, 100, 0}); // Lệnh di chuyển xuống không giới hạn
    while (1)
    {
        if (digitalRead(PIN_CTHT_1) == LOW) // Giả sử cảm biến CTHT 1 được kích hoạt khi robot đến vị trí cần dừng
        {
            sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0}); // Dừng động cơ
            delay(5);
        }
    }
    sendControlCMD(ControlCMD{MECANUM_MOTOR, LEFT_UNLIMIT, 100, 0});
    while (1)
    {
        int sensorValue = digitalRead(CB_QUANG_PIN_1); // Đọc giá trị
        if (sensorValue == HIGH)
        {
            Serial.println("kích hoạt cảm biến quang - có vật");
            sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0}); // Dừng động cơ
            break;
        }
        delay(5);
    }
    pcf8574.write(XILANH_KEP, LOW); // Kích hoạt xylanh đẩy
    delay(350);
}
// sendControlCMD(ControlCMD{OMNI_MOTOR, CROSS_UP_RIGHT, 100, 20000});
// delay(50);
// while (getControlCMD(OMNI_MOTOR).distance > 0) // Đợi đến khi hoàn thành lệnh di chuyển
// {
//     delay(50);
// }
// sendControlCMD(ControlCMD{OMNI_MOTOR, UP, 100, 2000});
// delay(50);
// while (getControlCMD(OMNI_MOTOR).distance > 0) // Đợi đến khi hoàn thành lệnh di chuyển
// {
//     delay(50);
// }
// lên bậc 1
// horizonAlign();
//{
// Test DC Motor
//-------------------------------------------------------------------------------------------------------
// if (digitalRead(PIN_CTHT_DUOI_DONGCO_KEO) == 0 && lastCTHTKeoDuoi == 1) // Nếu cảm biến CTHT dưới động cơ kéo được kích hoạt
// {
//     ControlCMD cmd;
//     cmd.physicMotor = DC_NANG_HA;
//     cmd.direct = MOTOR_BACKWARD;
//     cmd.speed = 80;
//     cmd.distance = 0;
//     Serial.println("CTHT dưới động cơ kéo được kích hoạt - Kéo xuống");
//     sendControlCMD(cmd);
// }
// if (digitalRead(PIN_CTHT_DUOI_DONGCO_KEO) == 1 && lastCTHTKeoDuoi == 0) // Nếu cảm biến CTHT trên động cơ kéo được kích hoạt
// {
//     ControlCMD cmd;
//     cmd.physicMotor = DC_NANG_HA;
//     cmd.direct = MOTOR_STOP;
//     cmd.speed = 0;
//     cmd.distance = 0;
//     Serial.println("CTHT dưới động cơ kéo được kích hoạt - Dừng kéo");
//     sendControlCMD(cmd);
// }
// lastCTHTKeoDuoi = digitalRead(PIN_CTHT_DUOI_DONGCO_KEO);

// if (digitalRead(PIN_CTHT_TREN_DONGCO_KEO) == 0 && lastCTHTKeoTren == 1) // Nếu cảm biến CTHT trên động cơ kéo được kích hoạt
// {
//     ControlCMD cmd;
//     cmd.physicMotor = DC_NANG_HA;
//     cmd.direct = MOTOR_FORWARD;
//     cmd.speed = 80;
//     cmd.distance = 0;
//     Serial.println("CTHT trên động cơ kéo được kích hoạt - Kéo lên");
//     sendControlCMD(cmd);
// }
// if (digitalRead(PIN_CTHT_TREN_DONGCO_KEO) == 1 && lastCTHTKeoTren == 0) // Nếu cảm biến CTHT trên động cơ kéo được kích hoạt
// {
//     ControlCMD cmd;
//     cmd.physicMotor = DC_NANG_HA;
//     cmd.direct = MOTOR_STOP;
//     cmd.speed = 0;
//     cmd.distance = 0;
//     Serial.println("CTHT trên động cơ kéo được kích hoạt - Dừng kéo");
//     sendControlCMD(cmd);
// }
// lastCTHTKeoTren = digitalRead(PIN_CTHT_TREN_DONGCO_KEO);

//-----------------------------------------------------------------------------------------------------------

// Test động cơ Omni
// ControlCMD cmd;
// cmd.physicMotor = OMNI_MOTOR;
// cmd.direct = UP;
// cmd.speed = 100;
// cmd.distance = 5000;
// Serial.println("Gửi lệnh điều khiển...");
// sendControlCMD(cmd);
// delay(100);
// Serial.println("Yêu cầu trạng thái hiện tại của động cơ...");
// ControlCMD receivedCmd = getControlCMD(OMNI_MOTOR);
// Serial.printf("Lệnh nhận được: Motor: 0x%02X, Direct: 0x%02X, Speed: %d, Distance: %d\n",
//               receivedCmd.physicMotor, receivedCmd.direct, receivedCmd.speed, receivedCmd.distance);
// while (receivedCmd.distance > 0)
// {
//     delay(100);
//     receivedCmd = getControlCMD(OMNI_MOTOR);
//     Serial.printf("Lệnh nhận được: Motor: 0x%02X, Direct: 0x%02X, Speed: %d, Distance: %d\n",
//                   receivedCmd.physicMotor, receivedCmd.direct, receivedCmd.speed, receivedCmd.distance);
// }
// cmd.physicMotor = OMNI_MOTOR;
// cmd.direct = DOWN;
// cmd.speed = 100;
// cmd.distance = 5000;
// Serial.println("Gửi lệnh điều khiển...");
// sendControlCMD(cmd);
// delay(100);
// while (receivedCmd.distance > 0)
// {
//     delay(100);
//     receivedCmd = getControlCMD(OMNI_MOTOR);
//     Serial.printf("Lệnh nhận được: Motor: 0x%02X, Direct: 0x%02X, Speed: %d, Distance: %d\n",
//                   receivedCmd.physicMotor, receivedCmd.direct, receivedCmd.speed, receivedCmd.distance);
// }
// delay(3000);
// MotorStatus status1 = {0, 0, 0};
// while (status1.status != STOP_MOTOR)
// {
//     status1 = getMotorStatus();
//     Serial.printf("Motor Status: %d, Speed: %d, Pulse: %d\n", status1.status, status1.speed, status1.pulse);

//     delay(100); // Đợi một chút trước khi kiểm tra lại
// }
// Serial.println("Đã hoàn thành lệnh nâng");
// while (
//--------------------------------
// readUartFromPC(); // Nhận dữ liệu Tag từ Camera (Serial2)
// kiemtratrangthainut();

// if (auto_done)
// {
//     auto_done = false;
//     robotReady = false;
//     Serial.println("XONG! Nhan START de chay lai.");
// }
//--------------------------------
// if (millis() - ControlTime > 505)
// {
//     Motor = getMotorStatus();
//     if (Motor.status != STOP)
//     {
//         sendMotorCommand(STOP, 0, 0);
//     }
//     ControlTime = millis();
// }

// delay(1);
//}