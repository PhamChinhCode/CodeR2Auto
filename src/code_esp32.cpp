#include <Wire.h>
#include <Preferences.h>
#include "Arduino.h"
#include "PCF8574.h"
#include "Adafruit_VL53L0X.h"
#include <ESP32Servo.h>

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
Servo servoKep;
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
#define SONAR_RIGHT 0x75

#define SONAR_UP_TRIG_PIN 12
#define SONAR_UP_ECHO_PIN 13
#define SONAR_DOWN_TRIG_PIN 46
#define SONAR_DOWN_ECHO_PIN 10
#define SONAR_LEFT_TRIG_PIN 18  // DUNG  CHÂN UART 1
#define SONAR_LEFT_ECHO_PIN 17  // DUNG  CHÂN UART 1
#define SONAR_RIGHT_TRIG_PIN 40 // DUNG  CHÂN UART 1
#define SONAR_RIGHT_ECHO_PIN 5  // DUNG  CHÂN UART 1

// ──────────────────────────────────────────────
//  chân CẢM BIẾN laser
// ──────────────────────────────────────────────
#define LASER_LEFT 0x30
#define LASER_RIGHT 0x31

#define XSHUT_LEFT_PIN 41
#define XSHUT_RIGHT_PIN 42

#define WIRE_SDA 7
#define WIRE_SCL 6

// Servo
#define SERVO_KEP_PIN 14

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
void fontAlign(int distance, uint16_t speed) // căn chỉnh khoảng cách với tường phía trước bằng cảm biến laser
{
    sendControlCMD(ControlCMD{MECANUM_MOTOR, UP_UNLIMIT, speed, 2000});
    while (getDisLaser(LASER_LEFT) > distance || getDisLaser(LASER_RIGHT) > distance)
    {
        delay(5);
    }
    sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0});
}
void leftAlign(int distance, uint16_t speed) // căn chỉnh khoảng cách với tường bên trái bằng cảm biến siêu âm
{
    while (getSonar(SONAR_LEFT) != distance)
    {
        delay(5);
        if (getSonar(SONAR_LEFT) > distance)
        {
            sendControlCMD(ControlCMD{MECANUM_MOTOR, LEFT_UNLIMIT, speed, 0});
        }
        else
        {
            sendControlCMD(ControlCMD{MECANUM_MOTOR, RIGHT_UNLIMIT, speed, 0});
        }
    }
    sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0});
}
void verticalAlign() // căn chỉnh song song với tường bằng cảm biến laser
{
    while (getDisLaser(LASER_LEFT) / 10 != getDisLaser(LASER_RIGHT) / 10)
    {
        delay(5);
        if (getDisLaser(LASER_LEFT) > getDisLaser(LASER_RIGHT))
        {
            sendControlCMD(ControlCMD{MECANUM_MOTOR, RO_LEFT_UNLIMIT, 20, 0});
        }
        else
        {
            sendControlCMD(ControlCMD{MECANUM_MOTOR, RO_RIGHT_UNLIMIT, 20, 0});
        }
    }
    sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0});
}
void moveABS(uint8_t dir, uint16_t speed, uint16_t distance)
{
    sendControlCMD(ControlCMD{MECANUM_MOTOR, dir, speed, distance});
    delay(50);
    while (getControlCMD(MECANUM_MOTOR).distance > 0) // Đợi đến khi hoàn thành lệnh di chuyển
    {
        delay(20);
    }
    sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0});
    delay(20);
}
void jumpUp(bool pick)
{
    sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_HA, 90, 0});
    while (digitalRead(PIN_CTHT_DUOI_DONGCO_NANG) == HIGH)
        ;
    sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_STOP, 0, 0});
    delay(20);
    sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_TIEN, 70, 0});
    while (getSonar(SONAR_DOWN) > 15)
    {
        delay(10);
    }
    sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_STOP, 0, 0});
    delay(20);
    servoKep.write(pick ? 170 : 0);
    delay(200);
    sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_NANG, 90, 0});
    while (digitalRead(PIN_CTHT_TREN_DONGCO_NANG) == HIGH)
        ;
    sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_STOP, 0, 0});
    delay(20);
}
void jumpDown()
{
    sendControlCMD(ControlCMD{MECANUM_MOTOR, DOWN_UNLIMIT, 50, 2000});
    while (getSonar(SONAR_DOWN) < 15)
    {
        delay(5);
    }
    sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0});
    delay(20);
    sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_HA, 90, 0});
    while (digitalRead(PIN_CTHT_DUOI_DONGCO_NANG) == HIGH)
        ;
    sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_STOP, 0, 0});
    delay(20);
    sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_LUI, 50, 0});
    while (getSonar(SONAR_UP) < 15)
    {
        delay(5);
    }
    delay(40);
    sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_STOP, 0, 0});
    delay(20);
    sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_NANG, 90, 0});
    while (digitalRead(PIN_CTHT_TREN_DONGCO_NANG) == HIGH)
        ;
    sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_STOP, 0, 0});
}

void autogapvkSanDo();
void runForest()
{
    moveABS(UP, 20, 2000);
    //---> cbi lên bậc 1
    fontAlign(150, 60);
    verticalAlign();
    fontAlign(70, 20);
    jumpUp(false);
    /// --> đã lên được bậc 1
    fontAlign(150, 60);
    verticalAlign();
    leftAlign(42, 20);
    fontAlign(70, 20);
    jumpUp(true);
    sendControlCMD(ControlCMD{DC_KEO, MOTOR_KEO, 70, 0});
    delay(1500);
    sendControlCMD(ControlCMD{DC_KEO, MOTOR_STOP, 0, 0});
    // ------>> đã lên được bậc 2

    fontAlign(150, 60);
    verticalAlign();
    fontAlign(70, 20);
    jumpUp(true);
    // ------>> đã lên được bậc 3
    moveABS(UP, 50, 2000);
    moveABS(RO_LEFT, 50, 1100);
    jumpDown();
    // ------>> đã xuống được bậc 2
    moveABS(DOWN, 50, 2000);
    verticalAlign();
    moveABS(RO_LEFT, 50, 1100);
    jumpDown();
    // ------>> đã xuống được bậc 1
    moveABS(DOWN, 50, 2000);
    verticalAlign();
    jumpDown();
    // ------>> đã xuống được bậc 0
    moveABS(DOWN, 50, 2000);
    verticalAlign();
}
void gotoSideC()
{
    // lên khu C
    moveABS(LEFT, 70, 8000);
    delay(50);
    moveABS(RO_LEFT, 50, 2200);
    delay(50);
    sendControlCMD(ControlCMD{DC_KEO, MOTOR_KEO, 70, 0});
    while (digitalRead(PIN_CTHT_TREN_DONGCO_KEO) == HIGH)
        ;
    sendControlCMD(ControlCMD{DC_KEO, MOTOR_STOP, 0, 0});
    delay(50);
    sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_TIEN, 90, 0});
    sendControlCMD(ControlCMD{MECANUM_MOTOR, UP_LEFT_UNLIMIT, 100, 0});
    delay(1500);
    fontAlign(300, 100);
    sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_STOP, 90, 0});
    verticalAlign();
    delay(20);
    moveABS(DOWN, 50, 1000);
    delay(20);
}
void putKFS()
{
    moveABS(RO_LEFT, 50, 1100);
    delay(20);
    moveABS(UP, 100, 25000);
    delay(20);
    fontAlign(700, 30);
    delay(20);
    verticalAlign(); // căn vuông góc bờ đối diện
    delay(20);
    leftAlign(50, 30); // vij triss caafn chirnh
    delay(20);
    verticalAlign();
    delay(20);
    sendControlCMD(ControlCMD{DC_KEO, MOTOR_KEO, 90, 0});
    while (digitalRead(PIN_CTHT_TREN_DONGCO_KEO) == HIGH)
        ;
    sendControlCMD(ControlCMD{DC_KEO, MOTOR_STOP, 0, 0});
    delay(20);

    fontAlign(250, 20);
    delay(20);
    servoKep.write(0);
    delay(200);
    moveABS(DOWN, 50, 3000);
    delay(20);
    sendControlCMD(ControlCMD{DC_KEO, MOTOR_THA, 90, 0});
    while (digitalRead(PIN_CTHT_DUOI_DONGCO_KEO) == HIGH)
        ;
    sendControlCMD(ControlCMD{DC_KEO, MOTOR_STOP, 0, 0});
    delay(20);
}
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
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    servoKep.setPeriodHertz(50); // Standard 50hz servo
    servoKep.attach(SERVO_KEP_PIN, 500, 19000);
    servoKep.write(0);
    int_pcf8574();
    int_GPIO();
    initLaser();
}
// ──────────────────────────────────────────────
// LOOP
// ──────────────────────────────────────────────
bool lastCTHTKeoDuoi = 1;
bool lastCTHTKeoTren = 1;
void testcb()
{
    while (true)
    {
        // Serial.println("Test DC KEO");
        // sendControlCMD(ControlCMD{DC_KEO, MOTOR_KEO, 70, 0});
        // while (digitalRead(PIN_CTHT_TREN_DONGCO_KEO) == HIGH)
        //     ;
        // sendControlCMD(ControlCMD{DC_KEO, MOTOR_STOP, 0, 0});
        // delay(2000);
        // sendControlCMD(ControlCMD{DC_KEO, MOTOR_THA, 70, 0});
        // while (digitalRead(PIN_CTHT_DUOI_DONGCO_KEO) == HIGH)
        //     ;
        // sendControlCMD(ControlCMD{DC_KEO, MOTOR_STOP, 0, 0});
        // delay(2000);
        // Serial.println("Test DC NANG");

        // sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_HA, 90, 0});
        // while (digitalRead(PIN_CTHT_DUOI_DONGCO_NANG) == HIGH)
        //     ;
        // sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_STOP, 0, 0});
        // delay(2000);
        // sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_NANG, 90, 0});
        // while (digitalRead(PIN_CTHT_TREN_DONGCO_NANG) == HIGH)
        //     ;
        // sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_STOP, 0, 0});
        // delay(2000);
        // Serial.println("Test DC TIENLUI");
        // sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_TIEN, 70, 0});
        // delay(2000);
        // sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_STOP, 0, 0});
        // delay(2000);
        // sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_LUI, 70, 0});
        // delay(2000);
        // sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_STOP, 0, 0});
        // delay(2000);
        // Serial.println("Test DC XOAY");
        // sendControlCMD(ControlCMD{DC_XOAY, MOTOR_LEN, 70, 0});
        // while (digitalRead(PIN_CTHT_TREN_DONGCO_KEP) == HIGH)
        //     ;
        // sendControlCMD(ControlCMD{DC_XOAY, MOTOR_STOP, 0, 0});
        // delay(2000);
        // sendControlCMD(ControlCMD{DC_XOAY, MOTOR_XUONG, 70, 0});
        // while (digitalRead(PIN_CTHT_DUOI_DONGCO_KEP) == HIGH)
        //     ;
        // sendControlCMD(ControlCMD{DC_XOAY, MOTOR_STOP, 0, 0});
        // delay(2000);
        // Serial.println("Test DC KEO");
        // sendControlCMD(ControlCMD{DC_KEO, MOTOR_KEO, 70, 0});
        // while (digitalRead(PIN_CTHT_TREN_DONGCO_KEO) == HIGH)
        //     ;
        // sendControlCMD(ControlCMD{DC_KEO, MOTOR_STOP, 0, 0});
        // delay(2000);
        // sendControlCMD(ControlCMD{DC_KEO, MOTOR_THA, 70, 0});
        // while (digitalRead(PIN_CTHT_DUOI_DONGCO_KEO) == HIGH)
        //     ;
        // sendControlCMD(ControlCMD{DC_KEO, MOTOR_STOP, 0, 0});
        // delay(2000);
        // Serial.println("Test Xilanh KEP");
        // pcf8574.write(XILANH_KEP, LOW);
        // delay(2000);
        // pcf8574.write(XILANH_KEP, HIGH);
        // delay(2000);
        // pcf8574.write(XILANH_DAY, LOW);
        // delay(2000);
        // pcf8574.write(XILANH_DAY, HIGH);
        // delay(2000);
        // Serial.println("Test Servo KEP");
        servoKep.write(170);
        delay(2000);
        // servoKep.write(0);
        // delay(2000);
        while (true)
        {
            delay(500);
            Serial.printf("Sensor: LL=%d LR=%d SL=%d SR=%d SU=%d SD=%d\n", getDisLaser(LASER_LEFT), getDisLaser(LASER_RIGHT), getSonar(SONAR_LEFT), getSonar(SONAR_RIGHT), getSonar(SONAR_UP), getSonar(SONAR_DOWN));
        }
    }
    //
    // sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_NANG, 90, 0});
    // while (digitalRead(PIN_CTHT_TREN_DONGCO_NANG) == HIGH)
    //     ;
    // sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_STOP, 0, 0});
    // servoKep.write(0);
    //
    // moveABS(UP, 200, 50000);
    // moveABS(RO_LEFT, 200, 11000);
    delay(3000);
    runForest();
    delay(10000);
    while (true)
    {

        sendControlCMD(ControlCMD{MECANUM_MOTOR, UP, 20, 2000});
        delay(50);
        while (getControlCMD(MECANUM_MOTOR).distance > 0) // Đợi đến khi hoàn thành lệnh di chuyển
        {
            delay(50);
        }
        delay(2000);
        //---> cbi lên bậc 1
        sendControlCMD(ControlCMD{MECANUM_MOTOR, UP_UNLIMIT, 20, 2000});
        while (getDisLaser(LASER_LEFT) > 150 || getDisLaser(LASER_RIGHT) > 150)
        {
            delay(10);
        }
        sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0});
        delay(2000);
        while (getDisLaser(LASER_LEFT) / 10 != getDisLaser(LASER_RIGHT) / 10)
        {
            delay(5);
            if (getDisLaser(LASER_LEFT) > getDisLaser(LASER_RIGHT))
            {
                sendControlCMD(ControlCMD{MECANUM_MOTOR, RO_LEFT_UNLIMIT, 20, 0});
            }
            else
            {
                sendControlCMD(ControlCMD{MECANUM_MOTOR, RO_RIGHT_UNLIMIT, 20, 0});
            }
        }
        sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0});
        delay(2000);
        sendControlCMD(ControlCMD{MECANUM_MOTOR, UP_UNLIMIT, 20, 2000});
        while (getDisLaser(LASER_LEFT) > 50 || getDisLaser(LASER_RIGHT) > 50)
        {
            delay(10);
        }
        sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0});
        delay(2000);
        sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_HA, 90, 0});
        while (digitalRead(PIN_CTHT_DUOI_DONGCO_NANG) == HIGH)
            ;
        sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_STOP, 0, 0});
        delay(2000);
        sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_TIEN, 50, 0});
        while (getSonar(SONAR_DOWN) > 15)
        {
            delay(10);
        }
        sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_STOP, 0, 0});
        delay(2000);
        sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_NANG, 90, 0});
        while (digitalRead(PIN_CTHT_TREN_DONGCO_NANG) == HIGH)
            ;
        sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_STOP, 0, 0});
        delay(2000);
        /// --> đã lên được bậc 1
        sendControlCMD(ControlCMD{MECANUM_MOTOR, UP_UNLIMIT, 20, 2000});
        while (getDisLaser(LASER_LEFT) > 150 || getDisLaser(LASER_RIGHT) > 150)
        {
            delay(10);
        }
        sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0});
        delay(2000);
        while (getDisLaser(LASER_LEFT) / 10 != getDisLaser(LASER_RIGHT) / 10)
        {
            delay(5);
            if (getDisLaser(LASER_LEFT) > getDisLaser(LASER_RIGHT))
            {
                sendControlCMD(ControlCMD{MECANUM_MOTOR, RO_LEFT_UNLIMIT, 20, 0});
            }
            else
            {
                sendControlCMD(ControlCMD{MECANUM_MOTOR, RO_RIGHT_UNLIMIT, 20, 0});
            }
        }
        sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0});
        delay(2000);
        while (getSonar(SONAR_LEFT) != 42)
        {
            delay(5);
            if (getSonar(SONAR_LEFT) > 42)
            {
                sendControlCMD(ControlCMD{MECANUM_MOTOR, LEFT_UNLIMIT, 20, 0});
            }
            else
            {
                sendControlCMD(ControlCMD{MECANUM_MOTOR, RIGHT_UNLIMIT, 20, 0});
            }
        }
        sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0});
        delay(2000);
        sendControlCMD(ControlCMD{MECANUM_MOTOR, UP_UNLIMIT, 20, 2000});
        while (getDisLaser(LASER_LEFT) > 50 || getDisLaser(LASER_RIGHT) > 50)
        {
            delay(10);
        }
        sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0});
        delay(2000);
        sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_HA, 90, 0});
        while (digitalRead(PIN_CTHT_DUOI_DONGCO_NANG) == HIGH)
            ;
        sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_STOP, 0, 0});
        delay(2000);
        sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_TIEN, 50, 0});
        while (getSonar(SONAR_DOWN) > 15)
        {
            delay(10);
        }
        sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_STOP, 0, 0});
        delay(2000);
        servoKep.write(170);
        delay(2000);
        sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_NANG, 90, 0});
        while (digitalRead(PIN_CTHT_TREN_DONGCO_NANG) == HIGH)
            ;
        sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_STOP, 0, 0});
        delay(2000);
        // ------>> đã lên được bậc 2

        sendControlCMD(ControlCMD{MECANUM_MOTOR, UP_UNLIMIT, 20, 2000});
        while (getDisLaser(LASER_LEFT) > 150 || getDisLaser(LASER_RIGHT) > 150)
        {
            delay(10);
        }
        sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0});
        delay(2000);
        while (getDisLaser(LASER_LEFT) / 10 != getDisLaser(LASER_RIGHT) / 10)
        {
            delay(5);
            if (getDisLaser(LASER_LEFT) > getDisLaser(LASER_RIGHT))
            {
                sendControlCMD(ControlCMD{MECANUM_MOTOR, RO_LEFT_UNLIMIT, 20, 0});
            }
            else
            {
                sendControlCMD(ControlCMD{MECANUM_MOTOR, RO_RIGHT_UNLIMIT, 20, 0});
            }
        }
        sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0});
        delay(2000);
        sendControlCMD(ControlCMD{MECANUM_MOTOR, UP_UNLIMIT, 20, 2000});
        while (getDisLaser(LASER_LEFT) > 50 || getDisLaser(LASER_RIGHT) > 50)
        {
            delay(10);
        }
        sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0});
        delay(2000);
        sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_HA, 90, 0});
        while (digitalRead(PIN_CTHT_DUOI_DONGCO_NANG) == HIGH)
            ;
        sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_STOP, 0, 0});
        delay(2000);
        sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_TIEN, 50, 0});
        while (getSonar(SONAR_DOWN) > 15)
        {
            delay(10);
        }
        sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_STOP, 0, 0});
        delay(2000);
        sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_NANG, 90, 0});
        while (digitalRead(PIN_CTHT_TREN_DONGCO_NANG) == HIGH)
            ;
        sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_STOP, 0, 0});
        delay(2000);
        // ------>> đã lên được bậc 3
        sendControlCMD(ControlCMD{MECANUM_MOTOR, UP, 20, 2000});
        delay(10);
        while (getControlCMD(MECANUM_MOTOR).distance > 0) // Đợi đến khi hoàn thành lệnh di chuyển
        {
            delay(10);
        }
        delay(2000);
        sendControlCMD(ControlCMD{MECANUM_MOTOR, RO_LEFT, 20, 1100});
        delay(10);
        while (getControlCMD(MECANUM_MOTOR).distance > 0) // Đợi đến khi hoàn thành lệnh di chuyển
        {
            delay(10);
        }
        delay(2000);
        sendControlCMD(ControlCMD{MECANUM_MOTOR, DOWN_UNLIMIT, 20, 2000});
        while (getSonar(SONAR_DOWN) < 15)
        {
            delay(10);
        }
        sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0});
        delay(2000);
        sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_HA, 90, 0});
        while (digitalRead(PIN_CTHT_DUOI_DONGCO_NANG) == HIGH)
            ;
        sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_STOP, 0, 0});
        delay(2000);
        sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_LUI, 50, 0});
        while (getSonar(SONAR_UP) < 15)
        {
            delay(5);
        }
        sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_STOP, 0, 0});
        delay(2000);
        sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_NANG, 90, 0});
        while (digitalRead(PIN_CTHT_TREN_DONGCO_NANG) == HIGH)
            ;
        sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_STOP, 0, 0});
        //----->> đã xuống được bậc 2

        delay(10000);
        // sendControlCMD(ControlCMD{MECANUM_MOTOR, DOWN, 20, 2000});
        // delay(50);
        // while (getControlCMD(MECANUM_MOTOR).distance > 0) // Đợi đến khi hoàn thành lệnh di chuyển
        // {
        //     delay(50);
        // }
        // delay(5000);
        // sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_HA, 90, 0});
        // while (digitalRead(PIN_CTHT_DUOI_DONGCO_NANG) == HIGH)
        //     ;
        // sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_STOP, 0, 0});
        // delay(2000);
        // sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_TIEN, 99, 0});
        // while (getSonar(SONAR_DOWN) > 15)
        //     ;
        // sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_STOP, 0, 0});
        // delay(2000);
        // sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_NANG, 90, 0});
        // while (digitalRead(PIN_CTHT_TREN_DONGCO_NANG) == HIGH)
        //     ;
        // sendControlCMD(ControlCMD{DC_NANG_HA, MOTOR_STOP, 0, 0});
        // delay(2000);

        // // keo tha
        // sendControlCMD(ControlCMD{DC_KEO, MOTOR_KEO, 50, 0});
        // while (digitalRead(PIN_CTHT_TREN_DONGCO_KEO) == HIGH)
        //     ;
        // sendControlCMD(ControlCMD{DC_KEO, MOTOR_STOP, 0, 0});
        // delay(2000);
        // sendControlCMD(ControlCMD{DC_KEO, MOTOR_THA, 50, 0});
        // while (digitalRead(PIN_CTHT_DUOI_DONGCO_KEO) == HIGH)
        //     ;
        // sendControlCMD(ControlCMD{DC_KEO, MOTOR_STOP, 0, 0});
        // delay(2000);
        // sendControlCMD(ControlCMD{DC_XOAY, MOTOR_LEN, 50, 0});
        // while (digitalRead(PIN_CTHT_TREN_DONGCO_KEP) == HIGH)
        //     ;
        // sendControlCMD(ControlCMD{DC_XOAY, MOTOR_STOP, 0, 0});
        // delay(2000);
        // sendControlCMD(ControlCMD{DC_XOAY, MOTOR_XUONG, 50, 0});
        // while (digitalRead(PIN_CTHT_DUOI_DONGCO_KEP) == HIGH)
        //     ;
        // sendControlCMD(ControlCMD{DC_XOAY, MOTOR_STOP, 0, 0});
        // delay(2000);

        // while (digitalRead(PIN_CTHT_TREN_DONGCO_KEP) == HIGH)
        //     ;
        // sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_STOP, 0, 0});
        // delay(2000);
        // sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_LUI, 50, 0});
        // while (digitalRead(PIN_CTHT_DUOI_DONGCO_KEP) == HIGH)
        //     ;
        // sendControlCMD(ControlCMD{DC_TIENLUI, MOTOR_STOP, 0, 0});
        // delay(2000);
    }
}
void loop()
{

    autogapvkSanDo();

    moveABS(RIGHT, 100, 4000);
    moveABS(DOWN, 100, 6000);
    moveABS(RO_RIGHT, 100, 1100);

    runForest();
    gotoSideC();
    putKFS();
}

int getSonar(uint8_t cb_id)
{
    if (cb_id == SONAR_UP)
    {
        digitalWrite(SONAR_UP_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(SONAR_UP_TRIG_PIN, LOW);

        // Đo thời gian phản hồi
        long duration = pulseIn(SONAR_UP_ECHO_PIN, HIGH, 300000); // Timeout 30ms
        if (duration == 0)
            return -1; // Không nhận được phản hồi

        // Tính khoảng cách (cm)
        int distance = duration * 0.034 / 2;
        return distance;
    }
    else if (cb_id == SONAR_DOWN)
    {
        digitalWrite(SONAR_DOWN_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(SONAR_DOWN_TRIG_PIN, LOW);

        long duration = pulseIn(SONAR_DOWN_ECHO_PIN, HIGH, 300000);
        if (duration == 0)
            return -1;

        int distance = duration * 0.034 / 2;
        return distance;
    }
    else if (cb_id == SONAR_LEFT)
    {
        digitalWrite(SONAR_LEFT_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(SONAR_LEFT_TRIG_PIN, LOW);

        long duration = pulseIn(SONAR_LEFT_ECHO_PIN, HIGH, 300000);
        if (duration == 0)
            return -1;

        int distance = duration * 0.034 / 2;
        return distance;
    }
    else if (cb_id == SONAR_RIGHT)
    {
        digitalWrite(SONAR_RIGHT_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(SONAR_RIGHT_TRIG_PIN, LOW);

        long duration = pulseIn(SONAR_RIGHT_ECHO_PIN, HIGH, 300000);
        if (duration == 0)
            return -1;

        int distance = duration * 0.034 / 2;
        return distance;
    }

    return -1; // ID không hợp lệ
}
int getDisLaser(uint8_t cb_id)
{
    int dist;
    if (cb_id == LASER_LEFT)
    {
        dist = lox1.rangingTest(&measure1, false) == VL53L0X_ERROR_NONE ? measure1.RangeMilliMeter : -1;
        if (dist > 60)
            dist = dist - 60;
        return dist;
    }
    else if (cb_id == LASER_RIGHT)
    {
        dist = lox2.rangingTest(&measure2, false) == VL53L0X_ERROR_NONE ? measure2.RangeMilliMeter : -1;
        if (dist > 25)
            dist = dist - 25;
        return dist;
    }
    return -1; // ID không hợp lệ
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
    Wire.begin(WIRE_SDA, WIRE_SCL);
    // initing LOX1
    if (!lox1.begin(LASER_LEFT, false, &Wire))
    {
        Serial.println(F("Failed to boot first VL53L0X"));
    }
    delay(10);

    // activating LOX2
    digitalWrite(XSHUT_RIGHT_PIN, HIGH);
    delay(10);

    // initing LOX2
    if (!lox2.begin(LASER_RIGHT, false, &Wire))
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
void autogapvkSanDo() // Hàm thực hiện quy trình tự động cho sân đỏ
{
    sendControlCMD(ControlCMD{DC_XOAY, MOTOR_XUONG, 100, 0}); // Lệnh di chuyển xuống không giới hạn
    while (digitalRead(PIN_CTHT_DUOI_DONGCO_KEP) == HIGH)
        ;                                                           // Đợi đến khi cảm biến CTHT dưới động cơ kẹp được kích hoạt
    sendControlCMD(ControlCMD{DC_XOAY, MOTOR_STOP, 0, 0});          // Dừng động cơ
    sendControlCMD(ControlCMD{MECANUM_MOTOR, DOWN_UNLIMIT, 50, 0}); // Lệnh di chuyển xuống không giới hạn
    Serial.println("Di chuyển xuống...");
    while (digitalRead(PIN_CTHT_1) == HIGH)
        ;                                                  // Đợi đến khi cảm biến CTHT trên động cơ nâng được kích hoạt
    sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0}); // Dừng động cơ
    Serial.println("Đã đến vị trí cần dừng.");

    sendControlCMD(ControlCMD{MECANUM_MOTOR, LEFT_UNLIMIT, 100, 0});
    while (digitalRead(CB_QUANG_PIN_1) == 0)
        ;                                                  // Đợi đến khi cảm biến CTHT trên động cơ kẹp được kích hoạt
    sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0}); // Dừng động cơ
    Serial.println("Cam bien quang 1 kich hoat - Dung di chuyen sang trai.");
    pcf8574.write(XILANH_KEP, LOW); // Kẹp xylanh
    delay(250);
    pcf8574.write(XILANH_DAY, HIGH); // Thả xylanh
    sendControlCMD(ControlCMD{DC_XOAY, MOTOR_LEN, 100, 0});
    pcf8574.write(XILANH_DAY, HIGH); // Thả xylanh
    sendControlCMD(ControlCMD{MECANUM_MOTOR, UP, 100, 4000});
    Serial.println("Di chuyển lên...");
    delay(50);
    while (getControlCMD(MECANUM_MOTOR).distance > 0) // Đợi đến khi hoàn thành lệnh di chuyển
    {
        delay(10);
        if (digitalRead(PIN_CTHT_TREN_DONGCO_KEP) == LOW) // Nếu cảm biến CTHT trên động cơ kẹp được kích hoạt sớm hơn dự kiến
        {
            sendControlCMD(ControlCMD{DC_XOAY, MOTOR_STOP, 0, 0}); // Dừng động cơ
            break;
        }
    }
    while (digitalRead(PIN_CTHT_TREN_DONGCO_KEP) == HIGH)
        ;                                                  // Đợi đến khi cảm biến CTHT trên động cơ kẹp được kích hoạt
    sendControlCMD(ControlCMD{DC_XOAY, MOTOR_STOP, 0, 0}); // Dừng động cơ
    delay(50);
    sendControlCMD(ControlCMD{MECANUM_MOTOR, RO_RIGHT, 100, 2200});
    Serial.println("Quay phải...");
    delay(50);
    while (getControlCMD(MECANUM_MOTOR).distance > 0) // Đợi đến khi hoàn thành lệnh di chuyển
    {
        delay(50);
    }
    sendControlCMD(ControlCMD{MECANUM_MOTOR, RIGHT, 100, 2000});
    Serial.println("Di chuyển sang trái...");
    delay(50);
    while (getControlCMD(MECANUM_MOTOR).distance > 0) // Đợi đến khi hoàn thành lệnh di chuyển
    {
        delay(50);
    }

    tagReceived = false; // Reset trạng thái nhận tag để có thể nhận tag mới
    while (!tagReceived)
    {
        readUartFromPC(); // đọc liên tục cho đến khi có tag
        delay(10);
    }
    pcf8574.write(XILANH_KEP, HIGH); // Thả xylanh
    sendControlCMD(ControlCMD{DC_XOAY, MOTOR_XUONG, 100, 0});
    while (digitalRead(PIN_CTHT_DUOI_DONGCO_KEP) == HIGH)
        ;                                                  // Đợi đến khi cảm biến CTHT dưới động cơ kẹp được kích hoạt
    sendControlCMD(ControlCMD{DC_XOAY, MOTOR_STOP, 0, 0}); // Dừng động cơ
}
void autogapvkSanXanh() // Hàm thực hiện quy trình tự động cho sân xanh
{
    sendControlCMD(ControlCMD{MECANUM_MOTOR, DOWN_UNLIMIT, 100, 0}); // Lệnh di chuyển xuống không giới hạn
    Serial.println("Di chuyển xuống...");
    while (digitalRead(PIN_CTHT_1) == HIGH)
        ;                                                  // Đợi đến khi cảm biến CTHT trên động cơ nâng được kích hoạt
    sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0}); // Dừng động cơ
    Serial.println("Đã đến vị trí cần dừng.");

    sendControlCMD(ControlCMD{MECANUM_MOTOR, RIGHT_UNLIMIT, 100, 0});
    while (digitalRead(CB_QUANG_PIN_1) == 0)
        ;                                                  // Đợi đến khi cảm biến CTHT trên động cơ kẹp được kích hoạt
    sendControlCMD(ControlCMD{MECANUM_MOTOR, STOP, 0, 0}); // Dừng động cơ
    Serial.println("Cam bien quang 1 kich hoat - Dung di chuyen sang trai.");
    pcf8574.write(XILANH_KEP, LOW); // Kẹp xylanh
    delay(250);
    pcf8574.write(XILANH_DAY, HIGH); // Thả xylanh
    delay(250);
    sendControlCMD(ControlCMD{DC_XOAY, MOTOR_LEN, 100, 0});
    pcf8574.write(XILANH_DAY, HIGH); // Thả xylanh
    sendControlCMD(ControlCMD{MECANUM_MOTOR, UP, 100, 1500});
    Serial.println("Di chuyển lên...");
    while (getControlCMD(MECANUM_MOTOR).distance > 0) // Đợi đến khi hoàn thành lệnh di chuyển
    {
        delay(50);
    }
    while (digitalRead(PIN_CTHT_TREN_DONGCO_KEP) == HIGH)
        ;                                                  // Đợi đến khi cảm biến CTHT trên động cơ kẹp được kích hoạt
    sendControlCMD(ControlCMD{DC_XOAY, MOTOR_STOP, 0, 0}); // Dừng động cơ

    sendControlCMD(ControlCMD{MECANUM_MOTOR, RO_RIGHT, 100, 180});
    Serial.println("Quay phải...");
    while (getControlCMD(MECANUM_MOTOR).distance > 0) // Đợi đến khi hoàn thành lệnh di chuyển
    {
        delay(50);
    }
    sendControlCMD(ControlCMD{MECANUM_MOTOR, LEFT, 100, 1000});
    Serial.println("Di chuyển sang trái...");
    while (getControlCMD(MECANUM_MOTOR).distance > 0) // Đợi đến khi hoàn thành lệnh di chuyển
    {
        delay(50);
    }
    tagReceived = false; // Reset trạng thái nhận tag để có thể nhận tag mới
    while (!tagReceived)
    {
        readUartFromPC(); // đọc liên tục cho đến khi có tag
        delay(10);
    }
    pcf8574.write(XILANH_KEP, HIGH); // Thả xylanh
    sendControlCMD(ControlCMD{DC_XOAY, MOTOR_XUONG, 100, 0});
    while (digitalRead(PIN_CTHT_DUOI_DONGCO_KEP) == HIGH)
        ;                                                  // Đợi đến khi cảm biến CTHT dưới động cơ kẹp được kích hoạt
    sendControlCMD(ControlCMD{DC_XOAY, MOTOR_STOP, 0, 0}); // Dừng động cơ
} // sendControlCMD(ControlCMD{OMNI_MOTOR, CROSS_UP_RIGHT, 100, 20000});
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