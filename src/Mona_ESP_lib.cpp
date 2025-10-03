/*
  Mona_ESP_lib.cpp - Implementation of the library for Mona ESP robot
  Created by Bart Garcia, November 2020.
  bart.garcia.nathan@gmail.com
  Released into the public domain.
*/
#include "Mona_ESP_lib.h"
#include "Adafruit_MCP23008.h"
#include <Adafruit_NeoPixel.h>

/* ----Library functions implementation for Mona ESP in C style----*/
//Initialize global objects
Adafruit_MCP23008 IO_expander;  // GPIO Expander
ADS7830 ADC;
Adafruit_NeoPixel RGB1(1, LED_RGB1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel RGB2(1, LED_RGB2, NEO_GRB + NEO_KHZ800);
Adafruit_LSM9DS1 IMU = Adafruit_LSM9DS1();

//Global variables
sensors_event_t IMU_a, IMU_m, IMU_g, IMU_temp;

// === Encoder counters & direction hints ===
static volatile int32_t enc_l = 0;
static volatile int32_t enc_r = 0;
// 전진 +1, 후진 -1 (스핀 시 좌/우 반대 부호를 설정)
static volatile int8_t dir_l = +1;
static volatile int8_t dir_r = +1;

// === encoder ISRs (단일 채널 카운트) ===
static void IRAM_ATTR isr_left() { enc_l += dir_l; }
static void IRAM_ATTR isr_right() { enc_r += dir_r; }

void Encoders_begin() {
    // ESP32 34~39 입력은 내부 풀업이 약하므로, 하드웨어 풀업 권장.
    pinMode(Mot_left_feedback, INPUT_PULLUP);
    pinMode(Mot_right_feedback, INPUT_PULLUP);
    pinMode(Mot_left_feedback_2, INPUT_PULLUP);
    pinMode(Mot_right_feedback_2, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(Mot_left_feedback), isr_left, RISING);
    attachInterrupt(digitalPinToInterrupt(Mot_right_feedback), isr_right, RISING);
}

void Encoders_reset() { enc_l = 0; enc_r = 0; }
int32_t Encoder_left_count() { return enc_l; }
int32_t Encoder_right_count() { return enc_r; }

//Mona Init function - setup pinModes
void Mona_ESP_init(void) {
    //Set PinModes
    pinMode(Mot_right_forward, OUTPUT);
    pinMode(Mot_right_backward, OUTPUT);
    pinMode(Mot_left_forward, OUTPUT);
    pinMode(Mot_left_backward, OUTPUT);

    pinMode(Mot_right_feedback, INPUT);
    pinMode(Mot_right_feedback_2, INPUT);
    pinMode(Mot_left_feedback, INPUT);
    pinMode(Mot_left_feedback_2, INPUT);

    pinMode(LED_RGB1, OUTPUT);
    pinMode(LED_RGB2, OUTPUT);

    //Setup PWM channels for motors (ESP32 core)
    analogWriteFrequency(Mot_freq);   // 전역 1회
    analogWriteResolution(Mot_res);   // 전역 1회

    //Turn off the Motor
    analogWrite(Mot_right_forward, 0);
    analogWrite(Mot_right_backward, 0);
    analogWrite(Mot_left_forward, 0);
    analogWrite(Mot_left_backward, 0);

    //Initialize I2C pins
    Wire.begin(SDA, SCL);

    //Initialize GPIO Expander
    IO_expander.begin(); // use default address 0
    IO_expander.pinMode(IR_enable_5, OUTPUT);
    IO_expander.pinMode(IR_enable_4, OUTPUT);
    IO_expander.pinMode(IR_enable_3, OUTPUT);
    IO_expander.pinMode(IR_enable_2, OUTPUT);
    IO_expander.pinMode(IR_enable_1, OUTPUT);
    IO_expander.pinMode(exp_5, INPUT);
    IO_expander.pinMode(exp_6, INPUT);
    IO_expander.pinMode(exp_7, INPUT);
    //Turn off all outputs
    IO_expander.digitalWrite(IR_enable_5, LOW);
    IO_expander.digitalWrite(IR_enable_4, LOW);
    IO_expander.digitalWrite(IR_enable_3, LOW);
    IO_expander.digitalWrite(IR_enable_2, LOW);
    IO_expander.digitalWrite(IR_enable_1, LOW);

    //Initialise ADC ADS7830
    ADC.getAddr_ADS7830(ADS7830_DEFAULT_ADDRESS); // 0x48
    ADC.setSDMode(SDMODE_SINGLE);                 // Single-Ended Inputs
    ADC.setPDMode(PDIROFF_ADON);                  // Internal Ref OFF, ADC ON

    // Encoders
    Encoders_begin();
    Encoders_reset();

    //Initialize IMU and set up (IMU는 지금 사용 안 함. 유지만.)
    if (!IMU.begin()) {
        Serial.println("Unable to initialize the LSM9DS1");
    }
    IMU.setupAccel(IMU.LSM9DS1_ACCELRANGE_2G);
    IMU.setupMag(IMU.LSM9DS1_MAGGAIN_4GAUSS);
    IMU.setupGyro(IMU.LSM9DS1_GYROSCALE_245DPS);

    // Battery ADC
    analogSetAttenuation(ADC_0db); // input attenuation for ALL ADC inputs

    //Initialize WS2812B LED driver constructor
    RGB1.begin(); RGB1.clear(); RGB1.show();
    RGB2.begin(); RGB2.clear(); RGB2.show();
}

//Right Motor
void Right_mot_forward(int speed) {
    if (speed > 255) speed = 255;
    if (speed < 0)   speed = 0;
    dir_r = +1; // 방향 힌트
    analogWrite(Mot_right_forward, speed);
    analogWrite(Mot_right_backward, 0);
}

void Right_mot_backward(int speed) {
    if (speed > 255) speed = 255;
    if (speed < 0)   speed = 0;
    dir_r = -1; // 방향 힌트
    analogWrite(Mot_right_forward, 0);
    analogWrite(Mot_right_backward, speed);
}

void Right_mot_stop(void) {
    analogWrite(Mot_right_forward, 0);
    analogWrite(Mot_right_backward, 0);
}

//Left Motor
void Left_mot_forward(int speed) {
    if (speed > 255) speed = 255;
    if (speed < 0)   speed = 0;
    dir_l = +1; // 방향 힌트
    analogWrite(Mot_left_forward, speed);
    analogWrite(Mot_left_backward, 0);
}

void Left_mot_backward(int speed) {
    if (speed > 255) speed = 255;
    if (speed < 0)   speed = 0;
    dir_l = -1; // 방향 힌트
    analogWrite(Mot_left_forward, 0);
    analogWrite(Mot_left_backward, speed);
}

void Left_mot_stop(void) {
    analogWrite(Mot_left_forward, 0);
    analogWrite(Mot_left_backward, 0);
}

//Both motors (legacy helpers)
void Motors_forward(int speed) {
    Right_mot_forward(speed);
    Left_mot_forward(speed);
}
void Motors_backward(int speed) {
    Right_mot_backward(speed);
    Left_mot_backward(speed);
}
void Motors_spin_left(int speed) {
    // 좌스핀: 우전/좌후
    Right_mot_forward(speed);
    Left_mot_backward(speed);
}
void Motors_spin_right(int speed) {
    // 우스핀: 우후/좌전
    Right_mot_backward(speed);
    Left_mot_forward(speed);
}
void Motors_stop(void) {
    Right_mot_stop();
    Left_mot_stop();
}

// === 좌/우 개별 PWM (차동 구동) ===
void Motors_forward_lr(int left_pwm, int right_pwm) {
    left_pwm = constrain(left_pwm, -255, 255);
    right_pwm = constrain(right_pwm, -255, 255);
    if (left_pwm >= 0) Left_mot_forward(left_pwm);
    else               Left_mot_backward(-left_pwm);
    if (right_pwm >= 0) Right_mot_forward(right_pwm);
    else                Right_mot_backward(-right_pwm);
}

//IR sensors
void Enable_IR(int IR_number) {
    if (IR_number >= 1 && IR_number < 6) {
        if (IR_number == 1) IO_expander.digitalWrite(IR_enable_1, HIGH);
        if (IR_number == 2) IO_expander.digitalWrite(IR_enable_2, HIGH);
        if (IR_number == 3) IO_expander.digitalWrite(IR_enable_3, HIGH);
        if (IR_number == 4) IO_expander.digitalWrite(IR_enable_4, HIGH);
        if (IR_number == 5) IO_expander.digitalWrite(IR_enable_5, HIGH);
    }
}
void Disable_IR(int IR_number) {
    if (IR_number >= 1 && IR_number < 6) {
        if (IR_number == 1) IO_expander.digitalWrite(IR_enable_1, LOW);
        if (IR_number == 2) IO_expander.digitalWrite(IR_enable_2, LOW);
        if (IR_number == 3) IO_expander.digitalWrite(IR_enable_3, LOW);
        if (IR_number == 4) IO_expander.digitalWrite(IR_enable_4, LOW);
        if (IR_number == 5) IO_expander.digitalWrite(IR_enable_5, LOW);
    }
}
int Read_IR(int IR_number) {
    if (IR_number >= 1 && IR_number < 6) {
        if (IR_number == 1) return ADC.Measure_SingleEnded(IR1_sensor);
        if (IR_number == 2) return ADC.Measure_SingleEnded(IR2_sensor);
        if (IR_number == 3) return ADC.Measure_SingleEnded(IR3_sensor);
        if (IR_number == 4) return ADC.Measure_SingleEnded(IR4_sensor);
        if (IR_number == 5) return ADC.Measure_SingleEnded(IR5_sensor);
    }
    return 0;
}
int Get_IR(int IR_number) {
    if (IR_number < 1 || IR_number>5) return 0;
    uint8_t dark_val = Read_IR(IR_number);
    Enable_IR(IR_number); delay(1);
    uint8_t light_val = Read_IR(IR_number);
    Disable_IR(IR_number);
    return (uint8_t)abs(dark_val - light_val);
}
bool Detect_object(int IR_number, int threshold) {
    if (IR_number < 1 || IR_number>5) return false;
    return Get_IR(IR_number) > threshold;
}

//Battery Voltage
int Batt_Vol(void) {
    int adc = analogRead(Batt_Vol_pin);
    int bat_percentage = (adc - 2750) / 8;
    if (bat_percentage > 100) bat_percentage = 100;
    if (bat_percentage < 0)   bat_percentage = 0;
    return bat_percentage;
}

//LEDS control
void Set_LED(int Led_number, int Red, int Green, int Blue) {
    uint32_t color = ((uint32_t)Red << 16) | ((uint32_t)Green << 8) | Blue;
    if (Led_number == 1) { RGB1.fill(color, 0, 1); RGB1.show(); }
    if (Led_number == 2) { RGB2.fill(color, 0, 1); RGB2.show(); }
}

// Read sensors (IMU는 현재 회전에 사용하지 않음)
void IMU_read_sensors(sensors_event_t* a, sensors_event_t* m, sensors_event_t* g, sensors_event_t* temp) {
    IMU.read();
    IMU.getEvent(a, m, g, temp);
}

/* ===== HIGH-LEVEL: 고정 상수 기반 펄스 제어 =====
   - WhyCon/IMU 없이 엔코더 펄스로만 제어
   - 거리/각도 환산은 '고정 상수' 사용 (네가 직접 맞추는 값)
   - 직진은 좌/우 쏠림 PI만 유지
*/
static const float PULSES_PER_MM = 18.5f; // 
static const float PULSES_PER_DEG = 12.80f; // 

void Mona_drive_mm(float dist_mm, int base_pwm) {
    Encoders_reset();
    const int32_t target = (int32_t)(fabs(dist_mm) * PULSES_PER_MM);

    // 진행 방향 설정
    bool forward = (dist_mm >= 0);
    base_pwm = forward ? abs(base_pwm) : -abs(base_pwm);

    // 좌우 쏠림 보정(간단 PI)
    float kp = 0.8f, ki = 0.02f, ie = 0.0f;

    while ((abs(Encoder_left_count()) + abs(Encoder_right_count())) / 2 < target) {
        int l = Encoder_left_count();
        int r = Encoder_right_count();

        int lprog = forward ? l : -l;
        int rprog = forward ? r : -r;

        int diff = lprog - rprog;   // +면 좌가 더 빠름
        ie += diff;

        int corr = (int)(kp * diff + ki * ie);
        int lpwm = base_pwm - corr;
        int rpwm = base_pwm + corr;

        lpwm = constrain(lpwm, -255, 255);
        rpwm = constrain(rpwm, -255, 255);

        // 스틱션 최소 PWM
        if (abs(lpwm) < 60 && lpwm != 0) lpwm = (lpwm > 0) ? 60 : -60;
        if (abs(rpwm) < 60 && rpwm != 0) rpwm = (rpwm > 0) ? 60 : -60;

        Motors_forward_lr(lpwm, rpwm);
        delay(10);
    }
    Motors_stop();

}

void Mona_spin_deg(float yaw_deg, int pwm) {
    Encoders_reset();
    bool cw = (yaw_deg >= 0);
    const int32_t target = (int32_t)(fabs(yaw_deg) * PULSES_PER_DEG);

    // 좌우 진행 균형만 간단히 보정
    float kbal = 0.5f;

    while (true) {
        int l = Encoder_left_count();
        int r = Encoder_right_count();

        int32_t sum = (abs(l) + abs(r)) / 2;        // 진행량
        int bal = (int)(kbal * ((int)abs(l) - (int)abs(r))); // 좌우 균형

        if (sum >= target) break;

        int lp = constrain(pwm - bal, 0, 255);
        int rp = constrain(pwm + bal, 0, 255);

        if (cw) Motors_forward_lr(+lp, -rp);  // 우회전: 좌F, 우B
        else    Motors_forward_lr(-lp, +rp);  // 좌회전: 좌B, 우F
        delay(10);
    }
    Motors_stop();

}
