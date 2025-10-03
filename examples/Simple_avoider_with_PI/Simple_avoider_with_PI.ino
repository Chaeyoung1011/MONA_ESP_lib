#include <Wire.h>
#include "Mona_ESP_lib.h" // 제공된 라이브러리 파일

// =================================================================
// ===== 1. 상태 머신, PI 제어, 펄스 카운팅을 위한 전역 변수 추가 =====
// =================================================================

// 로봇의 상태를 정의
enum RobotState {
  STATE_FORWARD_PI,
  STATE_AVOID
};
RobotState currentState = STATE_FORWARD_PI; // 시작 상태는 PI 제어 직진

// 펄스 카운팅 변수 (volatile: 인터럽트에서 사용)
volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;

// PI 제어 관련 파라미터 및 변수
static const float kp = 0.8f;
static const float ki = 0.02f;
static float integral_error = 0.0f;
static long prev_left_count = 0;
static long prev_right_count = 0;
static unsigned long prev_pi_time = 0;

// ---- 기존 튜닝 파라미터 ----
static const int TH = 60;
static const int TH_OUTER = 100;
static const int DELTA = 10;
static const int FWD_SPD = 100;
static const int TURN_SPD = 100;

// ---- EMERGENCY(끼임) 탈출 파라미터 ----
static const unsigned long EMERGENCY_SPIN_MS = 400;
static const unsigned long BACK_MS = 120;
bool emergency_active = false;
unsigned long emergency_until = 0;

// ---- 진동(OSCILLATION) 감지 파라미터 ----
static const unsigned long OSCILLATION_WINDOW_MS = 1200;
static const int OSCILLATION_COUNT_THRESHOLD = 4;
int last_turn_direction = 0;
int turn_change_count = 0;
unsigned long oscillation_timer_start = 0;

// =================================================================
// ===== 2. 엔코더 인터럽트 서비스 루틴(ISR) 추가 =====
// =================================================================
void IRAM_ATTR isr_left_encoder() {
  left_encoder_count++;
}
void IRAM_ATTR isr_right_encoder() {
  right_encoder_count++;
}


// =================================================================
// ===== 기존 함수들 (비상 탈출, 진동 감지) - 수정 없음 =====
// =================================================================
inline void start_emergency_left_spin() {
  Motors_backward(FWD_SPD);
  delay(BACK_MS);
  Motors_spin_left(300);
  emergency_active = true;
  emergency_until = millis() + EMERGENCY_SPIN_MS;
  turn_change_count = 0;
  last_turn_direction = -1;
}

void check_oscillation_and_escape(int current_direction) {
  if (last_turn_direction != 0 && current_direction != last_turn_direction) {
    unsigned long current_time = millis();
    if (current_time - oscillation_timer_start > OSCILLATION_WINDOW_MS) {
      turn_change_count = 1;
      oscillation_timer_start = current_time;
    } else {
      turn_change_count++;
    }
    if (turn_change_count >= OSCILLATION_COUNT_THRESHOLD) {
      Serial.println("\n*** OSCILLATION DETECTED! -> EMERGENCY ESCAPE ***\n");
      start_emergency_left_spin();
      return; 
    }
  }
  last_turn_direction = current_direction;
}

// =================================================================
// ===== 3. setup() 함수에 인터럽트 연결 코드 추가 =====
// =================================================================
void setup() {
  Serial.begin(115200);
  Mona_ESP_init();

  // 엔코더 핀을 인터럽트에 연결
  // Mot_left_feedback: 35, Mot_right_feedback: 39 (Mona_ESP_lib.h 참조)
  attachInterrupt(digitalPinToInterrupt(35), isr_left_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(39), isr_right_encoder, RISING);
  
  prev_pi_time = millis();
  Serial.println("State Machine Avoider with PI Control");
}

// =================================================================
// ===== 4. loop() 함수를 상태 머신 구조로 재구성 =====
// =================================================================
void loop() {
  // RAW 읽기 (모든 상태에서 공통으로 사용)
  int r[5];
  for (int i = 0; i < 5; ++i) r[i] = Get_IR(i + 1);

  // --- 비상 탈출 로직 (최상위 우선순위) ---
  if (emergency_active) {
    if (millis() < emergency_until) {
      Motors_spin_left(300); // 비상 회전 유지
      return;
    } else {
      emergency_active = false; // 비상 상태 해제
      Motors_stop();
    }
  }

  // --- 상태 머신 시작 ---
  switch (currentState) {
    
    case STATE_FORWARD_PI: {
      // [상태 전환 조건] 장애물 감지 시 AVOID 상태로 변경
      bool obstacle_detected = (r[0] > TH_OUTER) || (r[1] > TH) || (r[2] > TH) || (r[3] > TH) || (r[4] > TH_OUTER);
      
      if (obstacle_detected) {
        Motors_stop();
        currentState = STATE_AVOID;
        // 회피 시작 전 진동 감지 변수 초기화
        last_turn_direction = 0;
        turn_change_count = 0;
        return;
      }

      // [현재 상태 동작] PI 제어로 직진 (주기적으로 실행)
      if (millis() - prev_pi_time > 20) { // 20ms 마다 PI 제어 실행
        long current_left = left_encoder_count;
        long current_right = right_encoder_count;
        
        // 시간당 펄스 변화량 계산 (속도 개념)
        long diff_left = current_left - prev_left_count;
        long diff_right = current_right - prev_right_count;

        int error = diff_left - diff_right;
        integral_error += error;

        int correction = (int)(kp * error + ki * integral_error);

        int left_pwm = FWD_SPD - correction;
        int right_pwm = FWD_SPD + correction;

        left_pwm = constrain(left_pwm, 0, 255);
        right_pwm = constrain(right_pwm, 0, 255);
        
        Left_mot_forward(left_pwm);
        Right_mot_forward(right_pwm);
        
        prev_left_count = current_left;
        prev_right_count = current_right;
        prev_pi_time = millis();
      }
      break;
    }

    case STATE_AVOID: {
      // [상태 전환 조건] 모든 경로가 깨끗해지면 FORWARD_PI 상태로 복귀
      bool all_clear = (r[0] <= TH_OUTER) && (r[1] <= TH) && (r[2] <= TH) && (r[3] <= TH) && (r[4] <= TH_OUTER);

      if (all_clear) {
        Motors_stop();
        currentState = STATE_FORWARD_PI;
        // PI 제어 변수들을 현재 상태 기준으로 초기화
        prev_left_count = left_encoder_count;
        prev_right_count = right_encoder_count;
        integral_error = 0;
        return;
      }
      
      // [현재 상태 동작] 기존의 회피 로직 수행
      if (r[2] >= TH) {
        int diff24 = abs(r[1] - r[3]);
        if (diff24 <= DELTA) {
          Motors_spin_left(TURN_SPD);
          check_oscillation_and_escape(-1);
        } else {
          if (r[1] < r[3]) { Motors_spin_left(TURN_SPD); check_oscillation_and_escape(-1); } 
          else { Motors_spin_right(TURN_SPD); check_oscillation_and_escape(1); }
        }
      } else if (r[0] >= TH || r[1] >= TH) {
        Motors_spin_right(TURN_SPD);
        check_oscillation_and_escape(1);
      } else if (r[3] >= TH || r[4] >= TH) {
        Motors_spin_left(TURN_SPD);
        check_oscillation_and_escape(-1);
      }
      break;
    }
  } // switch 끝
}