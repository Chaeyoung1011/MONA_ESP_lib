#include <Wire.h>
#include "Mona_ESP_lib.h"

// ---- 튜닝 파라미터 ----
static const int TH = 70;       // 기본 임계(raw)
static const int TH_OUTER = 100;    // 1번, 5번(바깥쪽) 해제 임계
static const int DELTA = 15;      // 2번-4번 차이 데드밴드
static const int FWD_SPD = 100;
static const int TURN_SPD = 100;

// ---- EMERGENCY(끼임) 탈출 파라미터 ----
static const unsigned long EMERGENCY_SPIN_MS = 400; // 최소 왼쪽 스핀 유지(ms)
static const unsigned long BACK_MS = 120;       // 진입 직전 짧은 후진(ms)

bool emergency_active = false;
unsigned long emergency_until = 0;

static const unsigned long OSCILLATION_WINDOW_MS = 1200; // 진동 감지 시간
static const int OSCILLATION_COUNT_THRESHOLD = 4;     // 이 횟수 이상 방향 전환 시 비상 탈출

int last_turn_direction = 0; // -1: 좌회전, 0: 직진/정지, 1: 우회전
int turn_change_count = 0;   // 방향 전환 횟수 카운터
unsigned long oscillation_timer_start = 0; // 진동 감지 윈도우 시작 시간

// =================================================================

// 비상 탈출 로직 (왼쪽 스핀)
inline void start_emergency_left_spin() {
  // 백오프: 벽에서 살짝 떨어져 공간 확보
  Motors_backward(FWD_SPD);
  delay(BACK_MS);

  // 최소 EMERGENCY_SPIN_MS 동안은 무조건 왼쪽 회전 유지
  Motors_spin_left(300);
  emergency_active = true;
  emergency_until = millis() + EMERGENCY_SPIN_MS;
  
  // 비상 탈출 후 진동 감지 변수 초기화
  turn_change_count = 0;
  last_turn_direction = -1; // 왼쪽으로 돌고 있으므로 -1로 설정
}

void check_oscillation_and_escape(int current_direction) {
  // 이전에 회전한 기록이 있고, 현재 방향과 반대일 경우
  if (last_turn_direction != 0 && current_direction != last_turn_direction) {
    unsigned long current_time = millis();

    // 진동 감지 시간이 지났으면 카운터 초기화
    if (current_time - oscillation_timer_start > OSCILLATION_WINDOW_MS) {
      turn_change_count = 1;
      oscillation_timer_start = current_time;
    } else {
      // 시간 안에 방향이 바뀌었으면 카운트 증가
      turn_change_count++;
    }

    // 설정된 횟수 이상 방향이 전환되었으면 비상 탈출 실행
    if (turn_change_count >= OSCILLATION_COUNT_THRESHOLD) {
      Serial.println("\n*** OSCILLATION DETECTED! -> EMERGENCY ESCAPE ***\n");
      start_emergency_left_spin();
      return; 
    }
  }
  // 마지막 회전 방향을 현재 방향으로 업데이트
  last_turn_direction = current_direction;
}


void setup() {
  Serial.begin(115200);
  Mona_ESP_init();
  Serial.println("Simple RAW-based Avoider (oscillation detection only)");
}

void loop() {
  // RAW 읽기 (1..5: 왼쪽→오른쪽)
  int r[5];
  for (int i = 0; i < 5; ++i) r[i] = Get_IR(i + 1);

  // 디버깅 출력
  Serial.print("Raw: ");
  for (int i = 0; i < 5; ++i) { Serial.print(r[i]); Serial.print(" "); }

  // ---- EMERGENCY 동작 유지 및 해제 관리 ----

  // 비상 회전이 활성화되었고, 지정된 시간이 아직 안 끝났으면 계속 회전
  if (emergency_active && millis() < emergency_until) {
    Motors_spin_left(300);
    Serial.println(" | EMERGENCY COMMIT (spinning)");
    delay(5);
    return;
  }
  
  // 비상 회전 시간이 끝났으면, 비상 상태 플래그를 해제
  // (이 부분이 없으면 한번 비상상태 진입 후 절대 빠져나오지 못함)
  if (emergency_active && millis() >= emergency_until) {
    emergency_active = false;
  }

  // === 0. all_clear: 모든 센서가 안전할 때 ===
  bool all_clear =
    (r[0] <= TH_OUTER) && (r[1] <= TH) && (r[2] <= TH) && (r[3] <= TH) && (r[4] <= TH_OUTER);

  if (all_clear) {
    Motors_forward(FWD_SPD);
    last_turn_direction = 0; // 직진 상태이므로 방향 초기화
    turn_change_count = 0;   // 카운터 초기화
    Serial.println(" | FWD");
    delay(5);
    return;
  }

  // === 1. 가운데(3번) 회피 ===
  if (r[2] >= TH) {
    int diff24 = abs(r[1] - r[3]); // 2 vs 4
    if (diff24 <= DELTA) {
      Motors_spin_left(TURN_SPD);
      Serial.println(" | CENTER AVOID -> LEFT (diff<=DELTA)");
      check_oscillation_and_escape(-1);
    } else {
      if (r[1] < r[3]) {
        Motors_spin_left(TURN_SPD);
        Serial.println(" | CENTER AVOID -> LEFT (r2<r4)");
        check_oscillation_and_escape(-1);
      } else {
        Motors_spin_right(TURN_SPD);
        Serial.println(" | CENTER AVOID -> RIGHT (r4<r2)");
        check_oscillation_and_escape(1);
      }
    }
    delay(5);
    return;
  }

  // === 2. 왼쪽(1,2) 센서 감지 → 오른쪽 회전 ===
  if (r[0] >= TH || r[1] >= TH) {
    Motors_spin_right(TURN_SPD);
    Serial.println(" | L(1,2) AVOID -> RIGHT");
    check_oscillation_and_escape(1);
    delay(5);
    return;
  }

  // === 3. 오른쪽(4,5) 센서 감지 → 왼쪽 회전 ===
  if (r[3] >= TH || r[4] >= TH) {
    Motors_spin_left(TURN_SPD);
    Serial.println(" | R(4,5) AVOID -> LEFT");
    check_oscillation_and_escape(-1);
    delay(5);
    return;
  }

  // (위 조건에 아무것도 안 걸리면 안전 직진)
  Motors_forward(FWD_SPD);
  last_turn_direction = 0; // 직진 상태이므로 방향 초기화
  turn_change_count = 0;   // 카운터 초기화
  Serial.println(" | FWD(else)");
  delay(5);
}