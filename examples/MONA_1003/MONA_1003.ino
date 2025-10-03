#include <WiFi.h>
#include "Mona_ESP_lib.h"   // 모터/IR/LED 등 하드웨어 추상화

// ===================== WiFi =====================
const char* ssid     = "Young";
const char* password = "gh40998855%";
WiFiServer wifiServer(8080);
WiFiClient client;

// ===================== 상태 정의 =====================
enum RobotState {
  STATE_IDLE,       // 명령 대기
  STATE_TURNING,    // G 회전 실행
  STATE_MOVING,     // G 직진 실행
  STATE_AVOID,      // 장애물 회피(스핀 등)
  STATE_ESCAPING,   // 회피 후 안전 공간 확보(짧은 전진, 논블로킹)
  STATE_EMERGENCY   // 비상 회피(강제 스핀)
};
RobotState state = STATE_IDLE;

// ===================== 펄스/엔코더 =====================
static const float PULSES_PER_MM     = 18.5f;   // 1mm당 펄스 수
static const float PULSES_PER_DEGREE = 12.8f;   // 1도당 펄스 수

volatile long left_encoder_count  = 0;
volatile long right_encoder_count = 0;

void IRAM_ATTR isr_left_encoder()  { left_encoder_count++; }
void IRAM_ATTR isr_right_encoder() { right_encoder_count++; }

inline long avg_pulses_since(long l0, long r0) {
  long l = labs(left_encoder_count  - l0);
  long r = labs(right_encoder_count - r0);
  return (l + r) / 2;
}

// ===================== PI (직진 보정; 필요 시 토글) =====================
static const int   FWD_SPD   = 100;
static const int   TURN_SPD  = 100;

#define USE_P   1     // 직진에서 P 사용: 1=ON
#define USE_PI  1     // 직진에서 PI 사용: 1=ON (P와 함께)

static const float P_GAIN_MOVE = 0.8f;
static const float I_GAIN_MOVE = 0.02f;

static long  start_left_count  = 0;
static long  start_right_count = 0;
static long  target_turn_pulses = 0;
static long  target_move_pulses = 0;
static float integral_error = 0.0f;

// ===================== IR/회피 파라미터 =====================
static const int TH        = 60;
static const int TH_OUTER  = 100;
static const int DELTA     = 10;

static const unsigned long EMERGENCY_SPIN_MS = 400;
static const unsigned long BACK_MS           = 120;

bool emergency_active = false;
unsigned long emergency_until = 0;
int last_turn_direction = 0;
int turn_change_count   = 0;
static const unsigned long OSCILLATION_WINDOW_MS = 1200;
static const int OSCILLATION_COUNT_THRESHOLD = 4;
unsigned long oscillation_timer_start = 0;

// ===================== ESCAPING (논블로킹 & 즉시 프리엠션) =====================
static const uint16_t ESCAPING_MS = 150;  // 150~250ms 권장
unsigned long escaping_until_ms = 0;

inline void enter_escaping(uint16_t ms = ESCAPING_MS) {
  escaping_until_ms = millis() + ms;
  Motors_forward(FWD_SPD);
  state = STATE_ESCAPING;
}

// ===================== G/STOP 파서 & 큐 =====================
char rxBuf[128];
int  rxLen = 0;

struct PendingG { bool has=false; float deg=0; float mm=0; } pendingG;

void clear_motion_targets() {
  target_turn_pulses = 0;
  target_move_pulses = 0;
  start_left_count  = left_encoder_count;
  start_right_count = right_encoder_count;
  integral_error = 0.0f;
}

void queue_g(float angle_deg, float dist_mm) {
  pendingG.has = true;
  pendingG.deg = angle_deg;
  pendingG.mm  = dist_mm;
  if (client) client.println("ACK G queued");
}

void start_g_now(float angle_deg, float dist_mm) {
  target_turn_pulses = (long)lroundf(fabs(angle_deg) * PULSES_PER_DEGREE);
  target_move_pulses = (long)lroundf(fabs(dist_mm)  * PULSES_PER_MM);
  start_left_count  = left_encoder_count;
  start_right_count = right_encoder_count;
  integral_error = 0.0f;

  if (target_turn_pulses > 0) {
    state = STATE_TURNING;
    if (angle_deg > 0) Motors_spin_right(TURN_SPD);
    else               Motors_spin_left(TURN_SPD);
  } else if (target_move_pulses > 0) {
    state = STATE_MOVING;
  } else {
    if (client) client.println("OK G (no movement)");
    state = STATE_IDLE;
  }
}

void maybe_start_queued_g() {
  if (!pendingG.has) return;
  float angle_deg = pendingG.deg;
  float dist_mm   = pendingG.mm;
  pendingG.has = false;
  start_g_now(angle_deg, dist_mm);
}

void handle_G_line(char* line) {
  float angle_deg=0, dist_mm=0;
  if (sscanf(line + 1, "%f %f", &angle_deg, &dist_mm) != 2) {
    if (client) client.println("ERR: G-code parse error");
    return;
  }
  if (state == STATE_IDLE) start_g_now(angle_deg, dist_mm);
  else                     queue_g(angle_deg, dist_mm);
}

void handle_STOP() {
  Motors_stop();
  clear_motion_targets();
  pendingG.has = false;
  state = STATE_IDLE;
  if (client) client.println("OK STOP");
}

// ===================== 비상/진동 억제 보조 =====================
inline void start_emergency_left_spin() {
  Motors_backward(FWD_SPD); delay(BACK_MS);  // 짧은 백업 허용
  Motors_spin_left(300);
  emergency_active = true;
  emergency_until = millis() + EMERGENCY_SPIN_MS;
  turn_change_count = 0; last_turn_direction = -1;
}

void check_oscillation_and_escape(int current_direction) {
  if (last_turn_direction != 0 && current_direction != last_turn_direction) {
    unsigned long now = millis();
    if (now - oscillation_timer_start > OSCILLATION_WINDOW_MS) {
      turn_change_count = 1; oscillation_timer_start = now;
    } else {
      turn_change_count++;
    }
    if (turn_change_count >= OSCILLATION_COUNT_THRESHOLD) {
      start_emergency_left_spin();
    }
  }
  last_turn_direction = current_direction;
}

// ===================== setup =====================
void setup() {
  Serial.begin(115200);
  Mona_ESP_init();

  attachInterrupt(digitalPinToInterrupt(35), isr_left_encoder,  RISING);
  attachInterrupt(digitalPinToInterrupt(39), isr_right_encoder, RISING);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis()-t0 < 8000) { delay(200); }
  wifiServer.begin();

  Set_LED(0, 0, 12, 12); // 파란 LED: 준비 완료
}

// ===================== loop =====================
void loop() {
  // --- TCP 연결 수락 ---
  if (!client || !client.connected()) {
    WiFiClient c = wifiServer.available();
    if (c) { client = c; client.setTimeout(1); Serial.println("Client connected"); }
  }

  // --- 수신 처리 ---
  if (client && client.available()) {
    char c = client.read();
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) {
        rxBuf[rxLen] = '\0';
        if (rxBuf[0] == 'G' || rxBuf[0] == 'g')      handle_G_line(rxBuf);
        else if (strncasecmp(rxBuf, "STOP", 4) == 0) handle_STOP();
        rxLen = 0;
      }
    } else if (rxLen < (int)sizeof(rxBuf)-1) {
      rxBuf[rxLen++] = c;
    }
  }

  // --- IR 읽기 ---
  int r1 = Get_IR(1), r2 = Get_IR(2), r3 = Get_IR(3), r4 = Get_IR(4), r5 = Get_IR(5);

  // --- 비상 스핀 유지/해제 ---
  if (emergency_active) {
    if (millis() < emergency_until) {
      Motors_spin_left(300);
      delay(2);
      return;
    } else {
      emergency_active = false;
      Motors_stop();
      state = STATE_IDLE;
      if (client) client.println("INFO: Avoidance complete. Ready for new command.");
    }
  }

  // --- 회피 최우선 전역 프리엠션 ---
  bool obstacle = (r1 > TH_OUTER) || (r2 > TH) || (r3 > TH) || (r4 > TH) || (r5 > TH_OUTER);
  if (obstacle && state != STATE_AVOID) {
    Motors_stop();
    integral_error = 0.0f;           // 직진 적분 리셋
    state = STATE_AVOID;
    if (client) client.println("WARN: obstacle detected -> AVOID (preempt)");
  }

  // --- 상태 머신 ---
  switch (state) {

    case STATE_IDLE:
      // 새 G가 있으면 여기서만 시작
      maybe_start_queued_g();
      break;

    case STATE_TURNING: {
      long avg = avg_pulses_since(start_left_count, start_right_count);
      if (avg >= target_turn_pulses) {
        Motors_stop();
        if (target_move_pulses > 0) {
          state = STATE_MOVING;
          start_left_count  = left_encoder_count;
          start_right_count = right_encoder_count;
          integral_error = 0.0f;
        } else {
          if (client) client.println("OK G");
          state = STATE_IDLE;
          maybe_start_queued_g();
        }
      }
      break;
    }

    case STATE_MOVING: {
      long l_now = labs(left_encoder_count  - start_left_count);
      long r_now = labs(right_encoder_count - start_right_count);
      long avg   = (l_now + r_now) / 2;

      if (avg >= target_move_pulses) {
        Motors_stop();
        if (client) client.println("OK G");
        state = STATE_IDLE;
        maybe_start_queued_g();
      } else {
        // --- 직진 쏠림 보정 (옵션) ---
#if (USE_P || USE_PI)
        long err = l_now - r_now;        // +: 왼쪽이 더 굴렀음(오른쪽 PWM↑)
#if USE_PI
        integral_error += err;
#endif
        float u = P_GAIN_MOVE * (float)err
#if USE_PI
                + I_GAIN_MOVE * (float)integral_error
#endif
                ;
        int left_pwm  = constrain((int)lroundf(FWD_SPD - u), 60, 255);
        int right_pwm = constrain((int)lroundf(FWD_SPD + u), 60, 255);
        Left_mot_forward(left_pwm);
        Right_mot_forward(right_pwm);
#else
        Motors_forward(FWD_SPD);         // 보정 OFF(펄스만)
#endif
      }
      break;
    }

    case STATE_AVOID: {
      // 간단한 스핀 회피 정책 (중앙/대각/바깥 IR)
      bool all_clear = (r1 <= TH_OUTER) && (r2 <= TH) && (r3 <= TH) && (r4 <= TH) && (r5 <= TH_OUTER);
      if (all_clear) {
        enter_escaping(ESCAPING_MS);     // 안전 공간 확보 시작 (논블로킹)
      } else {
        if (r3 >= TH) {
          int ldiag = r2, rdiag = r4;
          if (abs(ldiag - rdiag) <= DELTA || ldiag < rdiag) {
            Motors_spin_left(TURN_SPD);  check_oscillation_and_escape(-1);
          } else {
            Motors_spin_right(TURN_SPD); check_oscillation_and_escape(+1);
          }
        } else if (r1 >= TH || r2 >= TH) {
          Motors_spin_right(TURN_SPD);   check_oscillation_and_escape(+1);
        } else if (r4 >= TH || r5 >= TH) {
          Motors_spin_left(TURN_SPD);    check_oscillation_and_escape(-1);
        }
      }
      break;
    }

    case STATE_ESCAPING: {
      // ESCAPING 중에도 회피 1순위: IR 재검사 → 즉시 AVOID 재진입
      int rr1 = r1, rr2 = r2, rr3 = r3, rr4 = r4, rr5 = r5;   // 이미 읽은 값을 사용(원하면 재측정 가능)
      bool obstacle_again = (rr1 > TH_OUTER) || (rr2 > TH) || (rr3 > TH) || (rr4 > TH) || (rr5 > TH_OUTER);
      if (obstacle_again) {
        Motors_stop();
        integral_error = 0.0f;
        state = STATE_AVOID;
        if (client) client.println("WARN: obstacle re-detected during ESCAPING -> re-avoid");
        break;
      }

      // 평소 종료(타이머 만료)
      if ((int32_t)(millis() - escaping_until_ms) >= 0) {
        Motors_stop();
        state = STATE_IDLE;
        if (client) client.println("INFO: Avoidance complete. Ready for new command.");
        maybe_start_queued_g();  // 바로 다음 G 실행(원하면 유지)
      }
      break;
    }

    case STATE_EMERGENCY:
    default:
      // (EMERGENCY는 emergency_active로 상단에서 처리)
      break;
  }

  delay(3); // 짧은 휴식(논블로킹 유지)
}
