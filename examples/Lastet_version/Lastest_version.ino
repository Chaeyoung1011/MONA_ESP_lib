// ===== Mona ESP (Wi-Fi G + 회피 우선, G 재개 없음 + 미세G합치기/10°룰) =====
#include "Mona_ESP_lib.h"
#include <WiFi.h>

// --- Wi-Fi ---
const char* ssid     = "Young";
const char* password = "gh40998855%";
const uint16_t SERVER_PORT = 8080;
WiFiServer wifiServer(SERVER_PORT);

// --- 주행/시간 튜닝 ---
static const float TURN_MS_PER_DEG = 14.0f;
static const float MOVE_MS_PER_MM  = 20.0f;
static const int   SPEED_TURN      = 100;   // 요구: 100
static const int   SPEED_FWD       = 100;   // 요구: 100
static const float MIN_DELTA_DEG   = 1.0f;
static const float MIN_DELTA_MM    = 5.0f;
static const uint32_t IDLE_TIMEOUT_MS = 8000;

// --- IR 회피 파라미터 ---
static const int TH        = 60;            // 2,3,4
static const int TH_OUTER  = 100;           // 1,5
static const int DELTA     = 15;            // |r2-r4| 데드밴드
static const unsigned long EMERGENCY_SPIN_MS = 400;  // 비상 커밋
static const unsigned long BACK_MS           = 120;  // 비상 전 후진
static const unsigned long OSCILLATION_WINDOW_MS = 1200;
static const int           OSCILLATION_COUNT_THRESHOLD = 4;

// --- 상태머신 ---
enum Phase { PH_IDLE=0, PH_TURN, PH_MOVE, PH_AVOID };
static Phase     phase = PH_IDLE;
static uint32_t  phase_end_ms = 0;          // 현재 단계 종료 시각
static float     queued_move_mm = 0.0f;     // 회전 후 이어질 직진(mm) — 회피 발동 시 즉시 폐기
static char      inflight_cmd = '\0';       // 진행 중 명령('G')
static int       current_turn_dir = 0;      // +1=우, -1=좌

// --- 회피/비상 ---
static bool      emergency_active = false;
static uint32_t  emergency_until  = 0;

// --- 진동(좌↔우) 감지 ---
static int       last_turn_direction = 0;   // -1 좌, 0 정지, +1 우
static int       turn_change_count   = 0;
static uint32_t  oscillation_timer_start = 0;

// --- 경로 상태 ---
static bool      path_clear = true;         // 1,5 ≤100 & 2,3,4 ≤60

// --- 입력 버퍼 ---
static char      rxBuf[128];
static int       rxLen = 0;
static uint32_t  lastRxMs = 0;

// ===== 미세 G 합치기(샘플러) + 10° 규칙 =====
static const uint32_t G_SCHEDULE_PERIOD_MS = 100; // 저주파 샘플링 주기
//static const float    ANGLE_EPS_DEG        = 6.0f; // 10도 이하이면 무시

// 펜딩(들어온 최신 G) 버퍼
static bool  pending_valid = false;
static float pending_yaw   = 0.0f;
static float pending_dist  = 0.0f;

// 현재 실행 중인(혹은 마지막으로 시작한) G의 목표를 기록
static bool  current_valid = false;
static float current_yaw   = 0.0f;  // 현재 G의 회전 목표 각
static float current_dist  = 0.0f;  // 현재 G의 직진 목표(mm)

// 스케줄러 타이밍
static uint32_t last_schedule_ms = 0;

// ===== 유틸 =====
static inline void stopMotors(){ Motors_forward(0); }

static inline void preempt_motion(){          // 현재 G 즉시 취소
  queued_move_mm = 0.0f;
  inflight_cmd   = '\0';
  current_turn_dir = 0;
  phase          = PH_IDLE;
  phase_end_ms   = 0;
  stopMotors();
  current_valid  = false;
}

// 회전 시작(도)
static inline void start_turn_deg_cw(float yaw_deg){
  phase_end_ms = millis() + (uint32_t)(fabs(yaw_deg) * TURN_MS_PER_DEG);
  phase = PH_TURN;
  if (yaw_deg >= 0){ Motors_spin_right(SPEED_TURN); current_turn_dir=+1; }
  else              { Motors_spin_left (SPEED_TURN); current_turn_dir=-1; }
  // 현재 목표 업데이트
  current_valid = true; current_yaw = yaw_deg; /* dist는 아래 queued로 이어짐 */
}

// 전진 시작(mm)
static inline void start_move_mm(float dist_mm){
  phase_end_ms = millis() + (uint32_t)(fabs(dist_mm) * MOVE_MS_PER_MM);
  phase = PH_MOVE;
  Motors_forward(SPEED_FWD);
  // 현재 목표 업데이트(회전은 이미 끝났을 수도)
  current_valid = true; current_dist = dist_mm;
}

// ===== 비상(백오프 + 좌스핀 커밋) =====
static inline void start_emergency_left_spin() {
  preempt_motion();                            // **G 즉시 취소**
  Motors_backward(SPEED_FWD); delay(BACK_MS);  // 공간 확보
  Motors_spin_left(300);                       // 좌스핀 커밋
  emergency_active = true;
  emergency_until  = millis() + EMERGENCY_SPIN_MS;
  turn_change_count = 0;
  last_turn_direction = -1;
  phase = PH_AVOID;                            // 회피 상태
}

// ===== 진동(좌↔우) 감지 → 비상 트리거 =====
static inline void check_oscillation_and_escape(int current_direction) {
  if (current_direction == 0) return;

  if (last_turn_direction != 0 && current_direction != last_turn_direction) {
    uint32_t now = millis();
    if (now - oscillation_timer_start > OSCILLATION_WINDOW_MS) {
      turn_change_count = 1; oscillation_timer_start = now;
    } else {
      turn_change_count++;
    }
    if (turn_change_count >= OSCILLATION_COUNT_THRESHOLD) {
      Serial.println("\n*** OSCILLATION DETECTED! -> EMERGENCY ESCAPE ***\n");
      start_emergency_left_spin();
      return;
    }
  } else if (last_turn_direction == 0) {
    oscillation_timer_start = millis();
    turn_change_count = 0;
  }
  last_turn_direction = current_direction;
}

// ===== 회피 틱(항상 G보다 우선, 재개 없음) =====
static inline void avoidance_tick(){
  // 비상 커밋 중이면 무조건 좌스핀 유지
  if (emergency_active){
    if (millis() < emergency_until){
      Motors_spin_left(300);
      path_clear = false;
      phase = PH_AVOID;
      return;
    } else {
      emergency_active = false; // 커밋 종료 → 아래에서 센서로 재판정
    }
  }

  // RAW 읽기
  int r[5]; for (int i=0;i<5;++i) r[i] = Get_IR(i+1);

  // 길 열림 판정: 1,5 ≤100 & 2,3,4 ≤60
  bool all_clear = (r[0] <= TH_OUTER) && (r[4] <= TH_OUTER)
                && (r[1] <= TH) && (r[2] <= TH) && (r[3] <= TH);

  if (all_clear){
    path_clear = true;
    // 회피가 끝났다면 **그냥 IDLE**. (재개 없음)
    if (phase == PH_AVOID){ stopMotors(); phase = PH_IDLE; }
    return;
  }

  // 여기 오면 막힘 → 회피가 선점
  path_clear = false;

  // 회피 시작 시점에 G가 돌고 있으면 즉시 취소
  if (phase == PH_TURN || phase == PH_MOVE){
    preempt_motion();              // **G 즉시 취소**
    phase = PH_AVOID;
  } else if (phase == PH_IDLE){
    phase = PH_AVOID;              // 명시적으로 회피 상태
  }

  // === 회피 규칙 ===
  if (r[2] >= TH) {                            // 정면
    int diff24 = abs(r[1] - r[3]);
    if (diff24 <= DELTA) {
      Motors_spin_left(SPEED_TURN);
      check_oscillation_and_escape(-1);
    } else {
      if (r[1] < r[3]) {
        Motors_spin_left(SPEED_TURN);
        check_oscillation_and_escape(-1);
      } else {
        Motors_spin_right(SPEED_TURN);
        check_oscillation_and_escape(+1);
      }
    }
    return;
  }
  if (r[0] >= TH || r[1] >= TH) {              // 좌측
    Motors_spin_right(SPEED_TURN);
    check_oscillation_and_escape(+1);
    return;
  }
  if (r[3] >= TH || r[4] >= TH) {              // 우측
    Motors_spin_left(SPEED_TURN);
    check_oscillation_and_escape(-1);
    return;
  }
}

static inline void g_scheduler_tick(){
  uint32_t now = millis();
  if (now - last_schedule_ms < G_SCHEDULE_PERIOD_MS) return;
  last_schedule_ms = now;

  if (!pending_valid) return;           // 새 G 없음
  if (!path_clear || phase == PH_AVOID || emergency_active) return; // 길 막힘/회피중이면 대기

  // 여기서부터 새 G 시작(IDLE이거나 방금 끊었거나)
  bool has_turn = fabs(pending_yaw)  >= MIN_DELTA_DEG;
  bool has_move = fabs(pending_dist) >= MIN_DELTA_MM;

  if (has_turn){
  Mona_spin_deg(pending_yaw, SPEED_TURN);   // ★ 엔코더 폐루프 회전
  }
  if (has_move){
    Mona_drive_mm(pending_dist, SPEED_FWD);   // ★ 엔코더 폐루프 직진(쏠림 보정)
  }
  Serial.println("OK G");                      // 결과 통일 후 완료 알림
  

  current_valid = true;
  current_yaw   = pending_yaw;
  current_dist  = pending_dist;

  pending_valid = false; // 펜딩 소진
}

// ===== 명령 파서: G만(회피 중엔 거절하지 않고 '펜딩'으로 저장) =====
//  * 회피/비상 중이어도 펜딩에 저장만 하고 실행은 스케줄러가 판단
static inline void handleLine(char* raw){
  if (raw[0] != 'G'){ Serial.println("ERR unknown cmd"); return; }

  float yaw=0, dist=0;
  if (sscanf(raw+1, "%f %f", &yaw, &dist) != 2){
    Serial.println("ERR G parse"); return;
  }

  // 항상 '펜딩'으로 저장(마지막으로 들어온 G가 우선)
  pending_yaw   = yaw;
  pending_dist  = dist;
  pending_valid = true;
  // 즉시 실행하지 않음 → g_scheduler_tick()이 100ms마다 심사/실행
}

// ===== setup/loop =====
void setup(){
  Mona_ESP_init();
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,password);
  Serial.printf("Connecting to WiFi: %s\n", ssid);
  while(WiFi.status()!=WL_CONNECTED){ delay(500); Serial.print("."); }
  Serial.println("\nWiFi Connected");
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  wifiServer.begin();
  Serial.printf("Server listening on %u\n", SERVER_PORT);
}

void loop(){
  WiFiClient client = wifiServer.available();
  if (!client){ delay(1); return; }

  client.setNoDelay(true);
  preempt_motion();
  lastRxMs = millis();
  rxLen = 0;

  while (client.connected()){
    // 0) 회피 우선
    avoidance_tick();

    // 1) 수신 (펜딩에만 쌓음)
    while (client.available()>0){
      char c = client.read();
      lastRxMs = millis();
      if (c=='\n' || c=='\r'){
        if (rxLen>0){ rxBuf[rxLen]='\0'; handleLine(rxBuf); rxLen=0; }
      }else if (rxLen<(int)sizeof(rxBuf)-1){
        rxBuf[rxLen++]=c;
      }
    }

    // 2) G 스케줄링(100ms 주기, 길 열릴 때만)
    g_scheduler_tick();

    // 3) 현재 G 단계 타임아웃 처리
    if (phase == PH_TURN || phase == PH_MOVE){
      if (millis() >= phase_end_ms){
        stopMotors();
        if (phase == PH_TURN && fabs(queued_move_mm) >= MIN_DELTA_MM){
          float mm = queued_move_mm; queued_move_mm = 0.0f;
          start_move_mm(mm);
        }else{
          phase = PH_IDLE;
          if (inflight_cmd=='G') Serial.println("OK G");
          inflight_cmd='\0';
          current_valid = false;
        }
      }
    }

    // 4) 유휴 타임아웃
    if (millis()-lastRxMs > IDLE_TIMEOUT_MS){
      Serial.println("Idle timeout. Closing client.");
      break;
    }

    delay(1);
  }

  preempt_motion();
  client.stop();
  Serial.println("Client disconnected");
}
