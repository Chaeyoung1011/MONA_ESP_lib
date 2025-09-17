#include "Mona_ESP_lib.h"
#include <Wire.h>
#include <WiFi.h>
#include <math.h>

// --- Wi-Fi ---
const char* ssid     = "InMO_Lab_2.4G";
const char* password = "dlsahfoq104";
const uint16_t SERVER_PORT = 8080;
WiFiServer wifiServer(SERVER_PORT);

// --- 튜닝(현장 캘리브 필요) ---
static const float TURN_MS_PER_DEG = 8.0f;   // 1도 회전에 걸리는 시간(ms)
static const float MOVE_MS_PER_MM  = 3.0f;   // 1mm 이동에 걸리는 시간(ms)
static const int   SPEED_TURN      = 100;    // 스핀 속도
static const int   SPEED_FWD       = 150;    // 전진 속도
static const float MIN_DELTA_DEG   = 1.0f;   // 너무 작은 회전은 무시
static const float MIN_DELTA_MM    = 5.0f;   // 너무 작은 이동은 무시
static const uint32_t IDLE_TIMEOUT_MS = 8000;

// --- IR 회피 설정 ---
static const int      CLEAR_THR       = 70;  // "일정한 값": 70
static const uint16_t AVOID_SAMPLE_MS = 5;   // 회피 샘플 주기

// "거리 비교"를 위한 스윕 범위(경계 threshold 탐색)
static const int SWEEP_THR_MIN  = 40;   // 60/80 포함하도록 40~90 사용
static const int SWEEP_THR_MAX  = 90;
static const int SWEEP_THR_STEP = 5;

// --- 논블로킹 상태머신 ---
enum Phase { PH_IDLE=0, PH_TURN, PH_MOVE, PH_AVOID };
static Phase     phase = PH_IDLE;
static uint32_t  phase_end_ms = 0;            // 현재 단계 종료 시각
static float     queued_move_mm = 0.0f;       // 회전 후 이어질 이동거리(같이 들어온 dist)
static char      inflight_cmd = '\0';         // 진행 중 명령표시('G')

// --- 수신 버퍼/타이밍 ---
static char      rxBuf[128];
static int       rxLen = 0;
static uint32_t  lastRxMs = 0;

// --- 우선순위 판단용 센서 캐시 ---
static bool det_clear_all = true;  // CLEAR_THR에서 전부 미감지?
static bool det_any_block = false; // CLEAR_THR에서 하나라도 감지?

// ===== 유틸 =====
static inline void stopMotors(){ Motors_forward(0); }
static inline void blink_ok(){} // LED 안 씀(더미)

static inline void preempt_motion(){
  queued_move_mm = 0.0f;
  inflight_cmd   = '\0';
  phase          = PH_IDLE;
  phase_end_ms   = 0;
  stopMotors();
}

static inline void start_turn_deg_cw(float yaw_deg){
  float dur_ms = fabsf(yaw_deg) * TURN_MS_PER_DEG * 0.9f; // 보수적 언더슈트
  phase_end_ms = millis() + (uint32_t)dur_ms;
  phase = PH_TURN;
  if (yaw_deg >= 0.0f) Motors_spin_right(SPEED_TURN);
  else                 Motors_spin_left(SPEED_TURN);
}

static inline void start_move_mm(float dist_mm){
  float dur_ms = fabsf(dist_mm) * MOVE_MS_PER_MM;
  phase_end_ms = millis() + (uint32_t)dur_ms;
  phase = PH_MOVE;
  // 미니멀: 후진 미지원(원하면 분기 추가)
  Motors_forward(SPEED_FWD);
}

static inline void start_move_for_ms(uint32_t ms){
  phase_end_ms = millis() + ms;
  phase = PH_MOVE;
  Motors_forward(SPEED_FWD);
}

// ===== 센서 유틸 =====
static inline void read_all_sensors_clearthr(bool outDet[5]){
  outDet[0] = Detect_object(1, CLEAR_THR); // 좌(1)
  outDet[1] = Detect_object(2, CLEAR_THR); // 좌전(2)
  outDet[2] = Detect_object(3, CLEAR_THR); // 정면(3)
  outDet[3] = Detect_object(4, CLEAR_THR); // 우전(4)
  outDet[4] = Detect_object(5, CLEAR_THR); // 우(5)
}

// 센서 하나의 "경계 threshold": TRUE가 되는 가장 높은 값(없으면 0)
static inline int scan_boundary_threshold(uint8_t channel){
  int boundary = 0;
  for (int thr = SWEEP_THR_MAX; thr >= SWEEP_THR_MIN; thr -= SWEEP_THR_STEP){
    if (Detect_object(channel, thr)){ boundary = thr; break; }
  }
  return boundary; // 0이면 해당 스윕 범위에서 감지 없음(= 가장 여유)
}

// ===== 안전 회피 틱(회피 최우선) =====
// - CLEAR_THR에서 하나라도 감지되면 → 회피(PH_AVOID). G보다 우선.
// - 전 센서 미감지면 → 회피 즉시 종료 + 모터 STOP, 이후 G 우선.
static uint32_t avoid_last_ms = 0;
static inline void safety_avoid_tick(){
  uint32_t now = millis();
  if (now - avoid_last_ms < AVOID_SAMPLE_MS) return;
  avoid_last_ms = now;

  // 1) CLEAR_THR에서 센서 판독
  bool D[5];
  read_all_sensors_clearthr(D);
  det_any_block = (D[0] || D[1] || D[2] || D[3] || D[4]);
  det_clear_all = !det_any_block;

  // 2) 클리어 → 회피 종료 & 정지
  if (det_clear_all){
    if (phase == PH_AVOID){
      stopMotors();
      phase = PH_IDLE;
      phase_end_ms = 0;
    }
    return; // 클리어면 회피 로직 끝. 이후 G 처리.
  }

  // 3) 감지됨 → 회피 우선(PH_AVOID 진입/유지)
  int B1 = scan_boundary_threshold(1); // 좌(1)
  int B2 = scan_boundary_threshold(2); // 좌전(2)
  int B3 = scan_boundary_threshold(3); // 정면(3)
  int B4 = scan_boundary_threshold(4); // 우전(4)
  int B5 = scan_boundary_threshold(5); // 우(5)

  // 전방 그룹 {2,3,4} 중 가장 막힌 센서 선택 (tie-break: 3 > 4 > 2)
  int worst_id = 2;
  int worst_B  = B2;
  if (B3 >= worst_B){ worst_B = B3; worst_id = 3; }
  if (B4 >  worst_B || (B4 == worst_B && worst_id == 2)){ worst_B = B4; worst_id = 4; }

  if (phase != PH_AVOID){
    stopMotors();
    phase = PH_AVOID;
    phase_end_ms = 0;
    inflight_cmd = '\0';
  }

  // 방향 결정 규칙(요청 명시):
  // worst==3 → 2 vs 4 비교, 더 "낮은"(여유) 쪽으로
  // worst==4 → 3 vs 5 비교, 더 낮은 쪽으로
  // worst==2 → 1 vs 3 비교, 더 낮은 쪽으로
  bool turn_left = true; // 기본 좌
  if (worst_id == 3){
    turn_left = (B2 < B4);          // 2가 여유면 좌, 4가 여유면 우
  } else if (worst_id == 4){
    turn_left = (B3 < B5);          // 3이 여유면 좌, 5가 여유면 우
  } else { // worst_id == 2
    turn_left = (B1 < B3);          // 1이 여유면 좌, 3이 여유면 우
  }

  if (turn_left) Motors_spin_left(SPEED_TURN);
  else           Motors_spin_right(SPEED_TURN);
}

// ===== 명령 파서: G만 지원 =====
// 정책: 감지/회피 중엔 G를 "저장하지 않고" 무시(BUSY AVOID), 클리어일 때만 즉시 실행.
static inline void launch_G(float yaw, float dist){
  preempt_motion();
  bool has_turn = fabsf(yaw)  >= MIN_DELTA_DEG;
  bool has_move = fabsf(dist) >= MIN_DELTA_MM;
  if (has_turn){
    start_turn_deg_cw(yaw);
    inflight_cmd='G';
    queued_move_mm = has_move ? dist : 0.0f;
  } else if (has_move){
    start_move_mm(dist);
    inflight_cmd='G';
  } else {
    Serial.println("OK G");
    blink_ok();
  }
}

static inline void handleLine(char* raw){
  if (raw[0]=='G'){
    float yaw=0, dist=0;
    if (sscanf(raw+1,"%f %f",&yaw,&dist)==2){
      // 감지/회피 중이면 실행하지 않고 무시(저장 안 함)
      if (!det_clear_all || phase == PH_AVOID){
        Serial.println("BUSY AVOID"); // 알림: 회피 중이라 무시됨
        return;
      }
      // 클리어면 즉시 실행(G 우선)
      launch_G(yaw, dist);
    }else{
      Serial.println("ERR G parse");
    }
    return;
  }
  Serial.println("ERR unknown cmd"); // G 외에는 거부
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
  if (!client){
    delay(1);
    return;
  }

  client.setNoDelay(true);
  preempt_motion();                 // 안전 정지 후 새 명령 대기
  lastRxMs = millis();
  rxLen = 0;

  while (client.connected()){
    // 1) 회피 최우선
    safety_avoid_tick();

    // 2) 수신/파싱
    while (client.available()>0){
      char c = client.read();
      lastRxMs = millis();
      if (c=='\n' || c=='\r'){
        if (rxLen>0){
          rxBuf[rxLen]='\0';
          handleLine(rxBuf);
          rxLen=0;
        }
      }else{
        if (rxLen<(int)sizeof(rxBuf)-1) rxBuf[rxLen++]=c;
      }
    }

    // 3) 모션 단계 타임아웃 (PH_AVOID는 타임아웃 없음)
    if (phase != PH_IDLE && phase != PH_AVOID && millis() >= phase_end_ms){
      stopMotors();
      if (phase == PH_TURN && fabsf(queued_move_mm) >= MIN_DELTA_MM){
        float mm = queued_move_mm; queued_move_mm = 0.0f;
        start_move_mm(mm);          // 회전 → 직진 전이
      }else{
        phase = PH_IDLE;
        if (inflight_cmd=='G') Serial.println("OK G");
        inflight_cmd='\0';
        blink_ok();
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
