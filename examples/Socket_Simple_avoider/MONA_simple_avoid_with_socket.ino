// ===== Mona ESP (G 전용 + 이동 중 안전 회피) =====
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

// --- IR 회피 설정(ICE9 simple_avoider 기반) ---
static const int      IR_THRESHOLD     = 80;  // 요청값
static const uint16_t AVOID_SAMPLE_MS  = 5;   // 회피 샘플 주기

// --- 논블로킹 상태머신 ---
enum Phase { PH_IDLE=0, PH_TURN, PH_MOVE, PH_AVOID };
static Phase     phase = PH_IDLE;
static uint32_t  phase_end_ms = 0;            // 현재 단계 종료 시각
static float     queued_move_mm = 0.0f;       // G: 회전 후 이어질 이동거리
static char      inflight_cmd = '\0';         // 진행 중 명령표시('G')

// --- 이동 재개용 남은 시간(회피로 일시정지 시) ---
static uint32_t  move_resume_ms = 0;

// --- 수신 버퍼/타이밍 ---
static char      rxBuf[128];
static int       rxLen = 0;
static uint32_t  lastRxMs = 0;

// ===== 유틸 =====
static inline void stopMotors(){ Motors_forward(0); }
static inline void blink_ok(){} // LED 안 씀(더미)

static inline void preempt_motion(){
  queued_move_mm = 0.0f;
  inflight_cmd   = '\0';
  move_resume_ms = 0;
  phase          = PH_IDLE;
  phase_end_ms   = 0;
  stopMotors();
}

static inline void start_turn_deg_cw(float yaw_deg){
  float dur_ms = fabsf(yaw_deg) * TURN_MS_PER_DEG;
  phase_end_ms = millis() + (uint32_t)dur_ms;
  phase = PH_TURN;
  if (yaw_deg >= 0.0f) Motors_spin_right(SPEED_TURN);
  else                 Motors_spin_left(SPEED_TURN);
}

static inline void start_move_mm(float dist_mm){
  float dur_ms = fabsf(dist_mm) * MOVE_MS_PER_MM;
  phase_end_ms = millis() + (uint32_t)dur_ms;
  phase = PH_MOVE;
  Motors_forward(SPEED_FWD);        // 미니멀: 후진 미지원(원하면 추후 추가)
}

static inline void start_move_for_ms(uint32_t ms){
  phase_end_ms = millis() + ms;
  phase = PH_MOVE;
  Motors_forward(SPEED_FWD);
}

// ===== 안전 회피 틱(세션 중 상시 호출) =====
//  - PH_MOVE에서 앞이 막히면 PH_AVOID로 진입하여 피함
//  - 앞이 다시 비면 남은 이동시간(move_resume_ms)만큼 전진 재개
static uint32_t avoid_last_ms = 0;
static inline void safety_avoid_tick(){
  uint32_t now = millis();
  if (now - avoid_last_ms < AVOID_SAMPLE_MS) return;
  avoid_last_ms = now;

  // 센서 읽기
  bool IR[5];
  IR[0]=Detect_object(1,IR_THRESHOLD);
  IR[1]=Detect_object(2,IR_THRESHOLD);
  IR[2]=Detect_object(3,IR_THRESHOLD);
  IR[3]=Detect_object(4,IR_THRESHOLD);
  IR[4]=Detect_object(5,IR_THRESHOLD);

  bool front_blocked = (IR[2] || IR[3] || IR[4]);

  // 이동 중: 앞 막히면 회피 상태로 진입(남은 이동시간 저장)
  if (phase == PH_MOVE && front_blocked){
    move_resume_ms = (now < phase_end_ms) ? (phase_end_ms - now) : 0;
    phase = PH_AVOID;
    // 우선 좌회전 기본(오른쪽이 더 여유면 우회전)
    if (IR[0] && !IR[4]) Motors_spin_right(SPEED_TURN);
    else                 Motors_spin_left(SPEED_TURN);
    return;
  }

  // 회피 중: 규칙적으로 회피 진행
  if (phase == PH_AVOID){
    if (front_blocked){
      // 방향 선택: 좌/우 중 더 빈 쪽으로 회전
      if (IR[0] && !IR[4]) Motors_spin_right(SPEED_TURN);
      else                 Motors_spin_left(SPEED_TURN);
    }else{
      // 앞이 비었으면 이동 재개
      uint32_t ms = move_resume_ms;
      move_resume_ms = 0;
      start_move_for_ms(ms);
    }
  }
}

// ===== 명령 파서: G만 지원 =====
// G <yaw_deg> <dist_mm> : 회전 후 직진(둘 중 하나만 커도 동작)
static inline void handleLine(char* raw){
  if (raw[0]=='G'){
    float yaw=0, dist=0;
    if (sscanf(raw+1,"%f %f",&yaw,&dist)==2){
      preempt_motion();

      bool has_turn = fabsf(yaw)  >= MIN_DELTA_DEG;
      bool has_move = fabsf(dist) >= MIN_DELTA_MM;

      if (has_turn){
        start_turn_deg_cw(yaw);
        inflight_cmd='G';
        queued_move_mm = has_move ? dist : 0.0f; // 회전 끝나면 직진
      }else if (has_move){
        start_move_mm(dist);
        inflight_cmd='G';
      }else{
        Serial.println("OK G"); // 변화 거의 없음
        blink_ok();
      }
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
    // 클라이언트 없으면 아무 것도 하지 않음(자율주행 없음)
    delay(1);
    return;
  }

  // 세션 시작
  client.setNoDelay(true);
  preempt_motion();                 // 안전 정지 후 새 명령 대기
  lastRxMs = millis();
  rxLen = 0;

  while (client.connected()){
    // 1) 수신
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

    // 2) 안전 회피 틱(항상 먼저 체크)
    safety_avoid_tick();

    // 3) 모션 단계 완료 체크(회피 상태는 타이머로 종료하지 않음)
    if (phase != PH_IDLE && phase != PH_AVOID && millis() >= phase_end_ms){
      stopMotors();

      if (phase == PH_TURN && fabsf(queued_move_mm) >= MIN_DELTA_MM){
        float mm = queued_move_mm;
        queued_move_mm = 0.0f;
        start_move_mm(mm);          // 회전 → 직진 전이
      }else{
        // 전체 명령 종료
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
