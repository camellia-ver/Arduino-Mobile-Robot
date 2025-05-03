#pragma once
#include <Arduino.h>

// ======= RFID 설정 =======
constexpr uint8_t RFID_SS_PIN  = 2;   // Slave Select (SPI CS) for RFID
constexpr uint8_t RFID_RST_PIN = 4;   // Reset pin for RFID

// ======= 부저 & 버튼 =======
constexpr uint8_t BUZZER_PIN   = 3;   // Buzzer output
constexpr uint8_t BUTTON_PIN   = A3;  // Start button input

// ======= 서보 모터 =======
constexpr uint8_t SERVO_PIN    = 9;   // Servo control pin
constexpr unsigned long SERVO_ATTACH_DELAY = 10UL;   // attach() 후 대기 (ms)
constexpr unsigned long SERVO_DETACH_DELAY = 10UL;   // detach() 후 대기 (ms)
constexpr unsigned long SERVO_MOVE_DELAY   = 300UL;  // 이동 지연 (ms)
constexpr int SERVO_POS_UP   = 170;   // 물체 들어올림 위치 (°)
constexpr int SERVO_POS_DOWN = 50;    // 물체 내려놓음 위치 (°)

// ======= 모터 설정 =======
constexpr uint8_t MOTOR_LEFT_DIR_PIN  = 7; // Left motor direction
constexpr uint8_t MOTOR_LEFT_PWM_PIN  = 5; // Left motor speed (PWM)
constexpr uint8_t MOTOR_RIGHT_DIR_PIN = 8; // Right motor direction
constexpr uint8_t MOTOR_RIGHT_PWM_PIN = 6; // Right motor speed (PWM)

constexpr int DIR_FORWARD  = 0;  // 전진
constexpr int DIR_BACKWARD = 1;  // 후진

constexpr int DEFAULT_SPEED = 80;  // 기본 속도
constexpr int TURN_SPEED    = 100; // 회전 속도

// ======= 라인트레이싱 센서 =======
constexpr uint8_t IR_PIN_LEFT         = A6; // 왼쪽 센서
constexpr uint8_t IR_PIN_RIGHT        = A7; // 오른쪽 센서
constexpr uint8_t IR_PIN_FRONT_CENTER = A1; // 전방 중앙 센서

// 임계값 및 캘리브레이션
constexpr int IR_CALIB_OFFSET    = 60;
constexpr int IR_MID_THRESHOLD   = 560 + IR_CALIB_OFFSET;
constexpr int IR_THRESHOLD_WHITE = 470; // 화이트 기준
constexpr int IR_THRESHOLD_BLACK = 700; // 블랙 기준

// ======= 장애물 감지 =======
constexpr int OBSTACLE_THRESHOLD = 1005; // 초과 시 장애물로 간주

// ======= 회전 & 타이밍 =======
constexpr unsigned long STOP_DELAY_BEFORE_TURN = 50UL;   // 회전 전 멈춤 (ms)
constexpr unsigned long TURN_EXTRA_DELAY       = 250UL;  // 회전 후 추가 지연 (ms)
constexpr unsigned long TURN_FINAL_FORWARD     = 90UL;   // 회전 후 전진 전 지연 (ms)
constexpr unsigned long TURN_DURATION_180      = 1200UL; // 180° 회전 시간 (ms)
constexpr unsigned long INTERSECTION_WAIT      = 160UL;  // 교차로 대기 시간 (ms)
constexpr unsigned long SKIP_LINE_DURATION    = 200UL;   // 라인 통과 시간 (ms)

// ======= 외부 변수 & 상태 =======
extern int runState;            // 현재 실행 상태
extern int selectedPath;        // 선택된 경로
extern unsigned long lineTraceStartTime; // 라인트레이스 시작 시각

enum RunState {
    STATE_WAIT_FOR_RFID       = 0,   // RFID 대기
    STATE_LINE_TRACE          = 1,   // 라인 추적
    STATE_TURN_LEFT_90        = 11,  // 90° 좌회전
    STATE_TURN_DECISION       = 12,  // 회전 방향 결정
    STATE_TURN_LEFT_180       = 13,  // 180° 좌회전
    STATE_FORWARD_AFTER_DECISION = 101, // 회전 결정 후 전진
    STATE_FORWARD_AFTER_TURN     = 102  // 회전 후 전진
};
