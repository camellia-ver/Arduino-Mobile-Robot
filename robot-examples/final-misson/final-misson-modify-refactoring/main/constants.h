#pragma once
#include <Arduino.h>

// RFID 핀 설정
constexpr uint8_t RFID_SS_PIN = 2;      // RFID Slave Select 핀 (RFID 모듈과 통신하기 위한 핀)
constexpr uint8_t RFID_RST_PIN = 4;     // RFID Reset 핀 (RFID 모듈을 리셋하는 핀)

// 부저 핀 설정
constexpr uint8_t BUZZER_PIN = 3;       // 부저 출력 핀 (경고음을 출력하기 위한 핀)
constexpr uint8_t BUTTON_PIN = A3;      // 시작 버튼 입력 핀 (시작 버튼이 눌렸는지 확인하는 핀)

// 서보 모터 핀 설정
constexpr uint8_t SERVO_PIN = 9;        // 서보 모터 제어 핀 (서보 모터의 위치를 조정하는 핀)

// 라인트레이싱 센서 핀 설정
constexpr uint8_t IR_PIN_LEFT = A6;     // 왼쪽 센서 (라인 추적을 위한 왼쪽 IR 센서)
constexpr uint8_t IR_PIN_RIGHT = A7;    // 오른쪽 센서 (라인 추적을 위한 오른쪽 IR 센서)
constexpr uint8_t IR_PIN_FRONT_CENTER = A1; // 전방 중앙 센서 (라인 추적을 위한 중앙 IR 센서)

// 모터 핀 설정
constexpr uint8_t MOTOR_LEFT_DIR_PIN = 7; // 왼쪽 모터 방향 핀 (왼쪽 모터의 회전 방향 제어)
constexpr uint8_t MOTOR_LEFT_PWM_PIN = 5; // 왼쪽 모터 속도 제어 핀 (왼쪽 모터의 속도 조절)
constexpr uint8_t MOTOR_RIGHT_DIR_PIN = 8; // 오른쪽 모터 방향 핀 (오른쪽 모터의 회전 방향 제어)
constexpr uint8_t MOTOR_RIGHT_PWM_PIN = 6; // 오른쪽 모터 속도 제어 핀 (오른쪽 모터의 속도 조절)

// 모터 방향 설정
constexpr int DIR_FORWARD  = 0; // 앞으로 이동 (모터가 전진하는 방향)
constexpr int DIR_BACKWARD = 1; // 뒤로 이동 (모터가 후진하는 방향)

// 라인트레이싱 센서의 캘리브레이션 및 임계값 설정
constexpr int IR_CALIB_OFFSET      = 60; // 센서 캘리브레이션을 위한 오프셋 값
constexpr int IR_MID_THRESHOLD     = 560 + IR_CALIB_OFFSET; // 중간값 센서 임계값 (센서에서 중앙을 기준으로 비교)

// 라인트레이싱 센서 임계값 설정 (흰색/검은색)
constexpr int IR_THRESHOLD_WHITE = 470; // 흰색을 기준으로 한 임계값 (라인 추적 시 흰색과 비교)
constexpr int IR_THRESHOLD_BLACK = 700; // 검은색을 기준으로 한 임계값 (라인 추적 시 검은색과 비교)

// 서보 모터 위치 설정
constexpr int SERVO_POS_DOWN       = 50;  // 서보 모터의 하향 위치 (작동 시 물체를 내려놓는 위치)
constexpr int SERVO_POS_UP         = 170; // 서보 모터의 상향 위치 (작동 시 물체를 올리는 위치)

// 기본 속도 및 회전 속도 설정
constexpr int DEFAULT_SPEED        = 80;  // 기본 이동 속도 (모터의 기본 속도)
constexpr int TURN_SPEED           = 100; // 회전 속도 (회전 시 더 빠르게 돌 수 있도록 설정)

// 시간 관련 상수들 (서보 모터의 움직임, 이동, 회전 등을 위한 딜레이 시간)
constexpr unsigned long SERVO_MOVE_DELAY     = 300UL;    // 서보 모터가 이동하는 시간 지연 (ms)
constexpr unsigned long SERVO_ATTACH_DELAY   = 10UL;     // 서보 모터가 부착되는 시간 지연 (ms)
constexpr unsigned long SERVO_DETACH_DELAY   = 10UL;     // 서보 모터가 분리되는 시간 지연 (ms)
constexpr unsigned long STOP_DELAY_BEFORE_TURN = 50UL;   // 회전 전에 멈추는 시간 지연 (ms)
constexpr unsigned long TURN_FINAL_FORWARD   = 90UL;     // 회전 후 이동 전 진입할 시간 (ms)
constexpr unsigned long TURN_EXTRA_DELAY     = 250UL;    // 회전 후 추가적인 지연 시간 (ms)
constexpr unsigned long SKIP_LINE_DURATION   = 200UL;    // 라인 추적 중 라인을 넘는 시간 지연 (ms)
constexpr unsigned long INTERSECTION_WAIT    = 160UL;    // 교차로 대기 시간 (ms)

// 외부 변수 선언
extern int runState;  // 시스템의 현재 실행 상태 (상태 값에 따라 다른 동작 수행)
extern int selectedPath; // 선택된 경로 (라인 추적을 위한 경로 선택)
extern int OBSTACLE_THRESHOLD; // 장애물 감지 임계값 (장애물의 크기를 판별하는 값)
extern unsigned long lineTraceStartTime; // 라인트레이싱 시작 시간 (라인 추적을 시작한 시간 기록)
