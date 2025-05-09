/**
 * @brief 90° 좌회전을 수행하고 회전 이후 직진을 시작
 * @details
 *   1. 모터 정지 및 짧은 후진으로 관성 보정
 *   2. 왼쪽 IR 센서를 사용해 화이트→블랙 임계값을
 *      두 단계로 감지하며 회전
 *   3. 회전 종료 후 직진 모드로 복귀
 */
void turnLeft90Degrees() {
  stopMotors();                       ///< 모터 정지
  delay(50);                          ///< 관성 보정용 짧은 대기
    
  driveMotors(DIRECTION_BACKWARD, 80, DIRECTION_BACKWARD, 80);
  delay(20);                          ///< 약간 후진
  stopMotors();                       ///< 다시 정지
  delay(50);                          ///< 안정화를 위한 대기

  // 1단계 좌회전: 라인 밖으로 벗어나기
  turnLeft(turnPower);
  while (analogRead(IR_SENSOR_LEFT_PIN) < MAX_WHITE_THRESHOLD) {
    delay(1);                         ///< 흰색 바닥을 벗어날 때까지 대기
  }
  delay(40);                          ///< 센서 반응 보정

  // 2단계 좌회전: 반대편 라인 감지
  turnLeft(turnPower);
  while (analogRead(IR_SENSOR_LEFT_PIN) > MIN_BLACK_THRESHOLD) {
    delay(1);                         ///< 검은색 라인을 만날 때까지 대기
  }
  delay(40);                          ///< 센서 반응 보정

  // 회전 마무리 후 직진으로 전환
  moveForward(turnPower);
  delay(90);                          ///< 직진 거리 보정

  // 최종 좌회전 보정
  turnLeft(turnPower);
  delay(250);                         ///< 로봇 관성에 따라 조정 필요

  stopMotors();                       ///< 최종 정지
}

/**
 * @brief 90° 우회전을 수행하고 회전 이후 직진을 시작
 * @details
 *   1. 모터 정지 및 짧은 후진으로 관성 보정
 *   2. 오른쪽 IR 센서를 사용해 화이트→블랙 임계값을
 *      두 단계로 감지하며 회전
 *   3. 회전 종료 후 직진 모드로 복귀
 */
void turnRight90Degrees() {
  stopMotors();                       ///< 모터 정지
  delay(50);                          ///< 관성 보정용 짧은 대기

  driveMotors(DIRECTION_BACKWARD, 80, DIRECTION_BACKWARD, 80);
  delay(20);                          ///< 약간 후진
  stopMotors();                       ///< 다시 정지
  delay(50);                          ///< 안정화를 위한 대기

  // 1단계 우회전: 라인 밖으로 벗어나기
  turnRight(turnPower);
  while (analogRead(IR_SENSOR_RIGHT_PIN) < MAX_WHITE_THRESHOLD) {
    delay(1);                         ///< 흰색 바닥을 벗어날 때까지 대기
  }
  delay(40);  
    
  // 2단계 우회전: 반대편 라인 감지
  turnRight(turnPower);
  while (analogRead(IR_SENSOR_RIGHT_PIN) > MIN_BLACK_THRESHOLD) {
    delay(1);                         ///< 검은색 라인을 만날 때까지 대기
  }
  delay(40);                          ///< 센서 반응 보정

  // 회전 마무리 후 직진으로 전환
  moveForward(turnPower);
  delay(90);                          ///< 직진 거리 보정

  // 최종 우회전 보정
  turnRight(turnPower);
  delay(250);                         ///< 로봇 관성에 따라 조정 필요

  stopMotors();                       ///< 최종 정지
}

/**
 * @brief    180° 회전 수행
 * @details  
 *   1. 전진 모드로 속도를 낮추고 정지하여 관성 보정  
 *   2. 뒤로 짧게 후진하여 정지선(검은색 라인)을 벗어남  
 *   3. 후진-전진 모터 제어로 좌회전을 두 단계로 수행  
 *      - 1단계: 흰색 바닥(라인 이탈) 감지  
 *      - 2단계: 검은색 바닥(라인 도달) 감지  
 *   4. 최종 정지  
 */
void turnAround180Degrees() {
    // 1) 관성 보정을 위해 전진 속도 감속 후 즉시 정지
    driveMotors(DIRECTION_FORWARD, 70, DIRECTION_FORWARD, 70);
    delay(20);
    stopMotors();
    delay(100);

    // 2) 정지선 위 출발 시 후진하여 라인 벗어나기
    driveMotors(DIRECTION_BACKWARD, 70, DIRECTION_BACKWARD, 70);
    while ((analogRead(IR_SENSOR_LEFT_PIN)  > MIN_BLACK_THRESHOLD) ||
           (analogRead(IR_SENSOR_RIGHT_PIN) > MIN_BLACK_THRESHOLD)) {
        delay(1);  ///< 검은색 라인을 벗어날 때까지 후진 유지
    }
    delay(230);    ///< 후진 거리 보정

    // 3) 180° 좌회전: 왼쪽 바퀴 후진, 오른쪽 바퀴 전진
    driveMotors(DIRECTION_BACKWARD, turnPower, DIRECTION_FORWARD, turnPower);
    // 3-1) 1단계: 흰색 바닥 감지 시까지 회전
    while (analogRead(IR_SENSOR_LEFT_PIN) < MIN_BLACK_THRESHOLD) {
        delay(1);
    }
    delay(30);     ///< 센서 반응 보정

    // 3-2) 2단계: 검은색 라인 감지 시까지 회전 유지
    driveMotors(DIRECTION_BACKWARD, 90, DIRECTION_FORWARD, 90);
    while (analogRead(IR_SENSOR_LEFT_PIN) > MAX_WHITE_THRESHOLD) {
        delay(1);
    }

    // 4) 최종 정지
    stopMotors();
}


/* 180
  // if (backOnStopLine) {
  //     // 정지선 등지고 첫 후진
  //     driveMotors(DIRECTION_BACKWARD, 70, DIRECTION_BACKWARD, 70);
  //     while ((analogRead(IR_SENSOR_LEFT_PIN)  > MIN_BLACK_THRESHOLD) ||
  //            (analogRead(IR_SENSOR_RIGHT_PIN) > MIN_BLACK_THRESHOLD)) delay(1);
  //     delay(130);
  // } else {
  //     // 일반 후진 후 회전 준비
  //     driveMotors(DIRECTION_BACKWARD, 90, DIRECTION_BACKWARD, 90);
  //     delay(150);
  //     stopMotors();
  //     delay(50);
  // }

  // // 중앙 라인 기준 회전
  // driveMotors(DIRECTION_BACKWARD, 100, DIRECTION_FORWARD, 100);
  // while (analogRead(IR_SENSOR_LEFT_PIN) < MIN_BLACK_THRESHOLD) delay(1);
  // delay(30);
  // driveMotors(DIRECTION_BACKWARD, 90, DIRECTION_FORWARD, 90);
  // while (analogRead(IR_SENSOR_LEFT_PIN) > MAX_WHITE_THRESHOLD) delay(1);
  
  // // 회전 후 전진 복귀
  // moveForward(defaultPower);

*/

/* simplelinetracing
// // 양쪽 센서 모두 흰색: 전진
  // if (leftValue < MAX_WHITE_THRESHOLD && rightValue < MAX_WHITE_THRESHOLD) {
  //     moveForward(power);
  // }
  // // 왼쪽만 검정: 왼쪽으로 벗어났으므로 우회전
  // else if (leftValue > MIN_BLACK_THRESHOLD) {
  //     turnRight(power);
  // }
  // // 오른쪽만 검정: 오른쪽으로 벗어났으므로 좌회전
  // else if (rightValue > MIN_BLACK_THRESHOLD) {
  //     turnLeft(power);
  // }
  // // 그 외: 전진
  // else {
  //     moveForward(power);
  // }
*/