// 위치 보정 테스트
void loop() {
    uint8_t dummyX, dummyY;
  
    // 1) RFID 태그 인식 대기
    if (readCoordinatesFromRFID(dummyX, dummyY)) {
      tone(BUZZER_PIN, 1000); delay(100); noTone(BUZZER_PIN);
      Serial.print(F("RFID detected. Aligning to line..."));
      Serial.println();
  
      // 2) 시작 시 라인 중앙 정렬
      alignToLine(defaultPower);
      Serial.println(F("Aligned. Starting line trace."));
  
      // 3) 교차로까지 라인트레이싱
      while (!atIntersection()) {
        simpleLineTrace(defaultPower);
        delay(2);
      }
      stopMotors();
      Serial.println(F("Intersection detected!"));
  
      // 4) 위치 보정
      correctPositionAfterIntersection(defaultPower);
      Serial.println(F("Position correction complete."));
  
      // 5) 방금 읽은 카드를 HALT 처리 → 다음 카드 대기
      rfidReader.PICC_HaltA();
      rfidReader.PCD_StopCrypto1();
      Serial.println(F("Test complete. Waiting for next card..."));
    }
  
    delay(50);
  }

/**
 * @brief simpleLineTrace로 선을 따라 주행하다가
 *        교차로 감지 시 우회전, 좌회전, 180° 회전을 순차적으로 테스트하는 루프
 * @details
 * 1) phase 0: simpleLineTrace(defaultPower)로 라인 추종  
 *    → isIntersection() == true 이면 정지 후 우회전 테스트  
 * 2) phase 1: simpleLineTrace(defaultPower)로 라인 추종  
 *    → isIntersection() == true 이면 정지 후 좌회전 테스트  
 * 3) phase 2: simpleLineTrace(defaultPower)로 라인 추종  
 *    → isIntersection() == true 이면 정지 후 180° 회전 테스트  
 * 4) phase 3: 테스트 완료 후 무한 대기
 */
void loop() {
    uint8_t dummyX, dummyY;
  
    // 1) RFID 태그 인식 대기
    if (readCoordinatesFromRFID(dummyX, dummyY)) {
      tone(BUZZER_PIN, 1000); delay(100); noTone(BUZZER_PIN);
      Serial.print(F("RFID detected. Aligning to line..."));
      Serial.println();
  
      // 2) 시작 시 라인 중앙 정렬
      alignToLine(defaultPower);
      Serial.println(F("Aligned. Starting line trace."));
  
      static uint8_t phase = 0;

    switch (phase) {
        case 0:
            // 1단계: 우회전 90° 테스트
            simpleLineTrace(defaultPower);
            if (isIntersection()) {
                stopMotors();
                Serial.println(F("교차로 감지: 90° 우회전 테스트"));

                correctPositionAfterIntersection(defaultPower);
                Serial.println(F("Position correction complete."));

                turnRight90Degrees();
                delay(1000);            ///< 회전 후 잠시 대기
                // 교차로를 벗어날 때까지 대기
                while (isIntersection()) { delay(10); }
                phase++;
            }
            break;

        case 1:
            // 2단계: 좌회전 90° 테스트
            simpleLineTrace(defaultPower);
            if (isIntersection()) {
                stopMotors();
                Serial.println(F("교차로 감지: 90° 좌회전 테스트"));

                correctPositionAfterIntersection(defaultPower);
                Serial.println(F("Position correction complete."));

                turnLeft90Degrees();
                delay(1000);            ///< 회전 후 잠시 대기
                while (isIntersection()) { delay(10); }
                phase++;
            }
            break;

        case 2:
            // 3단계: 180° 회전 테스트
            simpleLineTrace(defaultPower);
            if (isIntersection()) {
                stopMotors();
                Serial.println(F("교차로 감지: 180° 회전 테스트"));

                correctPositionAfterIntersection(defaultPower);
                Serial.println(F("Position correction complete."));

                turnAround180Degrees(false);
                delay(2000);            ///< 회전 후 잠시 대기
                while (isIntersection()) { delay(10); }
                stopMotors();
                phase++;
            }
            break;

        default:
            // 테스트 완료: 대기
            Serial.println(F("테스트 완료"));
    }
  
      // 5) 방금 읽은 카드를 HALT 처리 → 다음 카드 대기
      rfidReader.PICC_HaltA();
      rfidReader.PCD_StopCrypto1();
      Serial.println(F("Test complete. Waiting for next card..."));
    }
  
    delay(10);
}
