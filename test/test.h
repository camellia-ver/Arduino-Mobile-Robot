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