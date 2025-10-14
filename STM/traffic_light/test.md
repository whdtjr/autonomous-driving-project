제목
- [개선] 신호등 STM32 IWDG(Watchdog) 적용

현재 상태/문제점
- 네트워크 스택/메인 루프가 교착되면 LED/통신이 멈춰 수동 재부팅 필요
- 장애를 자동 복구할 장치 부재, 리셋 원인 파악 어려움
- 목표: 비정상 상태 자동 감지·복구 및 리셋 원인 로그화

개선 내용
- IWDG 적용 및 구성
  - 초기화: MX_IWDG_Init() 호출로 시작됨. Core/Src/main.c:169
  - 설정: Prescaler=128, Reload=2500-1 → 타임아웃 ≈ 10 s (LSI 기준)
    - hiwdg.Instance = IWDG; Core/Src/main.c:341
    - hiwdg.Init.Prescaler = IWDG_PRESCALER_128; Core/Src/main.c:342
    - hiwdg.Init.Reload = 2500-1; Core/Src/main.c:343
  - 부팅 시 IWDG 리셋 여부 출력: Core/Src/main.c:130
- 킥 정책(헬스체크 기반)
  - 1초 타이머(TIM3) 운용: Core/Src/main.c:373-378
  - 3초마다 킥 기회 생성: wdKickReady = 1 (time3SecCnt % 3 == 0) Core/Src/main.c:230-233
  - 헬스 플래그
    - cp_main_ok: 메인 주기 처리(상태 출력/전송 등) 1회 정상 실행 시 1 Core/Src/main.c:240-246
    - cp_net_ok: esp_get_status() == 0(온라인)일 때 1 Core/Src/main.c:212-233
  - 킥 조건 충족 시에만 리프레시:
    - if (wdKickReady && cp_main_ok && cp_net_ok) HAL_IWDG_Refresh(&hiwdg); Core/Src/main.c:262
    - 실패 시 스킵 로그 출력, 이후 플래그 초기화 Core/Src/main.c:254-268
- 타임아웃 동작
  - 10초 내 리프레시 미발생 시 자동 리셋
  - 리셋 후 UART 로그에 “[BOOT] Reset by IWDG” 출력 Core/Src/main.c:130

수용 기준(측정 가능한 목표)
- 정상 상태에서 IWDG 리프레시가 약 3초 간격으로 수행됨(로그/프로브로 확인)
- 메인 루프 정지 또는 네트워크 오프라인(>10 s) 시 7–13 s 내 자동 리셋
- 리셋 직후 “[BOOT] Reset by IWDG” 로그 출력(Core/Src/main.c:130)
- 리셋 후 LED 상태머신/통신 시도 재개, 3초 내 재연결 시도 로그 확인
- 구성 변경 가능
  - 킥 주기: time3SecCnt % 3의 ‘3’을 N으로 조정해 N초마다 검증
  - 타임아웃: Prescaler/Reload 값으로 목표 타임아웃 설정(LSI 오차 고려)

리스크/마이그레이션
- LSI 오차(±~20%)로 실제 타임아웃 편차 가능 → 킥 주기를 타임아웃의 1/2 이하로 유지 권장
- cp_net_ok를 엄격히 요구하므로 네트워크 플래핑 시 리셋 잦아질 수 있음
  - 대안: 오프라인 그레이스 기간 N회 허용 또는 제한적 킥 허용 정책
- 장시간 블로킹 작업/펌웨어 업데이트 중 타임아웃 위험 → 구간 전후 헬스 신호 설계 필요
- IWDG는 Stop/Standby에서도 동작 → 저전력 모드 사용 시 타임아웃 재검토 필요

추가 메모
- TIM3 1 Hz 설정: Core/Src/main.c:373-378
- 타이머 인터럽트 콜백(1초마다 플래그 세팅): Core/Src/main.c:543-558
- 과거 무조건 킥 라인 주석 처리됨(Core/Src/main.c:255) → 현재는 헬스 기반만 킥

