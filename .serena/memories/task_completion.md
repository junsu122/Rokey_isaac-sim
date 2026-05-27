# 태스크 완료 체크리스트

## 코드 변경 후
1. 문법 검사:
   ```bash
   python3 -c "import ast; ast.parse(open('파일.py').read()); print('OK')"
   ```
2. 좌표 변경 시 → `robot_config.py`만 수정 (에이전트 파일 분산 수정 금지)
3. FSM 상태 추가 시 → 파일 상단 FSM 상태 목록 주석도 업데이트

## 테스트
- 별도 단위 테스트 없음. 실행: `isaac-python main.py`
- 로그에서 상태 전이 확인: `[iw_hub_02] STATE_A → STATE_B`
- 린터/포매터 없음 (Isaac Sim 번들 Python 환경 제약)
