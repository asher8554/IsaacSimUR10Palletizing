# Checklist — UR10 팔레타이징 따라하기 가이드

목표: Isaac Sim 6.2의 UR10 Palletizing 예제를 직접 구현할 수 있게, 검증된 단계별 코드와 HTML 가이드를 만든다.

## 조사
- [x] Notion 관련 지식 검색 (팔레타이징 전용 페이지 없음)
- [x] 설치된 예제 소스 읽기 (`isaacsim.robot_motion.examples/.../ur10_palletizing.py`)
- [x] 공식 예제 헤드리스 실행 확인 (`--test --seed 1 --max-bins 1` 완료)
- [x] 로봇·그리퍼·박스·컨베이어 prim 구조 추출

## 코드 (단계마다 `--test --headless` 통과)
- [x] step01 로봇 불러오기
- [x] step02 설비 배치 (`palletizing/scene.py`)
- [x] step03 컨베이어 구동
- [x] step04 무작위 박스 생성 (`spawner.py`)
- [x] step05 위치 인식 (`common.py`, `perception.py`)
- [x] step06 흡착 그리퍼 (`gripper.py`)
- [x] step07 로봇 모션 (`motion.py`)
- [x] step08 한 사이클 집기·놓기 (`tasks.py`, `cell.py`)
- [x] step09 상태기계·뒤집기 (`fsm.py`) — `--seed 5 --max-bins 2`, `--seed 3 --max-bins 4` PASS
- [x] step09 전체 9개 적재 (`--seed 1`) PASS — 첫 시도 실패(집는 방향 부호 경계) 후 `perception.py` 수정

## 가이드
- [x] `docs/build_guide.py` (실제 소스를 가이드에 끼워 넣는 빌드)
- [x] `docs/guide.html` 글래스모피즘·다크·Pretendard
- [x] 렌더링 화면 캡처 확인 (데스크톱 1360px, 좁은 폭 520px. 키보드 포커스 표시는 CSS만 넣고 눈으로는 미확인)
- [x] README (실행 명령), 커밋
