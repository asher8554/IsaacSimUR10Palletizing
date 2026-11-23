# Context Notes

## 2026-11-23 결정

- **대상 버전은 Isaac Sim 6.2.0** (`C:\isaacsim-6.2`, rc.14). 6.1도 설치돼 있으나 최신 설치본 기준. 6.2 예제는 Cortex가 아니라 `isaacsim.robot_motion`(cuMotion RMPflow) + `transitions` FSM 구조다.
- **Python은 Isaac Sim 내장 `python.bat`(3.12)** 을 쓴다. Isaac Sim 모듈은 그 인터프리터에서만 로드되므로 별도 `.venv`를 만들지 않는다(전역 AGENTS.md의 "공식 문서가 요구하는 도구" 예외).
- **구현 방식.** 공식 `ur10_palletizing.py`(Apache-2.0, 1,720줄)를 기능별 모듈로 쪼개고 한글 주석을 단 축약 재구성. 동작 로직·수치는 검증된 공식 값을 유지한다. 이유는 RMPflow 튜닝값을 임의로 바꾸면 실패 원인을 가려내기 어렵기 때문.
- **가이드의 코드 블록은 `docs/build_guide.py`가 실제 소스에서 뽑는다.** 가이드 코드와 시험한 코드가 어긋나지 않게 하기 위함.
- **"위치 인식"은 카메라 비전이 아니라 시뮬레이터의 prim 월드 포즈 읽기**다(공식 예제와 동일). 가이드에 명시한다.
- "외부 로봇"은 에셋 서버의 UR10 USD를 reference로 불러오는 것으로 해석. 자기 URDF를 쓰는 길은 가이드 끝에 포인터만 둔다.

## 확인된 사실 (probe)

- 에셋 루트: `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2` (인터넷 필요)
- UR10: `/Isaac/Robots_Multiphysics/UniversalRobots/ur10/ur10.usda`, variant `Gripper=short_suction` (선택지 long_suction/none/short_suction)
- 그리퍼 prim `ee_link/SurfaceGripper`(IsaacSurfaceGripper), attachmentPoints → `ee_link/suction_cup/Suction_Joint`(D6), maxGripDistance 0.02
- small_KLT: 0.198 × 0.297 × 0.146 m, 0.56 kg, 바닥판 `Collision/Cube_03`(로컬 z = -0.072)
- 헤드리스 종료 시 `Simulation App Shutting Down` 뒤 프로세스가 안 끝나는 경우가 있어 시험은 `timeout`으로 감싼다.

## 2026-11-23 실행 중 알게 된 것

- **집는 방향 부호 경계.** 뒤집는 동안 컨베이어 끝 벽에 반듯하게 걸린 박스는 bin_x가 월드 X와 나란해져, 공식 판정 `bin_ax[1] < 0`의 부호가 잡음(±0.0006)으로 갈린다. 반대쪽이 뽑히면 wrist_1·wrist_3이 ±2π 한계까지 감겨 ReachToPlace에서 멈춘다(`--seed 1` 2번째 박스에서 재현). 같은 박스 순서를 공식 예제에 넣었을 때는 우연히 좋은 쪽이 뽑혀 성공했다. `perception._GRASP_Y_REFERENCE = (-0.15, 1, 0)`으로 고정해 해결. 공식 예제도 같은 경계를 가진다는 것은 코드 판독에 따른 추정이고 공식 예제에서 실패를 재현하지는 않았다.
- **잠든 강체.** 흡착을 꺼도 가만히 매달려 있던 박스는 안 떨어진다. 속도를 조금 줘 깨워야 한다(step06).
- **동작 중 일시정지.** 캡처하려고 pause/play를 끼워 넣자 3번째 박스가 뒤집기 장치에서 떨어졌다. 캡처는 멈추지 않고 요청만 하도록 바꿨다.
- **무작위 순서는 공식 예제와 다르다.** 공식은 `random() > 0.5`가 엎어짐, 여기는 `random() < p`가 엎어짐이라 같은 시드라도 순서가 반대다.
- Edge 헤드리스 스크린샷은 `#앵커` 주소에서 빈 화면이 나오고, 창 폭 500px 미만은 지원하지 않는다. 구역별 확인은 `docs/_out/`에 구역만 남긴 사본을 만들어 찍었다.
