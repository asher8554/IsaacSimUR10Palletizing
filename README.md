# IsaacSimUR10Palletizing

Isaac Sim 6.2의 Robotics Examples **UR10 Palletizing**을 빈 스테이지에서 직접 구현해 보는 단계별 실습 코드와 가이드.

**가이드: [`docs/guide.html`](docs/guide.html)** 를 브라우저로 연다. 0단계부터 순서대로 따라 한다.

## 내용

| 단계 | 실행 스크립트 | 새로 쓰는 모듈 | 확인하는 것 |
|---|---|---|---|
| 1 | `step01_load_robot.py` | – | 외부 UR10을 불러와 홈 자세로 세운다 |
| 2 | `step02_build_scene.py` | `palletizing/scene.py` | 컨베이어·뒤집기 장치·팔레트 배치 |
| 3 | `step03_conveyor.py` | `scene.py` (컨베이어) | 박스가 0.30 m/s로 실려 간다 |
| 4 | `step04_spawn_bins.py` | `common.py`, `spawner.py` | 박스가 무작위 자세로 나온다 |
| 5 | `step05_perception.py` | `perception.py` | 집을 자세와 뒤집기 여부 계산 |
| 6 | `step06_gripper.py` | `gripper.py` | 흡착으로 붙였다 떨어뜨린다 |
| 7 | `step07_motion.py` | `motion.py` | RMPflow로 컵 끝을 목표로 보낸다 |
| 8 | `step08_pick_place.py` | `tasks.py`, `cell.py` | 박스 하나를 집어 팔레트에 놓는다 |
| 9 | `step09_palletizing.py` | `fsm.py` | 뒤집기 포함 9개 적재 |

## 실행

Isaac Sim 6.2.0이 `C:\isaacsim-6.2`에 있다고 가정한다. Isaac Sim 모듈은 내장 Python에서만 불러올 수 있으므로 별도 가상환경을 만들지 않고 `python.bat`으로 실행한다. 추가로 설치할 패키지는 없다. 에셋을 내려받으므로 인터넷이 필요하다.

```powershell
cd E:\Github\IsaacSimUR10Palletizing

# 창을 띄워 보기
C:\isaacsim-6.2\python.bat step09_palletizing.py

# 창 없이 자동 확인 ([step09] PASS 가 찍히면 성공)
C:\isaacsim-6.2\python.bat step09_palletizing.py --test --headless --seed 5 --max-bins 2
```

옵션: `--headless` 창 없이, `--test` 자동 확인 후 종료, `--seed N` 박스 순서 고정, `--max-bins N` (9단계) 쌓을 개수.

## 검증

2026-11-23, Isaac Sim 6.2.0-rc.14 · Windows 11 · RTX 3070에서 step01~09를 `--test --headless`로 실행해 모두 PASS를 확인했다. 9단계는 뒤집기가 포함된 순서(`--seed 5 --max-bins 2`)와 전체 9개 적재(`--seed 1`)를 각각 확인했다.

## 가이드 다시 만들기

`docs/guide.html`은 `docs/guide.src.html`의 코드 자리표시자를 실제 소스로 채워 만든다. 소스를 고쳤으면 다시 빌드한다.

```powershell
python docs\build_guide.py                                              # 아무 Python 3.9+
C:\isaacsim-6.2\python.bat docs\capture_shots.py --headless --seed 5    # 가이드 화면 다시 찍기 (선택)
```

## 출처

동작 로직과 수치는 Isaac Sim에 포함된 NVIDIA 예제 `isaacsim.robot_motion.examples/.../manipulation/ur10_palletizing.py`(Apache-2.0)를 기능별 모듈로 나눠 옮긴 것이다.
