# Go2 사족보행 자율주행 — 로보월드 2026 (우수상)

Unitree Go2 EDU 사족보행 로봇으로 실내 장애물 코스를 자율주행한 대회 프로젝트의 코드와 기록이다.
2026-08 말부터 2026-11-06 본선까지의 작업을 담았다.

## 결과
- **350점 / 만점 450 — 우수상**
- 규정 점수표: 정지선 2곳 100 · 장애물 100 · 주차 50 · 완주 200. 완주 없이 받을 수 있는 최고점이 250이므로, 채점 회차는 완주로 인정됐다.
- 러너 로그 기준 실주행(11-05~06): 출발 17번
  - 완주 판정 2번: 11-05 11:31 215초, 11-05 16:54 134초
  - 출구까지 갔지만 통과 판정 전 정지 2번: 11-06 11:19 168초, 11-06 14:15 136초
  - 자세한 기록: [docs/05_실주행_기록.md](docs/05_실주행_기록.md)

## 코스와 과제
8 × 6 m 경기장. 다음 과제를 순서대로 수행한다.
- 출발 → 정지선 1(신호등) → 라바콘 슬라럼 → X자 교차로(가운데 높은 단, 계단·경사로) → 정지선 2 → 주차(4다리 접고 5초) → Out Line
- 연산·전원은 전부 로봇 위에서 처리해야 한다. 경기 중 센서 데이터를 밖으로 보내면 안 된다.
- 대회 중(11-05) 규정이 바뀌었다: 콘을 고정하고 대형(높이 0.7~0.8 m)으로 교체해, 학습해 둔 YOLO로는 콘을 인식할 수 없게 됐다.

## 시스템 한눈에
```
센서      Hesai XT16(16빔) · Go2 내장 L1 · 몸통 IMU 500 Hz · 다리 기구학 odom · D435i · 앞머리 카메라
            │
위치추정  FAST-LIO(다리 속도 결합·모션 게이팅, max_range 15)
          + AMCL(Omni 이동모델, 이동 입력 = FAST-LIO)  + 출발 정렬(첫 3초 스캔 ↔ 지도)
            │
인식      LiDAR 콘 추적기 · YOLO 신호등(내부 PC, TensorRT FP16)
            │
계획·미션 ┌ 자율주행 미션 플래너: EGO-Planner + 코스 러너 (Rgosu → junsu)
          └ 기록 경로 반복 미션 플래너: 한 번 몰아 기록한 경로를 따라감 (junk → juninsu)
            │
제어      go2_bridge → Unitree Sport API (ClassicWalk, 직진 쏠림 보정, 최소 유효 명령)
연산      NUC16(Core Ultra 7, ROS 2 Jazzy) + 로봇 내부 PC(Jetson Orin NX, Foxy: 카메라·YOLO)
```

## 팀
정해진 역할 분담 없이, 모든 작업공간을 팀원들이 함께 작업했다. 이름은 책임자 또는 주 기여자다.

| 구분 | 폴더 | 책임자 | 내용 |
|---|---|---|---|
| 기반 기술 | `code/merge_tmp` | 김도현 | 위치추정(AMCL Omni·FAST-LIO odom·다리 결합 FAST-LIO), 출발 정렬, EGO 최적화, LiDAR 콘 추적기, 카메라 캘리브레이션 |
| 기반 기술 | `code/yolo` | 송인애 | 신호등·콘 인식 모델 |
| 미션 플래너(자율주행) | `code/rgosu` | 고수영 | 통합 주행 스택의 틀: FAST-LIO + EGO-Planner + 코스 러너 |
| 미션 플래너(자율주행) | `code/junsu` | 한준희 | Rgosu 틀에 기반 기술을 묶은 통합 스택 |
| 미션 플래너(기록 경로 반복) | `code/junk` → `code/juninsu` | 김도현 → 송인애 | 규정 변경 대응. 실시간 경로 계획을 하는 일반적인 자율주행 알고리즘이 아니라 기록 경로 추종 방식 |
| 로봇 보정 | (각 플래너의 모터 설정) | 김도현 → 송인애 | 주행 중 왼쪽 쏠림 피드포워드 보정 |

## 폴더
| 경로 | 내용 |
|---|---|
| [docs/01_시스템_구성.md](docs/01_시스템_구성.md) | 하드웨어·네트워크·소프트웨어 구성, 기술 계보 |
| [docs/02_기술_구현.md](docs/02_기술_구현.md) | 분야별 구현과 실측 성능 |
| [docs/03_트러블슈팅.md](docs/03_트러블슈팅.md) | 문제 → 원인 → 해결 → 근거 숫자 |
| [docs/04_설계_결정.md](docs/04_설계_결정.md) | 갈림길마다 고른 것과 버린 것, 그 근거 |
| [docs/05_실주행_기록.md](docs/05_실주행_기록.md) | 러너 로그에서 뽑은 11-05~06 실주행 18건 |
| [docs/06_타임라인.md](docs/06_타임라인.md) | 2026-08 ~ 11 주요 사건 |
| [docs/SOURCES.md](docs/SOURCES.md) | 각 `code/` 폴더의 원본 위치·커밋·가져온 방법 |
| `code/` | 작업공간 6개 + 연구 코드(원본별 시점은 SOURCES 참고) |
| `tools/real_runs_extract.py` | 러너 로그 → 실주행 표 추출기 |

## 코드 재사용 안내
작업공간마다 `README.md` 에 기능, 구조, 의존성, 빌드, 실행, 로봇 없이 시험, 재사용할 때 고칠 곳, 알려진 문제를 같은 틀로 적었다.

| 목적 | 먼저 볼 곳 |
|---|---|
| Go2 + XT16 로 위치추정·경로 계획만 가져다 쓰기 | [`code/merge_tmp`](code/merge_tmp/README.md) (맨 아래 "재사용 가이드") |
| 코스 과제를 자율주행으로 수행하는 전체 스택 | [`code/junsu`](code/junsu/README.md) (원형은 [`code/rgosu`](code/rgosu/README.md)) |
| 고정 코스를 확실히 완주하는 기록 경로 반복 | [`code/juninsu`](code/juninsu/README.md) (단순판·비교는 [`code/junk`](code/junk/README.md)) |
| 신호등·콘 YOLO 학습과 젯슨 TensorRT 배포 | [`code/yolo`](code/yolo/README.md) |
| 백업 위치추정, 경사로 헛장애물 제거, 녹화 도구 | [`code/research`](code/research/README.md) |

### 작업공간 사이의 빌드 의존
미션 플래너들은 다른 작업공간의 빌드 결과(`install/`)를 `source` 해서 쓴다. 가져다 쓸 때는 **merge_tmp 를 먼저 빌드**한다.
```
merge_tmp  (go2_bridge·Hesai 드라이버·Unitree 메시지·AMCL 설정·EGO·콘 추적기)
   ▲ source                       ▲ source
junsu / rgosu                  junk / juninsu
 + ws_fastlio_tunable           + ws_fastlio_tunable
 + ws_ego_review / ws_ego_planner_tunable
 + ws_wall_filter · ws_mission_vision · ws_legfuse_gated
   └ 내부 PC: onboard_vision (YOLO, TensorRT) ── 양쪽 모두
```
- 스크립트의 절대 경로(`~/ws/...`, `~/junsu_vision`)는 원래 개발 PC 기준이다. 각 README 의 "재사용할 때 고칠 곳"에 찾는 명령을 적었다.
- 공통 운영 조건: CycloneDDS, 커널 `rmem_max/wmem_max` 128 MB, 로봇은 엎드려 켜고 기립 후 30초 안에 출발, 재생은 별도 DDS 도메인에서.

## 읽을 때 주의
- 코드는 대회 현장 작업본이라 단독 빌드·실행을 보장하지 않는다. ROS 2 Jazzy, Unitree SDK2, Hesai 드라이버, 로봇 실기가 필요하다.
- 녹화(bag)·3D 지도(pcd)·모델 가중치·엔진 파일은 용량 때문에 빠져 있다(git 이력 안의 작은 지도 npz 는 남아 있음).
- 숫자는 모두 측정 조건과 함께 적었다. 조건 없이 인용하지 말 것(예: "FAST-LIO 오차 ≤ 4 cm"는 슬라럼 밖 4분 재생 기준).
- 외부 코드가 들어 있다: FAST-LIO(GPL-2.0), Hesai ROS 드라이버(Hesai 라이선스), EGO-Planner 등. 공개 전에 각 폴더의 LICENSE 를 확인할 것.
