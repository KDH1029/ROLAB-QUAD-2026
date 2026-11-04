# ROLAB-QUAD-2026 GitHub 협업 및 형상 관리 규칙 (Git Convention)

본 문서는 2026학년도 4족보행로봇 경진대회(`ROLAB-QUAD-2026`) 팀이 코드 충돌과 환경 종속 문제 없이 개발·튜닝·현장 배포를 수행하기 위한 형상 관리 규칙입니다.

---

## 1. 브랜치 전략: GitHub-Flow

본 프로젝트는 단일 메인 브랜치 중심의 **GitHub-Flow**를 채택합니다.

- **`main`**: 실제 Go2 로봇 또는 폐루프 시뮬레이터(`--sim`)에서 빌드 및 정상 동작이 검증된 기준(Production) 브랜치입니다.
- **작업 브랜치**: 새로운 기능 개발, 버그 수정, 파라미터 튜닝 시 항상 최신 `main`에서 분기(Checkout)합니다.

### 1.1 브랜치 명명 규칙 (Naming Convention)
작업 도메인과 성격에 따라 접두어를 분리합니다.
- `core/<기능명>`: 하드웨어 브릿지, 기구학 지터 필터, 로컬라이제이션, Nav2 등 기반 기술 수정 (예: `core/slope-scan-filter`)
- `mission/<미션명>`: 대회 개별 과제 시퀀스 및 Behavior Tree 제어 로직 개발 (예: `mission/hill-crossing`)
- `tune/<대상>`: 주행 제어 게인, 경사 임계값, 지도 파일 등 파라미터/맵 튜닝 (예: `tune/coex-hill-params`)
- `fix/<버그명>`: 런타임 오류 및 긴급 버그 수정 (예: `fix/tf-stamp-overflow`)
- `docs/<문서명>`: 아키텍처 문서 및 매뉴얼 업데이트

---

## 2. 디렉토리 물리적 분리 및 소유권 규칙

다수의 인원이 동시에 작업할 때 병합 충돌(Merge Conflict)을 방지하기 위해 워크스페이스 디렉토리 역할을 엄격히 분리합니다.

```text
ROLAB-QUAD-2026/
├── src/
│   ├── core/                    # [기반 기술 담당] 브릿지(go2_bridge), 위치추정(go2_localization), 경로계획(go2_navigation)
│   ├── missions/                # [미션 제어 담당] 대회 과제별 행동 제어 및 상위 미션 패키지
│   └── interfaces/              # [공통 인터페이스] Unitree ROS 2 메시지 패키지 (unitree_api, unitree_go)
├── config/                      # [공통 설정] 통합 시스템 튜닝 파라미터 (system_params.yaml) 및 웨이포인트 (targets.json)
├── scripts/                     # [실행 도구] 매핑/오도메트리/자율주행 실행 스크립트 및 CLI 도구
├── maps/                        # [지도 자산] 변환 완료된 2D 점유격자(.pgm, .yaml) 및 2.5D 고도 맵(_elevation.npz)
└── docs/                        # [문서] 시스템 아키텍처 블루프린트 및 협업 규칙
```

1. **도메인 격리**:
   - 기반 기술 담당자는 `src/core/` 및 `scripts/` 하위만 수정합니다.
   - 미션 제어 담당자는 `src/missions/` 하위만 수정합니다.
2. **공통 인터페이스 변경 협의**:
   - `src/interfaces/` 메시지 정의나 `config/system_params.yaml`의 키(Key) 구조를 변경할 때는 팀원 간 사전 공유 후 1인이 전담하여 반영합니다.

---

## 3. 코드 및 파라미터 관리 규칙

### 3.1 하드코딩 금지 (No Hardcoding)
- **절대 경로 금지**: 코드 및 런치 파일 내부에 `/home/kdh/...` 등 특정 PC 사용자 계정에 종속된 절대 경로를 작성하지 않습니다. 반드시 `get_package_share_directory()` 또는 `Path(__file__).resolve()` 기준 상대 경로를 사용합니다.
- **매직 넘버 금지**: 속도 제한, 경사각 임계값, 라이다 장착 오프셋 등 현장 튜닝이 필요한 수치는 C++/Python 소스코드에 직접 기입하지 않고 `config/system_params.yaml` 및 `src/core/go2_bridge/config/bridge_params.yaml`로 분리합니다.

### 3.2 빌드 산출물 및 대용량 파일 커밋 금지
- `build/`, `install/`, `log/`, `__pycache__/` 디렉토리는 절대 Git에 커밋하지 않습니다 (`.gitignore` 적용).
- 수백 MB 이상의 원시 데이터(`*.bag`, `*.db3`, `*.mcap`, `*.pcd`)는 저장소 용량 초과 및 클론 속도 저하를 유발하므로 원격 저장소에 올리지 않습니다.
- `maps/` 디렉토리에는 경량 지도 파일(`.pgm`, `.yaml`)만 커밋합니다.

---

## 4. 커밋 메시지 규칙 (Commit Convention)

커밋 메시지는 아래 접두어를 사용하여 변경 목적을 명확히 기록합니다.

- `feat:` 새로운 기능 또는 노드/스크립트 추가 (예: `feat: 12관절 FK 기반 지터 보정 필터 추가`)
- `fix:` 버그 및 예외 처리 수정 (예: `fix: 타임스탬프 나노초 반올림 오버플로우 수정`)
- `tune:` 주행 속도, 경사각, 필터 게인 등 파라미터 및 맵 튜닝 (예: `tune: 언덕 교차로 max_walkable_slope_deg 18도로 조정`)
- `refactor:` 동작 변경 없는 코드 구조 개선 및 정리
- `docs:` 문서 및 주석 수정
- `chore:` 빌드 설정(`CMakeLists.txt`, `package.xml`), `.gitignore` 등 유지보수

---

## 5. Pull Request (PR) 및 현장(Field) 운용 프로토콜

### 5.1 평상시 개발 프로토콜
1. 작업 브랜치에서 코드 수정 후 PR 생성 전 아래 **필수 로컬 검증**을 통과해야 합니다:
   ```bash
   colcon build --symlink-install
   python3 -m py_compile scripts/*.py
   bash -n scripts/*.sh
   ```
2. GitHub에 PR을 생성하고 **최소 1명 이상의 팀원 리뷰 및 승인(Approve)** 후 `main`에 병합합니다.
3. 병합이 완료된 작업 브랜치는 즉시 삭제합니다.

### 5.2 대회 현장(Field) 긴급 대응 프로토콜
- 경기장 현장에서 진행되는 파라미터 미세 튜닝(`config/*.yaml`) 및 현장 지도(`maps/*`) 갱신에 한해, 시간 지연을 막기 위해 PR 절차를 생략하고 `main` 브랜치에 직접 커밋(`tune: ...`) 및 `push`하는 것을 허용합니다.
- 단, 로봇 내장 PC 또는 원격 조종 PC에서 직접 푸시하기 직전 반드시 팀원들에게 구두로 변경 내역을 공유하여 다른 PC에서 즉시 `git pull --rebase`를 수행하도록 합니다.
