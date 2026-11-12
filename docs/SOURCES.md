# SOURCES — 코드 출처와 가져온 방법

2026-11-09 NUC16 에서 만들었다. **원본 작업공간은 읽기만 했고 바꾸지 않았다.**

## git 저장소였던 것 (원본 커밋 시점의 파일)
아래 커밋 시점의 추적 파일을 그대로 `code/<이름>/` 에 담았다. 커밋 이력은 넣지 않았고, 원본 저장소에 남아 있다.

| 폴더 | 원본(NUC16) | 브랜치 = 커밋 |
|---|---|---|
| `code/merge_tmp` | `~/ws/merge_tmp_20261102/review` (원격 github.com/kdh1029/rolab-quad-2026) | `route/sequencer` = 386d9b0 + 원격 `main` = 7a47cf2(README 갱신) |
| `code/junsu` | `~/ws/junsu` (worktree `~/ws/junsu_v2`) | `v2` = 12aff3e (11-06 13:38 최종, `sudong` 과 같은 커밋) |
| `code/junk` | `~/ws/junk` | `main` = b7728bd (11-06, 검증본 nail-1105 로 되돌린 판) |
| `code/juninsu` | `~/ws/juninsu` | `main` = 2f40691 (11-06 15:32, 본선 회전 실패 수정) |

### 원본 저장소의 주요 고정점
| 원본 | 태그·브랜치 | 가리키는 것 |
|---|---|---|
| `~/ws/junsu` | 태그 `run-1105-1654` (= `main` a438045) | 11-05 16:54 완주(134초) 때의 통합 스택 |
| `~/ws/junsu` | 태그 `v2-1105-1600` | v2 갈래 11-05 16:00 시점 |
| `~/ws/junk` | 태그 `nail-1105` | 기록 경로 반복 방식 검증본(11-05) |
| `~/ws/junk` | 태그 `nail-1106`, `nail-1106b~d` | 11-06 중간 고정점 |
| `~/ws/junk` | 브랜치 `juninsu-dev` (462cd33) | juninsu 가 갈라져 나온 개발 브랜치 |
| `~/ws/merge_tmp_20261102/review` | 브랜치 `push/main` (05b6ea9) | 11-04 팀 원격에 올린 판 |

## 소스만 복사한 것 (git 이 아니었음, 이력 없음)
| 폴더 | 원본(NUC16) | 넣은 것 | 뺀 것 |
|---|---|---|---|
| `code/rgosu` | `~/ws/Rgosu` (68 GB) | 소스·런치·설정·문서·스크립트(1 MB 이하) | `bags/`(38 GB), `maps/` 의 지도 바이너리(25 GB), `venv_yolo/`, `foxy_test/`(빌드 시험용 복사본), `jetson_transfer_bundle*`, `build/install/log`, `*.bak_*` |
| `code/yolo` | `~/ws/datasets` | 학습 노트북, 데이터셋 정의, 학습 지표, 프레임 추출 도구 | 이미지·라벨·가중치(`.pt`)·zip |
| `code/research/*` | `~/ws/{go2_3d,lab_visual,archbench,gosu_slam,v_gosu,night_20261104}` | 소스·설정·문서(300 KB 이하) | 로그, 결과 csv, `maps/`, 녹화, 가상환경, 외부 FAST-LIO 포크 사본 |
| `code/research/go2_record` | `~/go2_record` | 녹화 자동화 스크립트·README·카메라 장착 json | `logs/`, 대기 목록 tsv, `*.bak_*` |

## 넣지 않은 것
- 팀 공용 실험실 `~/ws/lab`(다른 사람 코드의 링크·사본), 외부 원본 포크(`fastlio_ws`, `liosam_ws`, `scpgo_ws`, `hesai_ws`, `livox`, `unitree_ros`)
- 녹화(bag, 수십 GB), 3D 지도(pcd), 모델 가중치·TensorRT 엔진
- `~/ws/junsu_merge`(merge-review 브랜치 4커밋), `~/ws/junsu_v2_bak`(= 태그 `run-1105-1654` + README 1커밋)

## 점검
- 비밀번호·키·토큰: 복사본 전체를 패턴 검색했고, 발견되지 않았다(문서 속 `<새값>` 같은 자리표시만 있음).
- 공개 전에 확인할 것
  - 내부 네트워크 주소(Tailscale)가 있는 파일: `code/rgosu/WORKLOG.md`, `code/junsu/WORKLOG.md`, `code/research/lab_visual/jetson_gosu/run_*.sh`
  - `code/merge_tmp/docs/rulebook/` 에 대회 규정집 PDF 가 있다(배포 허락 확인 필요).
  - 외부 코드 라이선스: FAST-LIO 사본(GPL-2.0), Hesai ROS 드라이버(Hesai 저작권 고지).
