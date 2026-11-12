# Sources

각 `code/` 폴더의 원본과 기준 커밋.

| 폴더 | 원본 | 기준 |
|---|---|---|
| `code/merge_tmp` | `merge_tmp_20261102/review` (github.com/kdh1029/rolab-quad-2026) | `route/sequencer` 386d9b0 + `main` 7a47cf2 |
| `code/junsu` | `junsu` | `v2` 12aff3e (11-06) |
| `code/junk` | `junk` | `main` b7728bd (검증본 `nail-1105`) |
| `code/juninsu` | `juninsu` | `main` 2f40691 (11-06) |
| `code/rgosu` | `Rgosu` | 11-05 소스 (git 아님) |
| `code/yolo` | `datasets` | 학습 노트북·데이터셋 정의·학습 지표 |
| `code/research/*` | `go2_3d`, `lab_visual`, `archbench`, `gosu_slam`, `v_gosu`, `night_20261104`, `go2_record` | 소스·설정·문서 |

## 원본 저장소 태그

| 저장소 | 태그·브랜치 | 내용 |
|---|---|---|
| junsu | `run-1105-1654` (a438045) | 11-05 16:54 완주 시점 |
| junsu | `v2-1105-1600` | v2 11-05 16:00 시점 |
| junk | `nail-1105` | 기록 경로 반복 검증본 |
| junk | `nail-1106`, `nail-1106b~d` | 11-06 중간 시점 |
| junk | `juninsu-dev` (462cd33) | juninsu 분기점 |
| merge_tmp | `push/main` (05b6ea9) | 11-04 원격 반영본 |

## 제외

- 녹화(bag), 3D 지도(pcd), 모델 가중치, TensorRT 엔진
- 빌드 산출물(`build/`, `install/`, `log/`), 가상환경
- 외부 원본 포크(`fastlio_ws`, `liosam_ws`, `scpgo_ws`, `hesai_ws`, `livox`, `unitree_ros`)
