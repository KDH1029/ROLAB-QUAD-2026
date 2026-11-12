# research

대회 스택에 넣지 않은 실험·검증 코드.

| 폴더 | 내용 |
|---|---|
| `night_20261104/backup_v2/` | 백업 위치추정 (다리 odom 예측 + 수평 보정 스캔 3D 정합, small_gicp). p95 5 cm |
| `night_20261104/ws/src/terrain_veto/` | 경사로·계단 오인식 제거 (C++). 가짜 장애물 13.9% → 1.0% |
| `night_20261104/ramp/cliff/` | 높은 단 가장자리 낙차 감지 |
| `night_20261104/vision/` | D435i 외부파라미터 보정 |
| `go2_record/` | 대회장 녹화 자동화 (`go2_record_auto.sh`) |
| `v_gosu/tools/record_all.sh` | 전 토픽 녹화 + 재현 조건 기록 |
| `v_gosu/backup/` | 백업 위치추정 v1 |
| `v_gosu/eval/` | XT16 / L1 지도 비교 |
| `lab_visual/` | 3D ICP 위치추정, 사전 장애물 층 생성·정렬 |
| `gosu_slam/` | 사전 지도 없는 SLAM 구성 |
| `archbench/` | FAST-LIO 지연 측정 |
| `go2_3d/` | 초기 지도 메타데이터, SC-PGO 결과 |

## Notes

- 대부분 `~/ws/<원래 폴더>` 경로 기준
- 재생 스크립트는 별도 DDS 도메인 사용, 로봇 연결 시 실행 거부
- `night_20261104/vision/CALIB.md`의 카메라 각도(하향 34°)는 11-04 기준. 11-06에는 수평에 가깝게 변경됨
