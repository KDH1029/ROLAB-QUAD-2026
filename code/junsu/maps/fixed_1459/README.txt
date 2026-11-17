gosu 경기장 지도 — 11-04 14:59 녹화(gosu_20261104_145915_arena3), FAST-LIO 다시 돌림 (2026-11-05)
고친 점: FAST-LIO 입력에서 로봇 15 m 넘는 점 제거(preprocess.max_range 15 와 같음)
  → 자세 출렁임 중앙 2.74 → 0.49 cm, 상위 1% 13.5 → 2.0 cm. 벽이 한 줄로 선명.
좌표계: FAST-LIO camera_init (출발점 = 원점, 기울기 보정 안 함). 저장 좌표(real_waypoints_venue.json)와 같은 원점 — 그대로 겹침.
파일
  *_full.pcd   지도 전체(2번 이상 찍힌 복셀, x y z float32 binary)
  *_arena.pcd  로봇 경로 ±2 m 안만(멀리 전시장·사람 잘라냄)
  5cm / 2cm    복셀 크기
  waypoints_on_goodmap_1459.png  저장 좌표를 지도 위에 올린 그림 (rabarout·park 는 벽 0.09·0.10 m — 좌표 오차 의심)
  abfar_render.png               원본 vs 먼 점 제거 비교
