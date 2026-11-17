rec1650 — 11-05 16:50 녹화로 만든 경기장 지도 (junk 에서 만든 것을 복사, 원본 ~/ws/junk/routes/rec1650)
  bag      : ~/bags/gosu_20261105_165023_rec   (복사 안 함 — 원본 그대로)
  만든 법  : junk tools/make_map.sh  (map_replay 도메인 78 lo 전용 → FAST-LIO 2 cm 복셀 → 벽 4 cm 칸, 녹화 길 0.18 m 안 점=로봇·사람 제거)
  좌표     : 이 녹화를 시작한 자리·방향이 원점(camera_init). maps/align/arena1459_* 와 좌표가 다르다(1459 경로와 섞지 말 것)
  map.pgm/yaml : 0.03 m, origin [-12.660, -12.980] — nav2 map_server·AMCL·출발 정렬(--map)에 그대로 쓸 수 있음
  map.npz      : 2 cm 복셀 점(지도 원본) · editor_cache.npz : 벽·바닥·녹화 길(route_editor 용)
  route.json   : 녹화 길 초안(route_v1, 60점, 정지선·주차 지정 전 초안) — v2 route_map.json 형식 아님
  비교     : 첫 녹화 rec1645 보다 벽 얇음(중앙 10.7 cm vs 12.4), rec1645 출발 자리 정렬 2.7/−2.1 cm·−2.39°·84%
