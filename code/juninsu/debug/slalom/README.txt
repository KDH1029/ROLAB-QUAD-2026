슬라럼 디버깅 (junk 대회 코드는 태그 nail-1105 로 고정 — 이 폴더는 새 파일만, junk 원본은 안 건드림)

준비(터미널 1, junk 관리자):  ~/ws/junk/tools/junk_ctl.sh
  Q → a → 로봇을 #6 자리에 #7 쪽(약 17°)을 보게 놓기 → 3(FAST-LIO 다시 켜기 = 여기가 원점) → 리모컨 기립 → m
실행(터미널 2):  ~/ws/junk/debug/slalom/run.sh            (#6 출발 → #7~#24 → #25 에서 멈춤)
  구간 바꾸기: run.sh 6 25     조정값: SLALOM_V=0.2 SLALOM_LOOK=0.35 SLALOM_WP_R=0.25 SLALOM_TURN=0.6 ~/ws/junk/debug/slalom/run.sh
정지: 터미널 2 Ctrl+C(StopMove) 또는 관리자 x
결과: runs/<시각>/slalom.png(지도·찍은 점·실제 길·몸 윤곽, 빨강 = 몸 0.70×0.31 이 벽·콘에 닿음), runner.log, trace.csv, align.log
점 고치기: ~/venv/go2/bin/python3 ~/ws/junk/tools/route_editor.py rec1650   (run.sh 는 매번 routes/rec1650/route.json 의 #6~#25 를 읽음 → 대회 경로에도 그대로 반영, 저장마다 .bak)
★ 슬라럼 시험 뒤 대회 주행은 로봇을 출발점에 놓고 3(FAST-LIO 다시) → r. 원점이 #6 으로 잡혀 있으면 출발 정렬이 거부됨

파일: real_course_runner.py(junk 러너 사본 + 조정값 환경변수 + 궤적 기록 + 끝나면 종료), start_align/(정렬 사본 + --prior: 경로 중간에서 시작),
      make_route.py(#A~#C 잘라냄), plot.py(그림)
