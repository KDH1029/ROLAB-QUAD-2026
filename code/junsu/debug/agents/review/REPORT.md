# review 에이전트 보고 (11-06 08시대) — 메인 세션이 받아 적음(에이전트는 보고 파일 쓰기가 막혀 메시지로 보냄)

## 결론
가장 큰 위험은 관리자 상태 파일이다. `maps/segments/logs/go2_ctl_state.json`(07:54 저장)의 route 가 옛 `route_map.json`(11-05 14:15, 16점, 슬라럼 대체 길 없음)이었고, 관리자는 이 저장값으로 코드 기본값을 덮었다(B7).

## 버그(심각도 순, 줄 번호는 08시 v2 기준)
| 번호 | 심각도 | 파일 | 내용 | 처리 |
|---|---|---|---|---|
| B7 | 상 | tools/go2_ctl.py | 상태 파일이 기본값(경로 등)을 통째로 덮음 | **적용**(OPS_VER 다르면 저장 작전 버림) |
| B8 | 중하 | tools/go2_ctl.py | 필터 off 여도 a 가 필터 노드를 띄움 | **적용** |
| B6 | 상(넘어진 뒤) | src/junsu_loc_node.py | FALL_REALIGN 까지 '자세 낮음'으로 봐서 재정렬을 걷는 중으로 미룸 | **적용**(FALLEN 만) |
| B3 | 중상 | runner offroute_check | 되짚기 포기 직후 다시 되짚기(경로에서 먼 자리로) 반복 | 시뮬 에이전트 끝난 뒤 적용 |
| B4 | 중상 | runner back_out_toward | 0.64 m 보다 좁은 통로에서 옆벽을 '앞'으로 봐 0.06 m/s(문턱 아래)로 제자리 | 〃 |
| B1 | 중 | runner target | '벽 건넘' 문턱 0.08 m 때문에 벽 0.08 m 안 점은 보정이 꺼짐(16:54 대비 퇴행) | 〃 |
| B2 | 중(CPU) | runner target | 보정 계산 틱당 ~16 ms | 〃 |
| B5 | 중 | runner 벽 추종 | parking→wall1 1~2 cm 구간이라 끝 판정이 안 걸리고 ego 없이 계속 걸을 수 있음 | 〃 |
| B9~B12 | 하 | runner | 정렬 거부 시 재정렬 대기, 넘어짐 뒤 pin_mode 잔류, 회전 15° 시계 안 지움, cb_align_T 뒤 옛 좌표 | 〃 |

확인했지만 버그 아님: PIN_TRACK 은 항상 빠져나옴, slalom_done 초기화, 걸음새 보류, ±180° 감김, None·빈 배열 가드, 관리자 put/ask 한글 폭, RUNNER_* 전달.

## 실기 위험 순위(16:54 대비)
1. 경로 파일(B7)
2. 경로 이탈 되짚기(새로 생김; 첫 실기에선 `RUNNER_OFF_ROUTE_M=9` 로 끄는 것도 고려)
3. 슬라럼 대체 길 auto
4. 목표 보정·미리 넘어가기 변경
5. 회전 규칙
6. 주차 뒤 순서
7. 정지선 빨강 40초
8. ego 설정

운영 위험: 출발 정렬 문턱 0.30 m 에 16:54 T x +0.288 m — 아슬아슬하다. 거부되면 경로가 0.3 m·4° 밀린 채 간다.

## 파일
`real_course_runner.patch`(B1~B5, B9~B12) · `junsu_loc_node.patch`(B6) · `go2_ctl.patch`(B7, B8) · `patched/`(적용 사본) · `test_review_repro.py`(원본 8개 재현, 패치본 0개; `REVIEW_SRC=patched`) · `run_orig.txt`·`run_patched.txt`
