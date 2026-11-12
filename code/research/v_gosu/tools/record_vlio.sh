#!/usr/bin/env bash
# (10-29 변경) 녹화는 전부 record_all.sh 로 한다 — 카메라 포함 전 토픽 + 조건 기록(meta.txt).
# 이 파일은 옛 이름을 쓰는 사람을 위해 남긴 연결이다.
exec ~/ws/v_gosu/tools/record_all.sh "${1:-vlio}" "${2:-VLIO 비교용: 같은 계단·경사 3회 왕복, 시작·끝 같은 자리 10초 정지}"
