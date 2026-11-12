#!/usr/bin/env bash
# junk 경로 전용 관리자 — 라이다·브리지·FAST-LIO·비전(신호등)·모터·러너. 기록 주행은 tools/teach.sh
exec "$(dirname "$(readlink -f "$0")")/go2_ctl.sh" "$@"
