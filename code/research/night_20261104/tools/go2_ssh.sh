#!/usr/bin/env bash
# 로봇 PC(젯슨) 바로 접속 — go2_record_oneclick.sh 가 연 연결(2시간 유지)을 재사용해 비밀번호를 다시 묻지 않는다.
# 사용: go2_ssh.sh            (대화형 — 로그인 메뉴에서 1 = foxy)
#       go2_ssh.sh '<명령>'    (명령만 실행)
ROBOT="${ROBOT:-unitree@192.168.123.18}"; CM="$HOME/.ssh/go2_cm_%r@%h:%p"
ssh -o ControlPath="$CM" -O check "$ROBOT" >/dev/null 2>&1 || ssh -o ControlPath="$CM" -o ControlMaster=auto -o ControlPersist=2h -fN "$ROBOT" || exit 1
if [ $# -gt 0 ]; then exec ssh -o ControlPath="$CM" "$ROBOT" "$@"; else exec ssh -o ControlPath="$CM" -t "$ROBOT"; fi
