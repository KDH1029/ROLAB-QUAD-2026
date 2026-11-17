#!/usr/bin/env bash
# nuc16에서 "랜선 뽑고 로봇 몰고 다니다가 나중에 다시 연결해서 녹화 끄기" 워크플로우용.
# tmux 세션 안에서 record_all_with_camera.sh를 돌려서, SSH 연결(=랜선)이 끊겨도
# 녹화는 nuc16 위에서 계속 살아있게 한다. 세션 이름은 고정(gosu_record)이라
# 나중에 다시 붙을 때 외울 필요 없음.
#
# 시작: ssh로 nuc16 접속 후
#   ~/ws/junsu/tools/start_recording_tmux.sh <이름> "<메모>"
# (랜선 뽑고 로봇 몰고 다녀도 녹화는 계속됨 — nuc16 자체 디스크에 저장되는 거라
#  네트워크가 전혀 필요없음)
#
# ★ 2026-11-04: 팀 record_all.sh는 필수 센서 토픽이 녹화 시작 시점에 아직 안
# 떠있으면 "그래도 녹화할까? [y/N]" 하고 사람 입력을 기다리며 멈춘다 — 아무도
# 안 보고 있으면 녹화가 시작도 안 된 채로 그 질문 앞에서 계속 멈춰있게 됨. 그래서
# 이 스크립트는 세션을 띄운 뒤 최대 20초 기다리면서 실제로 녹화가 시작됐는지
# (또는 질문에 걸려서 멈췄는지) 확인하고 결과를 명확하게 알려준다 — "✓ 시작됨"이
# 찍히기 전까지는 랜선을 뽑지 말 것.
#
# 나중에 돌아와서 랜선 다시 꽂고 ssh 재접속 후, 상태 확인+종료:
#   ~/ws/junsu/tools/stop_recording_tmux.sh
# (또는 수동으로: tmux attach -t gosu_record  ->  Ctrl+C  ->  Ctrl+B,D 로 나가기)

set -u
SESSION=gosu_record

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "★ 이미 '$SESSION' 세션이 돌고 있음 — 중복 시작 안 함."
  echo "  상태 확인: tmux attach -t $SESSION"
  exit 1
fi

NAME="${1:?사용: start_recording_tmux.sh <이름> \"<메모>\"}"
NOTE="${2:-메모 없음}"

tmux new-session -d -s "$SESSION" \
  "~/ws/junsu/tools/record_all_with_camera.sh '$NAME' '$NOTE'; echo; echo '[녹화 끝남 -- 아무 키나 누르면 이 창 닫힘]'; read -n1"

echo "세션 띄움 — 센서 점검 + 녹화 진입 확인 중(최대 20초, 기다릴 것)..."
STATUS="unknown"
for i in $(seq 1 10); do
  sleep 2
  PANE=$(tmux capture-pane -t "$SESSION" -p -S -200 2>/dev/null)
  if echo "$PANE" | grep -q '\[y/N\]'; then
    STATUS="stuck_prompt"
    break
  fi
  if echo "$PANE" | grep -q '^저장: '; then
    STATUS="recording"
    break
  fi
done

case "$STATUS" in
  recording)
    echo "✓ 실제로 녹화 시작 확인됨(이름: $NAME) — 이제 랜선 뽑아도 됨."
    echo "$PANE" | grep '^저장: '
    ;;
  stuck_prompt)
    echo "★★★ 멈춰있음 — 필수 센서 토픽이 아직 안 떠서 '그래도 녹화할까?' 질문에 걸림!"
    echo "    지금 랜선 뽑으면 녹화가 시작도 안 된 채로 끝남. 아래로 확인하고 답할 것:"
    echo "    tmux attach -t $SESSION"
    echo "    (y 입력하거나, 센서가 왜 안 뜨는지 먼저 확인할 것. 답한 뒤 Ctrl+B,D로 나가기)"
    ;;
  *)
    echo "★ 20초 안에 '녹화 시작' 신호를 못 봄 — 아직 센서 점검 중일 수 있음."
    echo "    랜선 뽑기 전에 직접 확인할 것: tmux attach -t $SESSION  (나가기 Ctrl+B,D)"
    echo "--- 현재 화면 ---"
    echo "$PANE" | tail -20
    ;;
esac

echo
echo "나중에 돌아와서 끄려면: ~/ws/junsu/tools/stop_recording_tmux.sh"
