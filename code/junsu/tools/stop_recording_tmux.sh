#!/usr/bin/env bash
# start_recording_tmux.sh로 시작한 녹화를 멈춘다 -- tmux 세션 안의 record_all.sh에
# Ctrl+C를 보내서 bag을 정상적으로 마무리(meta.txt 생성 등)시키고, 끝나면 보여준다.
set -u
SESSION=gosu_record

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "★ '$SESSION' 세션이 없음 — 이미 끝났거나 시작을 안 한 상태."
  exit 1
fi

echo "녹화 중지 신호(Ctrl+C) 전송 -- bag 마무리 중, 몇 초 걸릴 수 있음..."
tmux send-keys -t "$SESSION" C-c

for i in $(seq 1 20); do
  sleep 1
  if tmux capture-pane -t "$SESSION" -p 2>/dev/null | grep -q '녹화 끝남'; then
    break
  fi
done

echo
echo "=== 녹화 결과(최근 출력) ==="
tmux capture-pane -t "$SESSION" -p -S -40
echo
echo "=== 종료 ==="
tmux kill-session -t "$SESSION" 2>/dev/null
echo "세션 정리 완료. bag/뎁스스냅샷 저장 위치는 위 출력의 '저장:'/'뎁스 스냅샷:' 줄 참고."
