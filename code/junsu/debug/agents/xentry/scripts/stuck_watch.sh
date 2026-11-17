#!/usr/bin/env bash
# 내 판(xentry_*)이 STUCK_HOLD(사람 판단 대기 = 그 판은 끝)로 20초 넘게 있으면 그 판의 내 러너만 끈다 → run_sim_v2 가 정리·보고.
X=$HOME/ws/junsu_v2/debug/agents/xentry
until grep -q CHAIN_DONE $X/chain2.out 2>/dev/null; do
  for p in $(pgrep -f "real_course_runner.py .*runs_v2/xentry_"); do
    tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -q '^ROS_DOMAIN_ID=78$' || continue
    d=$(tr '\0' '\n' < /proc/$p/cmdline | grep route.json | xargs dirname); w=$d/world.jsonl
    n=$(tail -200 $w 2>/dev/null | grep -c '"rs": "STUCK_HOLD"')
    [ "$n" -ge 200 ] && { echo "$(date +%T) STUCK_HOLD 20초 — $d 러너 $p 끔"; kill -TERM $p; }
  done
  sleep 5
done
