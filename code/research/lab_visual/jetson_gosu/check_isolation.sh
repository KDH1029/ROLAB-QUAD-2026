#!/usr/bin/env bash
# 격리가 정말 되는지 **켜기 전에** 확인한다. 실패하면 아무것도 실행하지 않는다.
#
#   ./check_isolation.sh
#
# 보는 것
#   ① 환경변수가 제대로 걸렸는가
#   ② 도메인 77 에 팀원 노드가 하나도 안 보이는가
#   ③ 도메인 0(팀원 쪽)에 무엇이 떠 있는가 — 참고용으로만 본다
#   ④ 로봇이 연결돼 있는가 — 연결돼 있어도 격리 때문에 영향 없음을 확인
set -u
ok=0

echo "══ 격리 점검 ══"
echo "① 환경"
for v in ROS_DOMAIN_ID RMW_IMPLEMENTATION; do
    printf '   %-22s %s\n' "$v" "${!v:-(안 걸림)}"
done
if [ "${ROS_DOMAIN_ID:-0}" != "77" ]; then
    echo "   ★ 도메인이 77 이 아니다. env.sh 를 source 했는가?"; ok=1
fi
if [ -z "${CYCLONEDDS_URI:-}" ]; then
    echo "   ★ CYCLONEDDS_URI 가 비어 있다 — 루프백 제한이 안 걸렸다"; ok=1
else
    case "$CYCLONEDDS_URI" in
        *'name="lo"'*) echo "   CYCLONEDDS_URI        lo 전용 ✓";;
        *) echo "   ★ CYCLONEDDS_URI 가 lo 전용이 아니다"; ok=1;;
    esac
fi

echo
echo "② 우리 도메인(77)에 보이는 노드 — 비어 있거나 우리 것만 있어야 한다"
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1 </dev/null
mine=$(timeout 8 ros2 node list 2>/dev/null)
if [ -z "$mine" ]; then echo "   (없음) ✓"; else echo "$mine" | sed 's/^/   /'; fi
for n in fastlio ego_planner go2_bridge mission go2_3d_loc; do
    if echo "$mine" | grep -qi "$n"; then
        echo "   ★ 팀원 노드로 보이는 것이 우리 도메인에 있다: $n"; ok=1
    fi
done

echo
echo "③ 팀원 도메인(0) 상태 — 참고용. 여기 있는 것은 우리가 못 보고, 우리를 못 본다"
theirs=$(ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 CYCLONEDDS_URI= timeout 8 ros2 node list 2>/dev/null | head -20)
if [ -z "$theirs" ]; then echo "   (없음 — 팀원 스택이 안 떠 있다)"; else echo "$theirs" | sed 's/^/   /'; fi

echo
echo "④ 로봇 연결"
if ping -c1 -W1 192.168.123.18 >/dev/null 2>&1; then
    echo "   내부PC(192.168.123.18) 응답함 — 하지만 우리 토픽은 lo 밖으로 안 나간다"
else
    echo "   내부PC 무응답"
fi

echo
if [ $ok -eq 0 ]; then
    echo "✔ 격리됨. 실행해도 된다."
else
    echo "✗ 격리가 안 됐다. 아무것도 실행하지 마라."
fi
exit $ok
