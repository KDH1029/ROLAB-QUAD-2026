#!/usr/bin/env bash
# 내부 PC 에 둔 카메라 녹화 조각을 나중에 한 번에 받아 오기 + 시각 보정(내부 PC 시계 → NUC 시계) (11-05)
#   go2_record_auto.sh 는 끝날 때 영상을 옮기지 않고 대기 목록(pending_camera.tsv)에 적어 둔다(TRANSFER=1 이면 예전처럼 바로 옮김).
#
# 사용:   go2_record_fetch.sh              # 대기 목록·크기·NUC 디스크 여유만 보여 줌
#         go2_record_fetch.sh all          # 전부 받기
#         go2_record_fetch.sh <이름일부>   # 그 녹화만(예: arena3, 20261105_1530)
# 결과:   ~/bags/gosu_<시각>_<이름>/camera_robotpc_pN/ (원본) · camera_nuctime_pN/ (NUC 시계로 고친 것, 라이다와 바로 같이 씀)
# 끊겨도 다시 실행하면 이어 받는다(rsync --partial). 내부 PC 원본은 지우지 않는다 — 다 받은 뒤 지우려면 끝에 나오는 명령을 직접.
set -u
TOOLS="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
Q="$TOOLS/pending_camera.tsv"; DONE="$TOOLS/fetched_camera.tsv"
ROBOT="${ROBOT:-unitree@192.168.123.18}"; CM="$HOME/.ssh/go2_cm_%r@%h:%p"
SSH=(ssh -o "ControlPath=$CM" -o ControlMaster=auto -o ControlPersist=10m -o ConnectTimeout=6 -o BatchMode=yes)
rsh(){ "${SSH[@]}" "$ROBOT" "$@" </dev/null; }

[ -s "$Q" ] || { echo "대기 중인 카메라 조각 없음 ($Q)"; exit 0; }
ping -c1 -W1 "${ROBOT#*@}" >/dev/null 2>&1 || { echo "★ 내부 PC(${ROBOT#*@}) 응답 없음 — 로봇 전원·랜선 확인"; exit 1; }
rsh true || { echo "★ ssh 실패 — 비밀번호 없이 쓰려면 한 번: ssh-copy-id $ROBOT"; exit 1; }

SEL="${1:-}"
mapfile -t LINES < <(grep -v '^\s*$' "$Q")
echo "대기 목록 ${#LINES[@]}개:"
TOT=0; PICK=()
for L in "${LINES[@]}"; do
  IFS=$'\t' read -r OUT DIR PN <<<"$L"
  B=$(rsh "du -sb ~/$DIR 2>/dev/null | cut -f1"); B=${B:-0}
  H=$(numfmt --to=iec "$B" 2>/dev/null || echo "$B")
  st="받을 것"; [ "$B" = 0 ] && st="★ 내부 PC 에 없음"
  [ -f "$OUT/camera_nuctime_$PN/time_fix.json" ] && st="이미 받음(보정까지)"
  printf '  %-45s %-4s %8s  %s\n' "$(basename "$OUT")" "$PN" "$H" "$st"
  if [ -n "$SEL" ] && { [ "$SEL" = all ] || [[ "$OUT" == *"$SEL"* ]]; } && [ "$st" = "받을 것" ]; then PICK+=("$L"); TOT=$((TOT + B)); fi
done
FREE=$(df -B1 --output=avail "$HOME" | tail -1)
echo "NUC 디스크 여유 $(numfmt --to=iec "$FREE")"
[ -n "$SEL" ] || { echo; echo "받으려면: $0 all   또는   $0 <이름일부>"; exit 0; }
[ ${#PICK[@]} -gt 0 ] || { echo "받을 것 없음(이름 '$SEL' 에 맞는 대기 조각 없음)"; exit 0; }
[ $((TOT * 2 + 5 * 1024 ** 3)) -lt "$FREE" ] || { echo "★ 디스크 부족: 받을 것 $(numfmt --to=iec $TOT) ×2(원본+보정본) + 여유 5G > 남은 $(numfmt --to=iec "$FREE")"; exit 1; }

set +u; source /opt/ros/jazzy/setup.bash; set -u
echo; echo "=== ${#PICK[@]}개 받기 ($(numfmt --to=iec $TOT)) $(date +%T)"
for L in "${PICK[@]}"; do
  IFS=$'\t' read -r OUT DIR PN <<<"$L"
  echo "── $(basename "$OUT") $PN"
  ok=0
  for k in 1 2 3; do
    rsync -a --partial --info=progress2 -e "${SSH[*]}" "$ROBOT:$DIR/" "$OUT/camera_robotpc_$PN/" 2>&1 | tail -1
    [ -f "$OUT/camera_robotpc_$PN/metadata.yaml" ] && { ok=1; break; }
    echo "  옮기기 실패($k/3) — 다시"; sleep 3
  done
  [ "$ok" = 1 ] || { echo "  ★ 옮기기 실패 — 나중에 다시 실행(이어 받음)"; continue; }
  if [ -f "$OUT/clock_probe.csv" ]; then
    echo "  시각 보정 중…"
    python3 "$TOOLS/go2_camera_bag_fix.py" "$OUT/camera_robotpc_$PN" "$OUT/clock_probe.csv" "$OUT/camera_nuctime_$PN" 2>&1 | grep -v '^\[INFO\]' | tail -1
  else
    echo "  ★ $OUT/clock_probe.csv 없음 — 시각 보정 못 함(원본만)"
  fi
  if [ -f "$OUT/camera_nuctime_$PN/time_fix.json" ]; then
    grep -vF "$L" "$Q" > "$Q.tmp"; mv "$Q.tmp" "$Q"; printf '%s\t%s\n' "$L" "$(date '+%m-%d %H:%M')" >> "$DONE"
    echo "  ■ 끝 → $OUT/camera_nuctime_$PN"
  else
    echo "  ★ 보정 실패 — 원본은 받음: python3 $TOOLS/go2_camera_bag_fix.py $OUT/camera_robotpc_$PN $OUT/clock_probe.csv $OUT/camera_nuctime_$PN"
  fi
done
echo; echo "끝 $(date +%T). 남은 대기 $(grep -c . "$Q" 2>/dev/null || echo 0)개."
echo "내부 PC 원본 지우기(다 받은 뒤, 직접): $TOOLS/go2_ssh.sh 'ls ~/bags'  확인 후  'rm -r ~/bags/<폴더>'"
