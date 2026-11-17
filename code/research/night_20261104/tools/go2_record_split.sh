#!/usr/bin/env bash
# Go2 원시 데이터 녹화 원클릭 — 카메라는 내부 PC(로봇 PC) 디스크에, 나머지는 NUC 에 (11-04, 묻거나 멈추는 조건 없음)
#
#   내부 PC: D435i 드라이버 + 카메라 토픽 녹화(로컬 — 랜으로 영상을 안 보냄 → 대역 제한 없음)
#   NUC   : XT16 드라이버 + 카메라를 뺀 전 토픽 녹화(L1·IMU·다리 오도·lowstate·명령 …)
#   시각  : 녹화 내내 2초마다 내부 PC 시계 − NUC 시계를 잰다(go2_clock_probe.py, 오차 ≈ 왕복/2 < 0.5 ms)
#   끝    : Ctrl+C → 내부 PC 녹화 마무리 → NUC 로 옮김 → 카메라 시각을 NUC 시계로 고친 사본(go2_camera_bag_fix.py)
#
# 사용:   go2_record_split.sh <이름> ["메모"]
#   예:   go2_record_split.sh arena3 "경기장, 계단·경사로·단, 라바콘 5개, 시작·끝에 좌우 비틀기 2초"
# 입력:   내부 PC ssh 비밀번호(처음 한 번), 내부 PC sudo 비밀번호(시계 맞출 때) — 둘 다 사용자가 직접
# 환경변수(선택): CAM_PROFILE=848x480x30 (1280x720x30 도 가능 — 내부 PC 디스크만 충분하면)
#                 SET_CLOCK=1(내부 PC 시계를 sudo 로 맞춤 — 기본은 안 맞추고 녹화 내내 재서 보정, 묻는 것 없음)  TRANSFER=0(옮기기·보정 안 함)  DRY=1  ROBOT=unitree@192.168.123.18
# 결과:   ~/bags/gosu_<시각>/            NUC 녹화 + meta.txt + clock_probe.csv
#         ~/bags/gosu_<시각>/camera_robotpc/   내부 PC 원본(내부 PC 시계)
#         ~/bags/gosu_<시각>/camera_nuctime/   NUC 시계로 고친 카메라 bag(mcap) — 라이다와 바로 같이 쓰는 것
#         (내부 PC 에도 ~/bags/cam_<시각> 원본이 남는다 — 확인 후 지울 것)
set -u
NAME="${1:-rec}"; MEMO="${2:-}"
ROBOT="${ROBOT:-unitree@192.168.123.18}"; P="${CAM_PROFILE:-848x480x30}"
TS=$(date +%Y%m%d_%H%M%S); BAGNAME="gosu_${TS}"; BAGROOT="$HOME/bags"; OUT="$BAGROOT/$BAGNAME"; JBAG="bags/cam_$TS"
TOOLS="$HOME/ws/night_20261104/tools"; LOGDIR="$HOME/ws/night_20261104/logs/split_$TS"; mkdir -p "$LOGDIR" "$BAGROOT"
CM="$HOME/.ssh/go2_cm_%r@%h:%p"
SSH=(ssh -o ControlPath="$CM" -o ConnectTimeout=6 -o ServerAliveInterval=5 -o ServerAliveCountMax=3)
log(){ echo "[$(date +%T)] $*" | tee -a "$LOGDIR/split.log"; }
run(){ if [ "${DRY:-0}" = 1 ]; then log "(DRY) $*"; else "$@"; fi; }
W0=$SECONDS; [ "${DRY:-0}" = 1 ] || until { [ "$(cat /sys/class/net/robot-br/carrier 2>/dev/null)" = 1 ] && ping -c 1 -W 1 192.168.123.18 >/dev/null 2>&1; } || [ $((SECONDS-W0)) -ge "${WAIT_ROBOT:-120}" ]; do sleep 1; done   # 링크 없이 env_drive 를 읽으면 LOCALHOST 로 고정돼 끝까지 로봇 토픽을 못 받는다
echo "[$(date +%T)] 로봇 링크·내부 PC 대기 $((SECONDS-W0)) s"
source ~/gosu/env_drive.sh >/dev/null 2>&1 || source /opt/ros/jazzy/setup.bash
log "=== Go2 녹화(카메라는 내부 PC)  이름 $NAME  →  $OUT  (도메인 ${ROS_DOMAIN_ID:-0}) ==="
case "${ROS_DOMAIN_ID:-0}" in 77|78|79|80|81) log "★ 도메인 $ROS_DOMAIN_ID 은 재생용 — 0 으로 바꿔 녹화"; export ROS_DOMAIN_ID=0;; esac
[ "${DRY:-0}" = 1 ] || { ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 3; }   # 로봇 꺼졌을 때 뜬 도메인 0 데몬이 토픽 목록을 틀리게 보인다

# ── 0) 상태 기록 (멈추지 않음) ─────────────────────────────────────────────────
ROBOTPC=1
for h in 161:로봇본체 18:내부PC 20:XT16; do
  ip=${h%%:*}; nm=${h##*:}
  if ping -c 1 -W 1 192.168.123.$ip >/dev/null 2>&1; then log "  ✓ $nm 192.168.123.$ip 응답"
  else log "  ★ $nm 192.168.123.$ip 무응답(계속 진행)"; [ "$ip" = 18 ] && ROBOTPC=0; fi
done
for i in $(ls /sys/class/net/robot-br/brif 2>/dev/null); do
  [ "$(cat /sys/class/net/$i/carrier 2>/dev/null)" = 1 ] && log "  링크 $i $(cat /sys/class/net/$i/speed 2>/dev/null) Mb/s"
done
log "  NUC 디스크 여유 $(df -h "$BAGROOT" | awk 'NR==2{print $4}') · rmem_max $(sysctl -n net.core.rmem_max)"
command -v chronyc >/dev/null && log "  NUC 시각 동기: $(chronyc tracking 2>/dev/null | awk -F': ' '/System time/{print $2}')"

# ── 1) 내부 PC ssh (비밀번호 한 번, 3시간 유지) ────────────────────────────────
if [ "$ROBOTPC" = 1 ]; then
  if "${SSH[@]}" -O check "$ROBOT" >/dev/null 2>&1; then log "내부 PC 연결 재사용"
  else
    log "내부 PC($ROBOT) 연결 — 비밀번호를 입력하세요(한 번만)"
    run timeout --foreground 90 "${SSH[@]}" -o ControlMaster=auto -o ControlPersist=3h -fN "$ROBOT" || { log "★ 내부 PC ssh 실패 — 카메라 없이 계속"; ROBOTPC=0; }
  fi
fi
rsh(){ { [ "${DRY:-0}" = 1 ] || [ "$ROBOTPC" = 0 ]; } && return 0; timeout 20 "${SSH[@]}" -o BatchMode=yes "$ROBOT" "$@"; }
# 내부 PC 에서 Foxy 환경으로 명령 실행 — menu: 로그인 메뉴에 1 을 먹임(사람이 하던 방식) / direct: foxy + ~/cyclonedds_ws + eth0 직접
jrun(){   # jrun <menu|direct> "<명령>" <로그파일>
  # menu  : 로그인 메뉴 foxy(1) 이 하는 설정을 그대로(~/.bashrc 122~129행, 11-04 확인) — 비대화형으로
  #         (메뉴에 '1' 을 먹이는 대화형 방식은 readline 이 미리 보낸 입력을 버려 명령이 잘렸다 — 11-04 첫 실행)
  # direct: 대체 — eth0 인라인 DDS 설정
  local mode=$1 cmd=$2 lf=$3 dds
  if [ "$mode" = menu ]; then dds='export CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml'
  else dds="export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"eth0\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>'"; fi
  timeout --foreground 60 "${SSH[@]}" -o BatchMode=yes "$ROBOT" 'bash --noprofile --norc -s' >> "$lf" 2>&1 <<EOF

source /opt/ros/foxy/setup.bash
source ~/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
$dds
export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH
$cmd
EOF
}
cam_proc(){ rsh 'pgrep -af "[r]ealsense2_camera_node"' </dev/null 2>/dev/null; }
cam_local_hz(){   # 내부 PC 안에서 카메라가 실제로 데이터를 내는지 — 3초짜리 C++ 시험 녹화로 센다
  # (내부 PC 의 파이썬 ros2 topic hz/echo 는 같은 PC 구독을 못 받는다 — 11-04 확인. 녹화기(C++)는 30 Hz 정상)
  local lf="$LOGDIR/cam_hz_$1.log" d="/tmp/go2_camcheck_${TS}_$1_$RANDOM"; : > "$lf"
  jrun "$1" "timeout -s INT 4 ros2 bag record -o $d /camera/color/camera_info >/dev/null 2>&1; sleep 1; ros2 bag info $d 2>/dev/null | grep -o 'Count: [0-9]*' | head -1" "$lf"
  awk '/Count:/{c=$2}END{if (c+0 > 15) printf "%.1f", c/3.5}' "$lf"
}

# ── 2) 시각: 내부 PC 시계를 NUC 에 맞춤(기본) — 정밀 보정은 녹화 내내 재는 시각 차이로 ─────────
clock_offset(){
  { [ "${DRY:-0}" = 1 ] || [ "$ROBOTPC" = 0 ]; } && { echo "(측정 안 함)"; return; }
  local a b r best="" i   # 5번 재서 왕복이 가장 짧은 것(한 번만 재면 왕복 310 ms 짜리가 걸려 시계가 0.16 s 어긋났다 — 11-04)
  for i in 1 2 3 4 5; do a=$(date +%s.%N); r=$(rsh 'date +%s.%N' </dev/null 2>/dev/null | tail -1); b=$(date +%s.%N)
    [[ "$r" =~ ^[0-9]+\.[0-9]+$ ]] || continue
    if [ -z "$best" ] || python3 -c "import sys; x='$best'.split(); sys.exit(0 if $b-$a < float(x[1])-float(x[0]) else 1)"; then best="$a $b $r"; fi
  done
  [ -n "$best" ] || { echo "측정실패(${r:0:40})"; return; }
  set -- $best; a=$1; b=$2; r=$3
  python3 -c "a,b,r=$a,$b,$r; import time; print(f'{r-(a+b)/2:+.3f} s  (왕복 {1000*(b-a):.0f} ms, 내부PC {time.strftime(\"%Y-%m-%d %H:%M:%S\", time.localtime(r))})')"
}
OFF0=$(clock_offset); log "시각: 내부PC−NUC = $OFF0"
d=${OFF0%% s*}
if [ "${SET_CLOCK:-0}" = 1 ] && [ "$ROBOTPC" = 1 ] && [[ $d =~ ^[+-]?[0-9]+\.[0-9]+$ ]]; then
  if python3 -c "import sys; sys.exit(0 if abs($d) < 0.05 else 1)"; then log "  이미 0.05 s 안 — 시계 그대로"
  elif [ -n "$(cam_proc)" ]; then log "  ★ 카메라 노드가 이미 돌고 있어 시계를 안 바꾼다(stamp 가 깨짐) — 측정·보정만"
  else
    log "내부 PC 시계를 NUC 에 맞춤 — 내부 PC sudo 비밀번호를 입력하세요"
    if run timeout --foreground 60 "${SSH[@]}" -t "$ROBOT" "sudo -v && sudo date -s @\$(python3 -c 'import time; print(\"%.6f\" % (time.time() - ($d)))') >/dev/null && echo set"; then
      OFF0="$(clock_offset) (맞춘 뒤)"
    else log "  ★ 시계 맞춤 실패 — 측정·보정만 하고 계속"; fi
    log "시각: 내부PC−NUC = $OFF0"
  fi
fi

# ── 3) 카메라 드라이버 (내부 PC, Foxy, realsense 4.51.1) ─────────────────────────
CAM_ARGS="enable_color:=true enable_depth:=true enable_infra1:=true enable_infra2:=true enable_gyro:=false enable_accel:=false"
CAM_ARGS="$CAM_ARGS rgb_camera.profile:=$P depth_module.profile:=$P rgb_camera.color_profile:=$P depth_module.depth_profile:=$P depth_module.infra_profile:=$P"
CAM_LAUNCH="setsid nohup ros2 launch realsense2_camera rs_launch.py $CAM_ARGS > /tmp/go2_rs_$TS.log 2>&1 < /dev/null &"
cam_kill(){ rsh 'pkill -INT -f "[r]s_launch.py"; pkill -INT -f "[r]ealsense2_camera_node"; sleep 3; pkill -KILL -f "[r]ealsense2_camera_node"' </dev/null >/dev/null 2>&1; }
MODE=""; RC=""
if [ "$ROBOTPC" = 0 ]; then log "★ 내부 PC 에 못 붙어 카메라를 못 켬 — 라이다·로봇 데이터만 녹화"
elif [ "${DRY:-0}" = 1 ]; then log "(DRY) 카메라 켜기·내부 PC 녹화 건너뜀"; MODE=menu
else
  rsh 'grep -n -A6 -E "foxy|noetic" ~/.bashrc 2>/dev/null | head -40' </dev/null > "$LOGDIR/robotpc_bashrc_menu.txt" 2>&1
  cp_=$(cam_proc)
  if [ -n "$cp_" ]; then   # 이미 돌고 있으면 어느 환경에서 영상이 보이는지 확인해 재사용, 아니면 끄고 다시
    for m in menu direct; do RC=$(cam_local_hz $m); [ -n "$RC" ] && { MODE=$m; break; }; done
    if [ -n "$MODE" ]; then log "카메라: 이미 켜져 있음($MODE 환경에서 컬러 $RC Hz) — 재사용"; else log "  돌고 있는 카메라 노드에서 영상이 안 보임 → 끄고 다시 켬"; cam_kill; fi
  fi
  if [ -z "$MODE" ]; then
    for m in menu direct; do
      log "카메라 켜기 ($m)"; : > "$LOGDIR/cam_start_$m.log"; jrun $m "$CAM_LAUNCH" "$LOGDIR/cam_start_$m.log"
      for i in $(seq 1 8); do sleep 3; RC=$(cam_local_hz $m); [ -n "$RC" ] && break; done
      [ -n "$RC" ] && { MODE=$m; break; }
      log "  $m 로 영상이 안 보임"; cam_kill
    done
  fi
  if [ -n "$MODE" ]; then log "  ✓ 카메라 컬러 $RC Hz (내부 PC 안, $MODE 환경, 요청 $P)"
  else rsh "tail -20 /tmp/go2_rs_$TS.log" </dev/null > "$LOGDIR/robotpc_rs.log" 2>&1; log "  ★ 카메라 영상이 안 나옴(로그 $LOGDIR/robotpc_rs.log) — 라이다·로봇 데이터만 녹화"; fi
fi

# ── 4) 내부 PC 카메라 녹화 시작 ─────────────────────────────────────────────────
JREC=0
if [ -n "$MODE" ]; then
  JAV=$(rsh 'df -BG --output=avail ~ | tail -1' </dev/null 2>/dev/null | tr -dc 0-9)
  STO=$(rsh 'ls /opt/ros/foxy/share 2>/dev/null | grep -qx rosbag2_storage_mcap && echo mcap || echo sqlite3' </dev/null 2>/dev/null | tail -1 | tr -d '\r')
  log "  내부 PC 디스크 여유 ${JAV:-?} GB (848x480x30 4스트림 약 85 MB/s → 약 $(( ${JAV:-0} * 1024 / 85 / 60 ))분) · 저장 형식 ${STO:-sqlite3}"
  CAMTOPICS="/camera/color/image_raw /camera/color/camera_info /camera/color/metadata /camera/depth/image_rect_raw /camera/depth/camera_info /camera/depth/metadata"
  CAMTOPICS="$CAMTOPICS /camera/infra1/image_rect_raw /camera/infra1/camera_info /camera/infra1/metadata /camera/infra2/image_rect_raw /camera/infra2/camera_info /camera/infra2/metadata"
  CAMTOPICS="$CAMTOPICS /camera/extrinsics/depth_to_color /camera/extrinsics/depth_to_infra1 /camera/extrinsics/depth_to_infra2 /tf_static /tf"
  SOPT=""; [ "${STO:-}" = mcap ] && SOPT="-s mcap"
  if [ "${DRY:-0}" != 1 ]; then
    rsh 'mkdir -p ~/bags' </dev/null >/dev/null 2>&1
    : > "$LOGDIR/jrec_start.log"; jrun $MODE "setsid nohup ros2 bag record -o ~/$JBAG $SOPT $CAMTOPICS > /tmp/go2_camrec_$TS.log 2>&1 < /dev/null &" "$LOGDIR/jrec_start.log"
    sleep 4; rsh "pgrep -af '[b]ag record -o .*cam_$TS'" </dev/null > "$LOGDIR/jrec_proc.txt" 2>&1
    if [ -s "$LOGDIR/jrec_proc.txt" ]; then JREC=1; log "  ✓ 내부 PC 카메라 녹화 시작 → ~/$JBAG"
    else rsh "tail -20 /tmp/go2_camrec_$TS.log" </dev/null > "$LOGDIR/jrec_err.log" 2>&1; log "  ★ 내부 PC 녹화가 안 떴다(로그 $LOGDIR/jrec_err.log) — NUC 녹화는 계속"; fi
  fi
fi

# ── 5) XT16 드라이버 (NUC) ─────────────────────────────────────────────────────
nd=$(pgrep -c -x hesai_ros_drive)
xt_start(){ setsid nohup bash -c "source ~/gosu/env_drive.sh >/dev/null 2>&1; exec ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args -r __ns:=/hesai_ros_driver" > "$LOGDIR/xt16_$1.log" 2>&1 < /dev/null & }
xt_hz(){ [ "${DRY:-0}" = 1 ] && return; timeout 7 ros2 topic hz /lidar_points 2>/dev/null | awk '/average rate/{r=$3}END{print r}'; }
xt_ok(){ python3 -c "import sys; sys.exit(0 if float('${RX:-0}' or 0) >= 18 else 1)"; }
if [ "${nd:-0}" -ge 1 ]; then log "XT16 드라이버: 이미 ${nd}개"
else log "XT16 드라이버 켜기"; [ "${DRY:-0}" = 1 ] || xt_start 1; sleep 4; fi
RX=$(xt_hz)
if ! xt_ok && [ "${DRY:-0}" != 1 ]; then
  log "  XT16 ${RX:-0} Hz — 드라이버를 모두 끄고 하나만 다시 켬"; pkill -INT -x hesai_ros_drive
  for i in $(seq 1 10); do pgrep -x hesai_ros_drive >/dev/null || break; sleep 1; done
  pgrep -x hesai_ros_drive >/dev/null && { pkill -KILL -x hesai_ros_drive; sleep 1; }
  xt_start 2; sleep 5; RX=$(xt_hz)
fi
log "  XT16 /lidar_points ${RX:-0} Hz"; xt_ok || log "  ★ XT16 18 Hz 미만(계속 진행)"

# ── 6) 시각 차이 측정 시작 + NUC 녹화(카메라 뺀 전 토픽) ──────────────────────────
PROBE_PID=""
if [ "$ROBOTPC" = 1 ] && [ "${DRY:-0}" != 1 ]; then
  ROBOT="$ROBOT" CM="$CM" setsid python3 "$TOOLS/go2_clock_probe.py" "$LOGDIR/clock_probe.csv" 2 20 > "$LOGDIR/probe.log" 2>&1 < /dev/null & PROBE_PID=$!
  sleep 3; log "  시각 차이 측정 중: $(tail -1 "$LOGDIR/clock_probe.csv" 2>/dev/null)"
fi
LIST=$([ "${DRY:-0}" = 1 ] && echo "" || timeout 10 ros2 topic list -t 2>/dev/null)
KNOWN="unitree_arm/msg/ArmString unitree_go/msg/ConfigChangeStatus unitree_go/msg/VoxelMapCompressed unitree_interfaces/msg/PcToImage unitree_interfaces/msg/QtCommand unitree_interfaces/msg/QtEdge unitree_interfaces/msg/QtNode"   # 10-29·10-30·11-04 bag 에서 확인된 정의 없는 형식 — 녹화 도중에 뜰 토픽도 미리 뺀다
MISSING=$(comm -23 <({ sed -E 's/.*\[(.*)\]/\1/' <<<"$LIST" | tr ',' '\n'; tr ' ' '\n' <<<"$KNOWN"; } | sed 's/ //g' | grep . | sort -u) \
                   <(ros2 interface list 2>/dev/null | sed 's/^ *//' | sort -u) | tr '\n' ' ')
XTYPES=(); [ -n "${MISSING// /}" ] && XTYPES=(--exclude-topic-types $MISSING)
EXCL='^/camera/.*'   # 카메라는 내부 PC 가 녹화한다 — NUC 가 구독하면 영상이 랜으로 넘어와 대역을 먹는다
log "NUC 녹화 시작 → $OUT   (카메라 제외 토픽, 정의 없어 뺀 형식: ${MISSING:-없음})"
log "끝내기: Ctrl+C · 시각 확인용으로 시작·끝에 로봇을 2초간 좌우로 비틀기 · 기립 후 30초 안 출발"
[ "${DRY:-0}" = 1 ] && { log "(DRY) ros2 bag record --all-topics --exclude-regex '$EXCL' ${XTYPES[*]} -o $OUT  +  내부 PC: ros2 bag record -o ~/$JBAG <카메라 토픽>"; exit 0; }
trap ':' INT   # Ctrl+C 는 녹화기만 끝내고 아래 마무리(내부 PC 녹화 끝·옮기기·보정·기록)는 끝까지
( while sleep 30; do a=$(df --output=avail -BG "$BAGROOT" | tail -1 | tr -dc 0-9); [ "${a:-999}" -lt 10 ] && { echo "[$(date +%T)] ★ NUC 디스크 ${a}G — 녹화 정상 종료"; pkill -INT -f "[b]ag record.*$BAGNAME"; break; }
  if [ "$JREC" = 1 ]; then j=$(rsh 'df -BG --output=avail ~ | tail -1' </dev/null 2>/dev/null | tr -dc 0-9); [ -n "$j" ] && [ "$j" -lt 5 ] && { echo "[$(date +%T)] ★ 내부 PC 디스크 ${j}G — 카메라 녹화 정상 종료"; rsh "pkill -INT -f '[b]ag record -o .*cam_$TS'" </dev/null >/dev/null 2>&1; }; fi; done ) & DW=$!
ros2 bag record -s mcap --storage-preset-profile zstd_fast --all-topics \
  --exclude-regex "$EXCL" "${XTYPES[@]}" -d 300 \
  --custom-data "note=${MEMO//=/:}" "name=${NAME//=/:}" "host=$(hostname)" "camera=robotpc:$JBAG" \
  -o "$OUT"
RCREC=$?
kill $DW 2>/dev/null

# ── 7) 마무리: 내부 PC 녹화 끝 → 측정 끝 → 옮기기 → 시각 보정 ─────────────────────
log "NUC 녹화 끝(종료 코드 $RCREC). 내부 PC 카메라 녹화 마무리 중…"
if [ "$JREC" = 1 ]; then
  rsh "pkill -INT -f '[b]ag record -o .*cam_$TS'" </dev/null >/dev/null 2>&1
  for i in $(seq 1 60); do rsh "pgrep -f '[b]ag record -o .*cam_$TS'" </dev/null >/dev/null 2>&1 || break; sleep 1; done
  rsh "ls ~/$JBAG/metadata.yaml" </dev/null >/dev/null 2>&1 && log "  ✓ 내부 PC 녹화 마무리(metadata.yaml 있음)" || log "  ★ 내부 PC 녹화 metadata.yaml 없음 — 나중에 'ros2 bag reindex' 필요할 수 있음"
fi
OFF1=$(clock_offset)
[ -n "$PROBE_PID" ] && { kill -INT "$PROBE_PID" 2>/dev/null; wait "$PROBE_PID" 2>/dev/null; }
mkdir -p "$OUT"; cp "$LOGDIR/clock_probe.csv" "$OUT/" 2>/dev/null
if [ "$JREC" = 1 ] && [ "${TRANSFER:-1}" = 1 ]; then
  SZ=$(rsh "du -sh ~/$JBAG | cut -f1" </dev/null 2>/dev/null | tail -1); log "내부 PC 카메라 녹화 ${SZ:-?} → NUC 로 옮김 (1 Gbps 면 10 GB 에 약 1.5분)"
  if rsh 'command -v rsync' </dev/null >/dev/null 2>&1; then
    rsync -a --info=progress2 -e "ssh -o ControlPath=$CM -o BatchMode=yes" "$ROBOT:$JBAG/" "$OUT/camera_robotpc/" 2>&1 | tail -2
  else scp -r -o ControlPath="$CM" -o BatchMode=yes "$ROBOT:$JBAG" "$OUT/camera_robotpc" >/dev/null; fi
  if [ -f "$OUT/camera_robotpc/metadata.yaml" ]; then
    log "  ✓ 옮김. 카메라 시각을 NUC 시계로 고치는 중…"
    python3 "$TOOLS/go2_camera_bag_fix.py" "$OUT/camera_robotpc" "$OUT/clock_probe.csv" "$OUT/camera_nuctime" 2>&1 | grep -v '^\[INFO\]' | tee -a "$LOGDIR/split.log"
  else log "  ★ 옮기기 실패 — 수동: rsync -a -e 'ssh -o ControlPath=$CM' $ROBOT:$JBAG/ $OUT/camera_robotpc/"; fi
fi

# ── 8) 기록 ──────────────────────────────────────────────────────────────────
{
  echo "이름     $NAME"; echo "메모     $MEMO"
  echo "녹화     $(date '+%F %T') 종료, 호스트 $(hostname), 도메인 ${ROS_DOMAIN_ID:-0}, NUC 녹화기 종료 코드 $RCREC"
  echo "카메라   내부 PC 녹화 ~/$JBAG (요청 $P, 시작 때 컬러 ${RC:-?} Hz, 환경 ${MODE:-없음}, 저장 ${STO:-?}) → camera_robotpc/(원본) · camera_nuctime/(NUC 시계)"
  echo; echo "── 내부 PC 시계 − NUC 시계"; echo "시작 $OFF0"; echo "끝   $OFF1"
  [ -f "$OUT/camera_nuctime/time_fix.json" ] && { echo "보정 모델:"; cat "$OUT/camera_nuctime/time_fix.json"; }
  echo; echo "── XT16 ${RX:-?} Hz · NUC chrony"; chronyc tracking 2>/dev/null | grep -E 'Reference|System time|Last offset'
  echo; echo "── 커널 소켓 버퍼"; sysctl net.core.rmem_max net.core.wmem_max 2>/dev/null
  echo; echo "── 설정 파일 md5"; md5sum ~/ws/gosu_slam/config/*.yaml ~/ws/hesai_ws/src/*/config/config.yaml 2>/dev/null
  echo; echo "── 정의가 없어 제외한 형식"; echo "${MISSING:-없음}"
  echo; echo "── 녹화 시작 때 NUC 토픽 목록"; echo "$LIST"
  echo; echo "── NUC bag 정보"; ros2 bag info "$OUT" 2>/dev/null
  [ -d "$OUT/camera_nuctime" ] && { echo; echo "── 카메라 bag 정보(NUC 시계)"; ros2 bag info "$OUT/camera_nuctime" 2>/dev/null; }
} > "$OUT/meta.txt"
cp "$LOGDIR"/*.log "$LOGDIR"/*.txt "$OUT/" 2>/dev/null
log "요약:"; ros2 bag info "$OUT" 2>/dev/null | grep -E 'Duration|Bag size|Messages' | tee -a "$LOGDIR/split.log"
for t in /lidar_points /lowstate /utlidar/cloud /utlidar/robot_odom /sportmodestate; do ros2 bag info "$OUT" 2>/dev/null | grep -q " $t " || log "  ★ $t 기록 안 됨"; done
[ "$JREC" = 1 ] && for t in /camera/color/image_raw /camera/depth/image_rect_raw /camera/infra1/image_rect_raw; do ros2 bag info "$OUT/camera_nuctime" 2>/dev/null | grep -q " $t " || log "  ★ 카메라 $t 기록 안 됨(또는 옮기기·보정 전)"; done
[ -d "$OUT/camera_nuctime" ] && ros2 bag info "$OUT/camera_nuctime" 2>/dev/null | grep -E 'Duration|Bag size|Messages|camera/color/image_raw|camera/depth/image_rect_raw' | tee -a "$LOGDIR/split.log"
log "끝. 저장: $OUT   (내부 PC 원본 ~/$JBAG 은 확인 후 지울 것: go2_ssh.sh 'rm -rf ~/$JBAG')"
log "드라이버는 켜 둔 상태. 끄려면 — 카메라: $TOOLS/go2_ssh.sh 'pkill -INT -f realsense2_camera_node'   XT16: pkill -INT -x hesai_ros_drive"
