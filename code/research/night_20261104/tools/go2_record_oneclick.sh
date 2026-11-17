#!/usr/bin/env bash
# Go2 전체 원시 데이터 녹화 원클릭 (11-04 v2 — 묻거나 멈추는 조건 없음, 전부 자동)
#   로봇 PC(내부 PC) 시계 맞춤 → 로봇 PC 에서 D435i 드라이버 켜기 → NUC 에서 XT16 드라이버 켜기 → 전 토픽 녹화
#   문제가 있어도 묻지 않고 로그만 남기고 계속한다. 나중에 뜬 토픽(카메라 등)도 녹화기가 자동으로 붙잡는다.
#
# 사용:   go2_record_oneclick.sh <이름> ["메모"]
#   예:   go2_record_oneclick.sh arena2 "경기장, 계단·경사로·단 포함, 라바콘 5개"
# 입력:   로봇 PC ssh 비밀번호(처음 한 번), 로봇 PC sudo 비밀번호(시계 맞출 때) — 둘 다 사용자가 직접
# 끝내기: Ctrl+C (두 번 눌러도 메타·시각 기록은 남는다). 드라이버는 켜 둔 채로 끝난다(끄는 명령은 마지막에 출력).
# 환경변수(선택): CAM_PROFILE=848x480x30  CAM=0(카메라 안 켬)  SET_CLOCK=0(시계 안 맞춤, 차이만 기록)  DRY=1  ROBOT=unitree@192.168.123.18
set -u
NAME="${1:?사용: go2_record_oneclick.sh <이름> [\"메모\"]}"; MEMO="${2:-}"
ROBOT="${ROBOT:-unitree@192.168.123.18}"; P="${CAM_PROFILE:-848x480x30}"
TS=$(date +%Y%m%d_%H%M%S); BAGNAME="gosu_${TS}"; BAGROOT="$HOME/bags"; OUT="$BAGROOT/$BAGNAME"
LOGDIR="$HOME/ws/night_20261104/logs/oneclick_$TS"; mkdir -p "$LOGDIR" "$BAGROOT"
CM="$HOME/.ssh/go2_cm_%r@%h:%p"
SSH=(ssh -o ControlPath="$CM" -o ConnectTimeout=6 -o ServerAliveInterval=5 -o ServerAliveCountMax=3)
log(){ echo "[$(date +%T)] $*" | tee -a "$LOGDIR/oneclick.log"; }
run(){ if [ "${DRY:-0}" = 1 ]; then log "(DRY) $*"; else "$@"; fi; }
W0=$SECONDS; [ "${DRY:-0}" = 1 ] || until { [ "$(cat /sys/class/net/robot-br/carrier 2>/dev/null)" = 1 ] && ping -c 1 -W 1 192.168.123.18 >/dev/null 2>&1; } || [ $((SECONDS-W0)) -ge "${WAIT_ROBOT:-120}" ]; do sleep 1; done   # 링크 없이 env_drive 를 읽으면 LOCALHOST 로 고정돼 끝까지 로봇 토픽을 못 받는다
echo "[$(date +%T)] 로봇 링크·내부 PC 대기 $((SECONDS-W0)) s"
source ~/gosu/env_drive.sh >/dev/null 2>&1 || source /opt/ros/jazzy/setup.bash
log "=== Go2 원시 데이터 녹화  이름 $NAME  →  $OUT  (도메인 ${ROS_DOMAIN_ID:-0}) ==="
case "${ROS_DOMAIN_ID:-0}" in 77|78|79|80|81) log "★ 도메인 $ROS_DOMAIN_ID 은 재생용 — 0 으로 바꿔 녹화"; export ROS_DOMAIN_ID=0;; esac
[ "${DRY:-0}" = 1 ] || { ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 3; }   # 로봇 꺼졌을 때 뜬 도메인 0 데몬이 토픽 목록을 틀리게 보인다 → 새로 띄우게

# ── 0) 상태 기록 (멈추지 않음) ─────────────────────────────────────────────────
ROBOTPC=1
for h in 161:로봇본체 18:로봇PC 20:XT16; do
  ip=${h%%:*}; nm=${h##*:}
  if ping -c 1 -W 1 192.168.123.$ip >/dev/null 2>&1; then log "  ✓ $nm 192.168.123.$ip 응답"
  else log "  ★ $nm 192.168.123.$ip 무응답(계속 진행)"; [ "$ip" = 18 ] && ROBOTPC=0; fi
done
for i in $(ls /sys/class/net/robot-br/brif 2>/dev/null); do
  [ "$(cat /sys/class/net/$i/carrier 2>/dev/null)" = 1 ] && log "  링크 $i $(cat /sys/class/net/$i/speed 2>/dev/null) Mb/s"
done
m18=$(ip neigh show 192.168.123.18 dev robot-br 2>/dev/null | awk '{for(i=1;i<NF;i++) if($i=="lladdr"){print $(i+1); exit}}')
p18=$([ -n "$m18" ] && bridge fdb show br robot-br 2>/dev/null | awk -v m="$m18" 'tolower($1)==tolower(m){print $3; exit}')
s18=$(cat /sys/class/net/${p18:-none}/speed 2>/dev/null || echo 0)
log "  로봇 PC 쪽 포트 ${p18:-?} ${s18} Mb/s"
if [ -n "$p18" ] && [ "${s18:-0}" -gt 0 ] && [ "${s18:-0}" -lt 1000 ] && [ -z "${CAM_PROFILE:-}" ]; then
  P=424x240x6; LOWLINK=1; log "  ★ 로봇 PC 포트 ${s18} Mb/s — 카메라를 $P 컬러+깊이로 낮춤(라이다·lowstate 보호). 원본 화질은 go2_record_split.sh 로"; fi
log "  rmem_max $(sysctl -n net.core.rmem_max) · 디스크 여유 $(df -h "$BAGROOT" | awk 'NR==2{print $4}') (카메라 포함 10분 약 40~60 GB)"
command -v chronyc >/dev/null && log "  NUC 시각 동기: $(chronyc tracking 2>/dev/null | awk -F': ' '/System time/{print $2}')"

# ── 1) 로봇 PC ssh (비밀번호 한 번, 2시간 유지) ────────────────────────────────
if [ "$ROBOTPC" = 1 ]; then
  if "${SSH[@]}" -O check "$ROBOT" >/dev/null 2>&1; then log "로봇 PC 연결 재사용"
  else
    log "로봇 PC($ROBOT) 연결 — 비밀번호를 입력하세요(한 번만)"
    run timeout --foreground 90 "${SSH[@]}" -o ControlMaster=auto -o ControlPersist=2h -fN "$ROBOT" || { log "★ 로봇 PC ssh 실패 — 카메라 없이 계속"; ROBOTPC=0; }
  fi
fi
rsh(){ { [ "${DRY:-0}" = 1 ] || [ "$ROBOTPC" = 0 ]; } && return 0; timeout 20 "${SSH[@]}" -o BatchMode=yes "$ROBOT" "$@"; }
cam_up(){ [ "${DRY:-0}" = 1 ] && return 1; timeout 6 ros2 topic list 2>/dev/null | grep -qx /camera/color/image_raw; }
cam_proc(){ rsh 'pgrep -af "[r]ealsense2_camera_node"' </dev/null 2>/dev/null; }

# ── 2) 시각: 로봇 PC 시계를 NUC 에 맞춤(기본) + 차이 기록 ───────────────────────
clock_offset(){   # 출력: 로봇PC−NUC 초 (왕복 ms, 로봇PC 날짜)
  { [ "${DRY:-0}" = 1 ] || [ "$ROBOTPC" = 0 ]; } && { echo "(측정 안 함)"; return; }
  local a b r best="" i   # 5번 재서 왕복이 가장 짧은 것(한 번만 재면 왕복 310 ms 짜리가 걸려 시계가 0.16 s 어긋났다 — 11-04)
  for i in 1 2 3 4 5; do a=$(date +%s.%N); r=$(rsh 'date +%s.%N' </dev/null 2>/dev/null | tail -1); b=$(date +%s.%N)
    [[ "$r" =~ ^[0-9]+\.[0-9]+$ ]] || continue
    if [ -z "$best" ] || python3 -c "import sys; x='$best'.split(); sys.exit(0 if $b-$a < float(x[1])-float(x[0]) else 1)"; then best="$a $b $r"; fi
  done
  [ -n "$best" ] || { echo "측정실패(${r:0:40})"; return; }
  set -- $best; a=$1; b=$2; r=$3
  python3 -c "a,b,r=$a,$b,$r; import time; print(f'{r-(a+b)/2:+.3f} s  (왕복 {1000*(b-a):.0f} ms, 로봇PC {time.strftime(\"%Y-%m-%d %H:%M:%S\", time.localtime(r))})')"
}
OFF0=$(clock_offset); log "시각: 로봇PC−NUC = $OFF0"
d=${OFF0%% s*}
if [ "${SET_CLOCK:-1}" = 1 ] && [ "$ROBOTPC" = 1 ] && [[ $d =~ ^[+-]?[0-9]+\.[0-9]+$ ]]; then
  if python3 -c "import sys; sys.exit(0 if abs($d) < 0.05 else 1)"; then log "  이미 0.05 s 안 — 시계 그대로"
  elif cam_up; then log "  ★ 카메라가 이미 켜져 있어 시계를 안 바꾼다(켜진 카메라 stamp 가 깨짐) — 차이만 기록"
  else
    log "로봇 PC 시계를 NUC 에 맞춤 — 로봇 PC sudo 비밀번호를 입력하세요"
    if run timeout --foreground 60 "${SSH[@]}" -t "$ROBOT" "sudo -v && sudo date -s @\$(python3 -c 'import time; print(\"%.6f\" % (time.time() - ($d)))') >/dev/null && echo set"; then
      OFF0="$(clock_offset) (맞춘 뒤)"
    else log "  ★ 시계 맞춤 실패 — 차이만 기록하고 계속"; fi
    log "시각: 로봇PC−NUC = $OFF0"
  fi
fi

# ── 3) 카메라 D435i (로봇 PC, Foxy, realsense 4.51.1) ───────────────────────────
STARTED_CAM=0
CAM_ARGS="enable_color:=true enable_depth:=true enable_infra1:=true enable_infra2:=true enable_gyro:=false enable_accel:=false"
CAM_ARGS="$CAM_ARGS rgb_camera.profile:=$P depth_module.profile:=$P rgb_camera.color_profile:=$P depth_module.depth_profile:=$P depth_module.infra_profile:=$P"
# 4.51.1 은 *.profile, 4.54+ 는 *_profile — 둘 다 넘김(선언 안 된 쪽은 무시됨). 카메라 IMU 는 켜면 USB 가 끊겨서 끔.
[ "${LOWLINK:-0}" = 1 ] && CAM_ARGS="${CAM_ARGS/enable_infra1:=true enable_infra2:=true/enable_infra1:=false enable_infra2:=false}"
cam_start_menu(){   # ① 로그인 메뉴 foxy(1) 그대로 — 사람이 하던 방식
  printf '1\nsetsid nohup ros2 launch realsense2_camera rs_launch.py %s > /tmp/go2_rs_%s.log 2>&1 < /dev/null &\nsleep 1\nexit\n' "$CAM_ARGS" "$TS" \
    | timeout --foreground 40 "${SSH[@]}" -o BatchMode=yes -tt "$ROBOT" >> "$LOGDIR/cam_start_menu.log" 2>&1 || log "  ① 40초 초과/실패($LOGDIR/cam_start_menu.log)"
}
cam_start_direct(){ # ② 환경 직접 지정(foxy + ~/cyclonedds_ws, eth0)
  timeout --foreground 40 "${SSH[@]}" -o BatchMode=yes "$ROBOT" 'bash --noprofile --norc -s' >> "$LOGDIR/cam_start_direct.log" 2>&1 <<EOF

source /opt/ros/foxy/setup.bash
for w in ~/cyclonedds_ws ~/unitree_ros2/cyclonedds_ws; do [ -f \$w/install/setup.bash ] && source \$w/install/setup.bash; done
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
setsid nohup ros2 launch realsense2_camera rs_launch.py $CAM_ARGS > /tmp/go2_rs_$TS.log 2>&1 < /dev/null &
sleep 1; echo started
EOF
}
cam_kill(){ rsh 'pkill -INT -f "[r]s_launch.py"; pkill -INT -f "[r]ealsense2_camera_node"; sleep 3; pkill -KILL -f "[r]ealsense2_camera_node"' </dev/null >/dev/null 2>&1; }
cam_wait(){ for i in $(seq 1 25); do cam_up && return 0; sleep 1; done; return 1; }
if [ "${CAM:-1}" = 0 ]; then log "카메라: 끔(CAM=0)"
elif [ "$ROBOTPC" = 0 ]; then log "★ 로봇 PC 에 못 붙어 카메라를 못 켬 — 라이다·로봇 데이터만 녹화(카메라가 뜨면 자동으로 붙음)"
elif [ "${DRY:-0}" = 1 ]; then log "(DRY) 카메라 ①/② 켜기 건너뜀"
elif cam_up; then log "카메라: 이미 켜져 있음 — 재사용"
else
  rsh 'grep -n -A6 -E "foxy|noetic" ~/.bashrc 2>/dev/null | head -40' </dev/null > "$LOGDIR/robotpc_bashrc_menu.txt" 2>&1   # 진단용
  cp_=$(cam_proc); [ -n "$cp_" ] && { log "  로봇 PC 에 NUC 에서 안 보이는 카메라 노드가 있음 → 끄고 다시 켬: $cp_"; cam_kill; }
  STARTED_CAM=1
  log "카메라 켜기 ① 로그인 메뉴 foxy(1)"; cam_start_menu
  if ! cam_wait; then log "카메라 켜기 ② 환경 직접 지정"; cam_kill; cam_start_direct; cam_wait || true; fi
  if cam_up; then log "  ✓ 카메라 토픽 보임"
  else rsh "tail -20 /tmp/go2_rs_${TS}.log" </dev/null > "$LOGDIR/robotpc_rs.log" 2>&1
       log "  ★ 카메라 토픽이 아직 안 보임(로봇 PC 로그 $LOGDIR/robotpc_rs.log) — 그대로 녹화, 뜨면 자동으로 붙음"; fi
fi
W=; RC=; RD=
if cam_up; then
  W=$(timeout 6 ros2 topic echo --once /camera/color/camera_info 2>/dev/null | awk '/^width:/{w=$2}/^height:/{h=$2}END{print w"x"h}')
  RC=$(timeout 7 ros2 topic hz /camera/color/image_raw 2>/dev/null | awk '/average rate/{r=$3}END{print r}')
  RD=$(timeout 7 ros2 topic hz /camera/depth/image_rect_raw 2>/dev/null | awk '/average rate/{r=$3}END{print r}')
  log "  카메라: 컬러 ${W:-?} ${RC:-?} Hz · 깊이 ${RD:-?} Hz  (요청 $P)"
  python3 -c "import sys; sys.exit(0 if min(float('${RC:-0}' or 0), float('${RD:-0}' or 0)) >= 5 else 1)" || log "  ★ 영상이 거의 안 온다 — USB·노드 확인 필요(계속 진행)"
  [ "${W:-}" = "${P%x*}" ] || log "  ★ 컬러 해상도 ${W:-?} ≠ 요청 $P(계속 진행)"
fi

# ── 4) XT16 드라이버 (NUC) ─────────────────────────────────────────────────────
STARTED_XT=0; nd=$(pgrep -c -x hesai_ros_drive)
xt_start(){ setsid nohup bash -c "source ~/gosu/env_drive.sh >/dev/null 2>&1; exec ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args -r __ns:=/hesai_ros_driver" > "$LOGDIR/xt16_$1.log" 2>&1 < /dev/null & }
xt_hz(){ [ "${DRY:-0}" = 1 ] && return; timeout 7 ros2 topic hz /lidar_points 2>/dev/null | awk '/average rate/{r=$3}END{print r}'; }
xt_ok(){ python3 -c "import sys; sys.exit(0 if float('${RX:-0}' or 0) >= 18 else 1)"; }
if [ "${nd:-0}" -ge 1 ]; then log "XT16 드라이버: 이미 ${nd}개"
else log "XT16 드라이버 켜기"; [ "${DRY:-0}" = 1 ] || xt_start 1; STARTED_XT=1; sleep 4; fi
RX=$(xt_hz)
if ! xt_ok && [ "${DRY:-0}" != 1 ]; then   # 로봇이 꺼졌다 켜지면 예전 드라이버가 죽은 채 남거나 둘이 겹친다 → 깨끗이 하나만 다시
  log "  XT16 ${RX:-0} Hz — 드라이버를 모두 끄고 하나만 다시 켬"; pkill -INT -x hesai_ros_drive
  for i in $(seq 1 10); do pgrep -x hesai_ros_drive >/dev/null || break; sleep 1; done
  pgrep -x hesai_ros_drive >/dev/null && { pkill -KILL -x hesai_ros_drive; sleep 1; }
  xt_start 2; STARTED_XT=1; sleep 5; RX=$(xt_hz)
fi
log "  XT16 /lidar_points ${RX:-0} Hz"; xt_ok || log "  ★ XT16 18 Hz 미만(계속 진행, 로그 $LOGDIR/xt16_*.log)"

# ── 5) 녹화 — 전 토픽 원시 데이터 ─────────────────────────────────────────────
LIST=$([ "${DRY:-0}" = 1 ] && echo "" || timeout 10 ros2 topic list -t 2>/dev/null)
# 이 NUC 에 정의가 없는 메시지 형식은 녹화기가 토픽 찾기를 멈추게 한다(10-29 확인) → 그 형식만 뺀다(기술적으로 불가피)
KNOWN="unitree_arm/msg/ArmString unitree_go/msg/ConfigChangeStatus unitree_go/msg/VoxelMapCompressed unitree_interfaces/msg/PcToImage unitree_interfaces/msg/QtCommand unitree_interfaces/msg/QtEdge unitree_interfaces/msg/QtNode"   # 10-29·10-30·11-04 bag 에서 확인된 정의 없는 형식 — 녹화 도중에 뜰 토픽도 미리 뺀다
MISSING=$(comm -23 <({ sed -E 's/.*\[(.*)\]/\1/' <<<"$LIST" | tr ',' '\n'; tr ' ' '\n' <<<"$KNOWN"; } | sed 's/ //g' | grep . | sort -u) \
                   <(ros2 interface list 2>/dev/null | sed 's/^ *//' | sort -u) | tr '\n' ' ')
XTYPES=(); [ -n "${MISSING// /}" ] && XTYPES=(--exclude-topic-types $MISSING)
# 카메라 압축본만 뺀다 — 원본 영상은 전부 녹화. 압축본을 구독하면 로봇 PC 가 실시간 압축하느라 원본까지 끊길 수 있다
EXCL='^/camera/.*/(compressed|compressedDepth|theora)$'
log "녹화 시작 → $OUT   (토픽 $(grep -c . <<<"$LIST")개, 정의 없어 뺀 형식: ${MISSING:-없음})"
log "끝내기: Ctrl+C · 주행 요령: 시작·끝 같은 자리 10초 정지, 같은 구간 여러 번, 기립 후 30초 안 출발"
[ "${DRY:-0}" = 1 ] && { log "(DRY) ros2 bag record --all-topics --exclude-regex '$EXCL' ${XTYPES[*]} -o $OUT"; exit 0; }
trap ':' INT   # Ctrl+C 는 녹화기만 끝내고 아래 기록은 끝까지('' 로 하면 녹화기가 Ctrl+C 를 못 받으니 금지)
( while sleep 30; do a=$(df --output=avail -BG "$BAGROOT" | tail -1 | tr -dc 0-9); [ "${a:-999}" -lt 10 ] && { echo "[$(date +%T)] ★ NUC 디스크 ${a}G — 녹화 정상 종료"; pkill -INT -f "[b]ag record.*$BAGNAME"; break; }; done ) & DW=$!
ros2 bag record -s mcap --storage-preset-profile zstd_fast --all-topics \
  --exclude-regex "$EXCL" "${XTYPES[@]}" -d 300 \
  --custom-data "note=${MEMO//=/:}" "name=${NAME//=/:}" "host=$(hostname)" "cam=$P" \
  -o "$OUT"
RCREC=$?
kill $DW 2>/dev/null

# ── 6) 기록 — 나중에 이 bag 을 쓸 사람이 조건을 알 수 있게 ───────────────────────
OFF1=$(clock_offset)
if [ -d "$OUT" ]; then
  {
    echo "이름     $NAME"; echo "메모     $MEMO"
    echo "녹화     $(date '+%F %T') 종료, 호스트 $(hostname), 도메인 ${ROS_DOMAIN_ID:-0}, 녹화기 종료 코드 $RCREC"
    echo "커널     $(uname -r)"
    echo; echo "── 로봇 PC 시계 − NUC 시계 (카메라 stamp 보정용)"; echo "시작 $OFF0"; echo "끝   $OFF1"
    echo; echo "── 카메라 요청 $P · 시작 때 실제 컬러 ${W:-?} ${RC:-?} Hz · 깊이 ${RD:-?} Hz · XT16 ${RX:-?} Hz · 로봇 PC 포트 ${p18:-?} ${s18} Mb/s"
    echo; echo "── 시각 동기 (NUC chrony)"; chronyc tracking 2>/dev/null | grep -E 'Reference|System time|Last offset' || echo "chrony 정보 없음"
    echo; echo "── 커널 소켓 버퍼"; sysctl net.core.rmem_max net.core.wmem_max 2>/dev/null
    echo; echo "── 설정 파일 md5"; md5sum ~/ws/gosu_slam/config/*.yaml ~/ws/hesai_ws/src/*/config/config.yaml 2>/dev/null
    echo; echo "── 정의가 없어 제외한 형식"; echo "${MISSING:-없음}"
    echo; echo "── 녹화 시작 때 토픽 목록"; echo "$LIST"
    echo; echo "── bag 정보"; ros2 bag info "$OUT" 2>/dev/null
  } > "$OUT/meta.txt"
  cp "$LOGDIR"/*.log "$LOGDIR"/*.txt "$OUT/" 2>/dev/null
  echo; log "요약:"; ros2 bag info "$OUT" 2>/dev/null | grep -E 'Duration|Bag size|Messages' | tee -a "$LOGDIR/oneclick.log"
  ros2 bag info "$OUT" 2>/dev/null | awk '
    BEGIN { ref["/lidar_points"]=20; ref["/utlidar/cloud"]=15; ref["/lowstate"]=500; ref["/camera/depth/image_rect_raw"]=30; ref["/camera/color/image_raw"]=30; ref["/camera/infra1/image_rect_raw"]=30 }
    /Duration:/ { d=$2; sub(/s$/, "", d) }
    match($0, /Topic: [^ ]+ /) { t=substr($0, RSTART+7, RLENGTH-8); seen[t]=1
      if ((t in ref) && d > 0 && match($0, /Count: [0-9]+/)) { hz=substr($0, RSTART+7, RLENGTH-7)/d
        printf "  %s %s %.1f Hz (기대 %d Hz)\n", (hz < 0.9*ref[t] ? "★" : "✓"), t, hz, ref[t] } }
    END { for (t in ref) if (!(t in seen)) printf "  ★ %s 기록 안 됨\n", t }' | tee -a "$LOGDIR/oneclick.log"
else log "★ 녹화 폴더가 없다(녹화기 종료 코드 $RCREC)"; fi
log "끝. 저장: $OUT"
log "드라이버는 켜 둔 상태. 끄려면 — 카메라: ~/ws/night_20261104/tools/go2_ssh.sh 'pkill -INT -f realsense2_camera_node'   XT16: pkill -INT -x hesai_ros_drive"
