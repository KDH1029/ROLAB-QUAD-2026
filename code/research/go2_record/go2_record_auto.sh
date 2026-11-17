#!/usr/bin/env bash
# Go2 전체 원시 데이터 자동 녹화 (위치: ~/go2_record — 어디서든 go2_record_auto.sh 로 실행) — 묻지 않고 끝까지, 실패하면 다시 시도, 못 고친 문제는 원인·고치는 명령을 알려준다 (11-04)
#
#   내부 PC: D435i 드라이버 + 카메라 토픽 녹화(로컬 디스크 — 랜으로 영상을 안 보냄)
#   NUC   : XT16 드라이버 + 카메라를 뺀 전 토픽 녹화(L1·IMU·다리 오도·lowstate·명령 …)
#   시각  : 녹화 내내 2초마다 내부 PC 시계 − NUC 시계를 잰다(go2_clock_probe.py, 오차 < 0.5 ms) → 끝나고 카메라 시각을 NUC 시계로 보정
#   감시  : 녹화 중 15초마다 카메라 노드·내부 PC 녹화기·XT16 드라이버·시각 측정기를 보고, 죽으면 다시 켠다(카메라 녹화는 새 조각으로 이어 감)
#   끝    : Ctrl+C → 내부 PC 녹화 마무리 → (★ 11-05 기본: 카메라 영상은 내부 PC 에 둠 → 대기 목록 ~/go2_record/pending_camera.tsv)
#           → '문제와 해결 명령' 요약(problems.txt). 영상은 나중에 한 번에: go2_record_fetch.sh all (옮기기 + 시각 보정)
#           TRANSFER=1 이면 예전처럼 끝날 때 바로 옮김(재시도 3번) + 시각 보정
#
# 사용:   go2_record_auto.sh [이름] ["메모"]
#   예:   go2_record_auto.sh arena3 "경기장 주행, 시작·끝에 좌우 비틀기 2초"
# 입력:   없음. (내부 PC ssh 키가 없으면 처음 한 번 비밀번호만 묻는다 — 없애려면 한 번만: ssh-copy-id unitree@192.168.123.18)
# 환경변수(선택): CAM_PROFILE=848x480x30 · CAM=0(카메라 안 켬) · SET_CLOCK=1(내부 PC 시계를 sudo 로 맞춤) · TRANSFER=1(끝날 때 영상 바로 옮김 — 기본은 안 옮김) · WAIT_ROBOT=120 · CAM_USB_WAIT=60 · DRY=1
# 결과:   ~/bags/gosu_<시각>_<이름>/              NUC 녹화 + meta.txt + problems.txt + clock_probe.csv
#         ~/bags/gosu_<시각>/camera_robotpc_pN/   내부 PC 원본(내부 PC 시계)   ·   camera_nuctime_pN/  NUC 시계로 고친 카메라 bag(라이다와 바로 같이 씀)
set -u
NAME="${1:-rec}"; MEMO="${2:-}"
ROBOT="${ROBOT:-unitree@192.168.123.18}"; P="${CAM_PROFILE:-848x480x30}"
TS=$(date +%Y%m%d_%H%M%S); SAFE=$(printf %s "$NAME" | tr -c 'A-Za-z0-9가-힣._-' '_' | cut -c1-40)   # 폴더 이름에 쓸 수 있게(띄어쓰기·/ → _)
BAGNAME="gosu_${TS}_${SAFE}"; BAGROOT="$HOME/bags"; OUT="$BAGROOT/$BAGNAME"; JPRE="bags/cam_${TS}_${SAFE}"
TOOLS="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"; LOGDIR="$TOOLS/logs/auto_$TS"; mkdir -p "$LOGDIR" "$BAGROOT"   # 이 스크립트가 있는 폴더(~/go2_record) 기준
CM="$HOME/.ssh/go2_cm_%r@%h:%p"; SSHP="$TOOLS/go2_ssh.sh"
SSH=(ssh -o ControlPath="$CM" -o ConnectTimeout=6 -o ServerAliveInterval=5 -o ServerAliveCountMax=3)
PROB="$LOGDIR/problems.txt"; JPARTS="$LOGDIR/jparts.txt"; : > "$PROB"; : > "$JPARTS"
log(){ echo "[$(date +%T)] $*" | tee -a "$LOGDIR/auto.log"; }
problem(){   # problem "<무엇>" "<원인>" "<고치는 명령·조치>"
  printf '■ %s\n  원인: %s\n  해결: %s\n\n' "$1" "$2" "$3" >> "$PROB"
  log "★ 문제: $1"; log "    원인: $2"; log "    해결: $3"
}
fixed(){ printf '□ (자동 복구) %s\n\n' "$1" >> "$PROB"; log "  ↺ 자동 복구: $1"; }
run(){ if [ "${DRY:-0}" = 1 ]; then log "(DRY) $*"; else "$@"; fi; }

# ── 0) 로봇 연결 기다리기 (링크 없이 env_drive 를 읽으면 LOCALHOST 로 고정돼 끝까지 로봇 토픽을 못 받는다) ─────
W0=$SECONDS; ROBOTPC=1
if [ "${DRY:-0}" != 1 ]; then
  until { [ "$(cat /sys/class/net/robot-br/carrier 2>/dev/null)" = 1 ] && ping -c 1 -W 1 192.168.123.18 >/dev/null 2>&1; } || [ $((SECONDS-W0)) -ge "${WAIT_ROBOT:-120}" ]; do
    [ $(( (SECONDS-W0) % 10 )) = 0 ] && echo "[$(date +%T)] 로봇 링크·내부 PC 기다리는 중 $((SECONDS-W0)) s …"; sleep 1; done
fi
source ~/gosu/env_drive.sh >/dev/null 2>&1 || source /opt/ros/jazzy/setup.bash
log "=== Go2 자동 녹화  이름 $NAME  →  $OUT  (도메인 ${ROS_DOMAIN_ID:-0}, 연결 대기 $((SECONDS-W0)) s) ==="
case "${ROS_DOMAIN_ID:-0}" in 77|78|79|80|81) log "★ 도메인 $ROS_DOMAIN_ID 은 재생용 — 0 으로 바꿔 녹화"; export ROS_DOMAIN_ID=0;; esac
[ "${DRY:-0}" = 1 ] || { ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 3; }

for h in 161:로봇본체 18:내부PC 20:XT16; do
  ip=${h%%:*}; nm=${h##*:}
  if ping -c 1 -W 1 192.168.123.$ip >/dev/null 2>&1; then log "  ✓ $nm 192.168.123.$ip 응답"
  elif [ "${DRY:-0}" = 1 ]; then log "  (DRY) $nm 무응답"; [ "$ip" = 18 ] && ROBOTPC=0
  else
    case $ip in
      161) problem "로봇 본체(192.168.123.161) 무응답" "로봇 전원 꺼짐 또는 NUC↔로봇 랜 케이블 빠짐" "로봇 전원 확인 · ip -br link (enp86s0/enp87s0 중 하나가 UP 이어야) · ping 192.168.123.161 · 케이블 다시 꽂고 이 스크립트 다시 실행";;
      18)  ROBOTPC=0; problem "내부 PC(192.168.123.18) 무응답 — 카메라 없이 녹화" "내부 PC 부팅 중(전원 켠 뒤 1분 이상 걸림) 또는 랜" "1분 기다린 뒤 ping 192.168.123.18 · 응답하면 이 스크립트 다시 실행";;
      20)  problem "XT16(192.168.123.20) 무응답" "라이다 전원 또는 케이블" "ping 192.168.123.20 · XT16 전원 커넥터 확인";;
    esac
  fi
done
for i in $(ls /sys/class/net/robot-br/brif 2>/dev/null); do
  [ "$(cat /sys/class/net/$i/carrier 2>/dev/null)" = 1 ] && log "  링크 $i $(cat /sys/class/net/$i/speed 2>/dev/null) Mb/s"
done
m18=$(ip neigh show 192.168.123.18 dev robot-br 2>/dev/null | awk '{for(i=1;i<NF;i++) if($i=="lladdr"){print $(i+1); exit}}')
p18=$([ -n "$m18" ] && bridge fdb show br robot-br 2>/dev/null | awk -v m="$m18" 'tolower($1)==tolower(m){print $3; exit}')
s18=$(cat /sys/class/net/${p18:-none}/speed 2>/dev/null || echo 0)
[ -n "$p18" ] && { log "  내부 PC 쪽 포트 $p18 ${s18} Mb/s"; [ "${s18:-0}" -lt 1000 ] && problem "로봇 링크 ${s18} Mb/s (1 Gbps 아님)" "랜 케이블 불량(선 2쌍만 연결) 또는 다른 포트" "다른 랜 케이블로 바꾸기 · cat /sys/class/net/$p18/speed 가 1000 인지 확인 (카메라는 내부 PC 녹화라 괜찮지만 lowstate·L1 이 빠질 수 있음)"; }
FREE=$(df --output=avail -BG "$BAGROOT" | tail -1 | tr -dc 0-9)
log "  NUC 디스크 여유 ${FREE}G · rmem_max $(sysctl -n net.core.rmem_max)"
[ "${FREE:-0}" -lt 30 ] && problem "NUC 디스크 여유 ${FREE}G" "옛 bag 이 쌓임 (NUC 녹화 10분 약 4~6 GB + 카메라 옮기면 약 50 GB)" "du -sh ~/bags/* | sort -h | tail · 옛 bag 을 외장 디스크로 옮기기"
[ "$(sysctl -n net.core.rmem_max)" -lt 26214400 ] && problem "커널 수신 버퍼 작음(rmem_max)" "XT16 UDP 가 30% 유실될 수 있음" "sudo sysctl -w net.core.rmem_max=134217728 net.core.wmem_max=134217728"
command -v chronyc >/dev/null && log "  NUC 시각 동기: $(chronyc tracking 2>/dev/null | awk -F': ' '/System time/{print $2}')"

# ── 1) 내부 PC ssh — 키가 있으면 묻지 않음, 없으면 비밀번호 한 번(최대 2번 시도) ───────────────────
if [ "$ROBOTPC" = 1 ] && [ "${DRY:-0}" != 1 ]; then
  if "${SSH[@]}" -O check "$ROBOT" >/dev/null 2>&1; then log "내부 PC 연결 재사용"
  elif timeout 15 "${SSH[@]}" -o BatchMode=yes -o ControlMaster=auto -o ControlPersist=3h -fN "$ROBOT" 2>/dev/null; then log "내부 PC 연결(키)"
  else
    ok=0
    for k in 1 2; do
      log "내부 PC($ROBOT) 연결 — 비밀번호를 입력하세요 ($k/2)"
      timeout --foreground 90 "${SSH[@]}" -o ControlMaster=auto -o ControlPersist=3h -fN "$ROBOT" && { ok=1; break; }
    done
    [ "$ok" = 1 ] || { ROBOTPC=0; problem "내부 PC ssh 접속 실패 — 카메라 없이 녹화" "비밀번호 틀림/시간 초과, 또는 내부 PC sshd 문제" "ssh unitree@192.168.123.18 로 직접 접속해 보기 · 키를 등록하면 다음부터 안 물음: ssh-copy-id unitree@192.168.123.18"; }
    [ "$ok" = 1 ] && log "  (팁) 다음부터 비밀번호 안 묻게: ssh-copy-id unitree@192.168.123.18"
  fi
fi
rsh(){ { [ "${DRY:-0}" = 1 ] || [ "$ROBOTPC" = 0 ]; } && return 0; timeout 20 "${SSH[@]}" -o BatchMode=yes "$ROBOT" "$@"; }
jrun(){   # jrun <menu|direct> "<명령>" <로그> — menu: 로그인 메뉴 foxy(1) 과 같은 환경(비대화형) / direct: eth0 인라인 DDS
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
cam_usb(){ rsh 'lsusb | grep -c "8086:0b3a"' </dev/null 2>/dev/null | tail -1 | tr -dc 0-9; }
cam_local_hz(){   # 내부 PC 안에서 실제 데이터가 나오는지 — 3초짜리 C++ 시험 녹화로 센다(내부 PC 의 파이썬 구독은 같은 PC 데이터를 못 받는다 — 11-04)
  local lf="$LOGDIR/cam_hz_$1.log" d="/tmp/go2_camcheck_${TS}_$1_$RANDOM"; : > "$lf"
  jrun "$1" "timeout -s INT 4 ros2 bag record -o $d /camera/color/camera_info >/dev/null 2>&1; sleep 1; ros2 bag info $d 2>/dev/null | grep -o 'Count: [0-9]*' | head -1" "$lf"
  awk '/Count:/{c=$2}END{if (c+0 > 15) printf "%.1f", c/3.5}' "$lf"
}

# ── 2) 시각 ───────────────────────────────────────────────────────────────────
clock_offset(){   # 5번 재서 왕복이 가장 짧은 것
  { [ "${DRY:-0}" = 1 ] || [ "$ROBOTPC" = 0 ]; } && { echo "(측정 안 함)"; return; }
  local a b r best="" i
  for i in 1 2 3 4 5; do a=$(date +%s.%N); r=$(rsh 'date +%s.%N' </dev/null 2>/dev/null | tail -1); b=$(date +%s.%N)
    [[ "$r" =~ ^[0-9]+\.[0-9]+$ ]] || continue
    if [ -z "$best" ] || python3 -c "import sys; x='$best'.split(); sys.exit(0 if $b-$a < float(x[1])-float(x[0]) else 1)"; then best="$a $b $r"; fi
  done
  [ -n "$best" ] || { echo "측정실패"; return; }
  set -- $best
  echo "$1 $2 $3" >> "$LOGDIR/clock_fallback.txt"   # 시각 측정기가 죽었을 때 보정용 예비 표본
  python3 -c "a,b,r=$1,$2,$3; import time; print(f'{r-(a+b)/2:+.3f} s  (왕복 {1000*(b-a):.0f} ms, 내부PC {time.strftime(\"%Y-%m-%d %H:%M:%S\", time.localtime(r))})')"
}
OFF0=$(clock_offset); log "시각: 내부PC−NUC = $OFF0"
d=${OFF0%% s*}
if [ "${SET_CLOCK:-0}" = 1 ] && [ "$ROBOTPC" = 1 ] && [[ $d =~ ^[+-]?[0-9]+\.[0-9]+$ ]] && ! python3 -c "import sys; sys.exit(0 if abs($d) < 0.05 else 1)"; then
  if [ -n "$(cam_proc)" ]; then log "  카메라 노드가 돌고 있어 시계를 안 바꾼다(stamp 가 깨짐) — 측정·보정만"
  else log "내부 PC 시계를 NUC 에 맞춤 — sudo 비밀번호"
    run timeout --foreground 60 "${SSH[@]}" -t "$ROBOT" "sudo -v && sudo date -s @\$(python3 -c 'import time; print(\"%.6f\" % (time.time() - ($d)))') >/dev/null && echo set" \
      && { OFF0="$(clock_offset) (맞춘 뒤)"; log "시각: 내부PC−NUC = $OFF0"; } || log "  시계 맞춤 실패 — 측정·보정만(녹화엔 지장 없음)"
  fi
fi

# ── 3) 카메라 D435i — USB 기다림 → 켜기(일반 → 하드웨어 초기화 → 다른 DDS 설정) ───────────────────
CAM_ARGS="enable_color:=true enable_depth:=true enable_infra1:=true enable_infra2:=true enable_gyro:=false enable_accel:=false"
CAM_ARGS="$CAM_ARGS rgb_camera.profile:=$P depth_module.profile:=$P rgb_camera.color_profile:=$P depth_module.depth_profile:=$P depth_module.infra_profile:=$P"
cam_launch(){ echo "setsid nohup ros2 launch realsense2_camera rs_launch.py $CAM_ARGS $1 > /tmp/go2_rs_$TS.log 2>&1 < /dev/null &"; }
cam_kill(){ rsh 'pkill -INT -f "[r]s_launch.py"; pkill -INT -f "[r]ealsense2_camera_node"; sleep 3; pkill -KILL -f "[r]ealsense2_camera_node"' </dev/null >/dev/null 2>&1; }
MODE=""; RC=""
if [ "${CAM:-1}" = 0 ]; then log "카메라: 끔(CAM=0)"
elif [ "$ROBOTPC" = 0 ]; then log "카메라: 내부 PC 에 못 붙어 건너뜀"
elif [ "${DRY:-0}" = 1 ]; then log "(DRY) 카메라·내부 PC 녹화 건너뜀"; MODE=menu
else
  U=$(cam_usb); W1=$SECONDS
  while [ "${U:-0}" -lt 1 ] && [ $((SECONDS-W1)) -lt "${CAM_USB_WAIT:-60}" ]; do
    log "  ★ D435i 가 내부 PC USB 에 안 보임 — USB3(파란) 포트에 다시 꽂으세요 (기다리는 중 $((SECONDS-W1))/${CAM_USB_WAIT:-60} s)"; sleep 10; U=$(cam_usb); done
  if [ "${U:-0}" -lt 1 ]; then
    problem "D435i USB 인식 안 됨 — 카메라 없이 녹화" "카메라 USB 케이블 빠짐·접촉 불량·전원 부족" "$SSHP 'lsusb | grep 8086'  (나와야 함: 8086:0b3a Intel RealSense 435i) · 카메라 쪽·내부 PC 쪽 USB 다시 꽂기(USB3 포트) · $SSHP 'dmesg | tail -20'"
  else
    [ $((SECONDS-W1)) -gt 0 ] && fixed "D435i USB 다시 인식(기다린 $((SECONDS-W1)) s)"
    us=$(rsh 'lsusb -t | grep -m1 uvcvideo' </dev/null 2>/dev/null); log "  카메라 USB: ${us##*, }"
    [[ "$us" == *480M* ]] && problem "D435i 가 USB2(480M)로 붙음" "USB2 포트나 USB2 케이블 — 4스트림 30 fps 대역이 안 됨" "USB3(파란) 포트·USB3 케이블로 바꾸기 · 당장은 CAM_PROFILE=640x480x15 로 다시 실행"
    if [ -n "$(cam_proc)" ]; then
      for m in menu direct; do RC=$(cam_local_hz $m); [ -n "$RC" ] && { MODE=$m; break; }; done
      [ -n "$MODE" ] && log "카메라: 이미 켜져 있음($MODE, 컬러 $RC Hz) — 재사용" || { log "  돌고 있는 카메라 노드가 데이터를 안 냄 → 끄고 다시 켬"; cam_kill; }
    fi
    if [ -z "$MODE" ]; then
      for att in "menu|" "menu|initial_reset:=true" "direct|initial_reset:=true"; do
        m=${att%%|*}; extra=${att#*|}
        log "카메라 켜기 ($m ${extra:-일반})"; : > "$LOGDIR/cam_start.log"; jrun $m "$(cam_launch "$extra")" "$LOGDIR/cam_start.log"
        for i in $(seq 1 8); do sleep 3; RC=$(cam_local_hz $m); [ -n "$RC" ] && break; done
        if [ -n "$RC" ]; then MODE=$m; [ "$att" != "menu|" ] && fixed "카메라를 '$m ${extra}' 로 다시 켜서 살림"; break; fi
        log "  데이터 없음 — 다음 방법으로"; cam_kill
      done
    fi
    if [ -n "$MODE" ]; then log "  ✓ 카메라 컬러 $RC Hz (내부 PC 안, $MODE, 요청 $P)"
    else rsh "tail -30 /tmp/go2_rs_$TS.log" </dev/null > "$LOGDIR/robotpc_rs.log" 2>&1
         problem "카메라 노드가 데이터를 안 냄 — 카메라 없이 녹화" "펌웨어/USB 전원 불안정, 다른 프로그램이 카메라 점유" "$SSHP 'tail -30 /tmp/go2_rs_$TS.log' (로그 사본 $LOGDIR/robotpc_rs.log) · $SSHP 'pgrep -af realsense' 로 다른 카메라 프로그램 확인 · 카메라 USB 뽑았다 꽂고 다시 실행"; fi
  fi
fi

# ── 4) 내부 PC 카메라 녹화 (조각 단위 — 녹화 중 죽으면 감시자가 다음 조각으로 이어 감) ─────────────────
JREC=0; STO=sqlite3; SOPT=""
CAMTOPICS="/camera/color/image_raw /camera/color/camera_info /camera/color/metadata /camera/depth/image_rect_raw /camera/depth/camera_info /camera/depth/metadata"
CAMTOPICS="$CAMTOPICS /camera/infra1/image_rect_raw /camera/infra1/camera_info /camera/infra1/metadata /camera/infra2/image_rect_raw /camera/infra2/camera_info /camera/infra2/metadata"
CAMTOPICS="$CAMTOPICS /camera/extrinsics/depth_to_color /camera/extrinsics/depth_to_infra1 /camera/extrinsics/depth_to_infra2 /tf_static /tf"
jrec_alive(){ rsh "pgrep -f '[b]ag record -o .*cam_$TS'" </dev/null >/dev/null 2>&1; }
jrec_start(){   # jrec_start <조각번호> → 성공하면 0
  local n=$1 dir="${JPRE}_p$1"
  jrun $MODE "setsid nohup ros2 bag record -o ~/$dir $SOPT $CAMTOPICS > /tmp/go2_camrec_${TS}_p$n.log 2>&1 < /dev/null &" "$LOGDIR/jrec_start.log"
  sleep 5
  local s=$(rsh "du -sk ~/$dir 2>/dev/null | cut -f1" </dev/null 2>/dev/null | tail -1 | tr -dc 0-9)
  if jrec_alive && [ "${s:-0}" -gt 0 ]; then echo "$dir" >> "$JPARTS"; return 0; fi
  rsh "tail -20 /tmp/go2_camrec_${TS}_p$n.log" </dev/null > "$LOGDIR/jrec_err_p$n.log" 2>&1; return 1
}
if [ -n "$MODE" ] && [ "${DRY:-0}" != 1 ]; then
  JAV=$(rsh 'df -BG --output=avail ~ | tail -1' </dev/null 2>/dev/null | tr -dc 0-9)
  STO=$(rsh 'ls /opt/ros/foxy/share 2>/dev/null | grep -qx rosbag2_storage_mcap && echo mcap || echo sqlite3' </dev/null 2>/dev/null | tail -1 | tr -d '\r'); [ "$STO" = mcap ] && SOPT="-s mcap"
  log "  내부 PC 디스크 여유 ${JAV:-?} GB (4스트림 30 fps 약 85 MB/s → 약 $(( ${JAV:-0} * 1024 / 85 / 60 ))분) · 저장 ${STO}"
  [ "${JAV:-0}" -lt 20 ] && problem "내부 PC 디스크 여유 ${JAV}G" "예전 카메라 녹화가 쌓임" "$SSHP 'du -sh ~/bags/*' 로 확인 후 옮긴 것 지우기"
  rsh 'mkdir -p ~/bags' </dev/null >/dev/null 2>&1; : > "$LOGDIR/jrec_start.log"
  for k in 1 2 3; do
    if jrec_start $k; then JREC=1; log "  ✓ 내부 PC 카메라 녹화 시작 → ~/${JPRE}_p$k"; [ $k -gt 1 ] && fixed "내부 PC 녹화기를 $k 번째에 띄움"; break; fi
    log "  내부 PC 녹화기 안 뜸($k/3) — 다시"; rsh "pkill -INT -f '[b]ag record -o .*cam_$TS'" </dev/null >/dev/null 2>&1; sleep 2
  done
  [ "$JREC" = 1 ] || problem "내부 PC 카메라 녹화기 안 뜸 — 카메라 없이 녹화" "내부 PC 디스크·권한·ROS 환경" "$SSHP 'tail -20 /tmp/go2_camrec_${TS}_p1.log; df -h ~' (사본 $LOGDIR/jrec_err_p1.log)"
fi

# ── 5) XT16 드라이버 (NUC) — 18 Hz 미만이면 깨끗이 다시 켬, 최대 2번 ───────────────────────────────
xt_start(){ setsid nohup bash -c "source ~/gosu/env_drive.sh >/dev/null 2>&1; exec ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args -r __ns:=/hesai_ros_driver" > "$LOGDIR/xt16_$1.log" 2>&1 < /dev/null & }
xt_hz(){ [ "${DRY:-0}" = 1 ] && return; timeout 7 ros2 topic hz /lidar_points 2>/dev/null | awk '/average rate/{r=$3}END{print r}'; }
xt_ok(){ python3 -c "import sys; sys.exit(0 if float('${RX:-0}' or 0) >= 18 else 1)"; }
xt_restart(){ pkill -INT -x hesai_ros_drive; for i in $(seq 1 10); do pgrep -x hesai_ros_drive >/dev/null || break; sleep 1; done
              pgrep -x hesai_ros_drive >/dev/null && { pkill -KILL -x hesai_ros_drive; sleep 1; }; xt_start "$1"; sleep 5; }
nd=$(pgrep -c -x hesai_ros_drive)
if [ "${nd:-0}" -ge 1 ]; then log "XT16 드라이버: 이미 ${nd}개"; else log "XT16 드라이버 켜기"; [ "${DRY:-0}" = 1 ] || xt_start 1; sleep 4; fi
RX=$(xt_hz)
for k in 2 3; do
  xt_ok || [ "${DRY:-0}" = 1 ] && break
  log "  XT16 ${RX:-0} Hz — 드라이버를 모두 끄고 하나만 다시 켬($((k-1))/2)"; xt_restart $k; RX=$(xt_hz)
  xt_ok && fixed "XT16 드라이버 재시작으로 ${RX} Hz 회복"
done
log "  XT16 /lidar_points ${RX:-0} Hz"
[ "${DRY:-0}" = 1 ] || xt_ok || problem "XT16 ${RX:-0} Hz (18 Hz 미만) — 그대로 녹화" "라이다 전원·케이블, 드라이버 중복, 수신 버퍼" "ping 192.168.123.20 · pgrep -a hesai_ros_drive (1개여야) · sysctl net.core.rmem_max (≥26214400) · 로그 $LOGDIR/xt16_*.log"

# ── 6) 시각 측정 + 감시자 + NUC 녹화 ────────────────────────────────────────────
probe_start(){ ROBOT="$ROBOT" CM="$CM" setsid python3 "$TOOLS/go2_clock_probe.py" "$LOGDIR/clock_probe.csv" 2 20 >> "$LOGDIR/probe.log" 2>&1 < /dev/null & echo $! > "$LOGDIR/probe.pid"; }
if [ "$ROBOTPC" = 1 ] && [ "${DRY:-0}" != 1 ]; then
  probe_start; sleep 4
  if [ "$(grep -c . "$LOGDIR/clock_probe.csv" 2>/dev/null)" -lt 2 ]; then kill "$(cat "$LOGDIR/probe.pid")" 2>/dev/null; probe_start; sleep 4; fi
  [ "$(grep -c . "$LOGDIR/clock_probe.csv" 2>/dev/null)" -ge 2 ] && log "  시각 차이 측정 중: $(tail -1 "$LOGDIR/clock_probe.csv")" \
    || problem "시각 측정기 안 돎 — 시작·끝 측정값으로만 보정(수 ms)" "ssh 연결·내부 PC python3" "로그 $LOGDIR/probe.log · $SSHP 'python3 --version'"
fi
LIST=$([ "${DRY:-0}" = 1 ] && echo "" || timeout 10 ros2 topic list -t 2>/dev/null)
KNOWN="unitree_arm/msg/ArmString unitree_go/msg/ConfigChangeStatus unitree_go/msg/VoxelMapCompressed unitree_interfaces/msg/PcToImage unitree_interfaces/msg/QtCommand unitree_interfaces/msg/QtEdge unitree_interfaces/msg/QtNode"
MISSING=$(comm -23 <({ sed -E 's/.*\[(.*)\]/\1/' <<<"$LIST" | tr ',' '\n'; tr ' ' '\n' <<<"$KNOWN"; } | sed 's/ //g' | grep . | sort -u) \
                   <(ros2 interface list 2>/dev/null | sed 's/^ *//' | sort -u) | tr '\n' ' ')
XTYPES=(); [ -n "${MISSING// /}" ] && XTYPES=(--exclude-topic-types $MISSING)
for t in /lowstate /utlidar/cloud /utlidar/robot_odom /sportmodestate; do grep -q "^$t " <<<"$LIST" || { [ "${DRY:-0}" = 1 ] || problem "토픽 $t 안 보임" "로봇 본체 서비스 미기동 또는 DDS" "ros2 topic list | grep $t · 로봇 재부팅 뒤 1분 기다렸다 다시 실행"; }; done
EXCL='^/camera/.*'   # 카메라는 내부 PC 가 녹화 — NUC 가 구독하면 영상이 랜으로 넘어온다
log "NUC 녹화 시작 → $OUT   (카메라 제외 전 토픽, 정의 없어 뺀 형식 $(wc -w <<<"$MISSING")개)"
log "끝내기: Ctrl+C · 시작·끝에 로봇을 2초간 좌우로 비틀기(시각 확인용) · 기립 후 30초 안 출발"
[ "${DRY:-0}" = 1 ] && { log "(DRY) ros2 bag record --all-topics --exclude-regex '$EXCL' ... -o $OUT · 내부 PC ros2 bag record -o ~/${JPRE}_p1"; exit 0; }
trap ':' INT   # Ctrl+C 는 녹화기만 끝내고 마무리는 끝까지
watch(){   # 15초마다 점검·자동 복구 (서브셸 — 상태는 파일로)
  local n=1 ev="$LOGDIR/watch.log"
  while sleep 15; do
    [ -f "$LOGDIR/stop" ] && break
    a=$(df --output=avail -BG "$BAGROOT" | tail -1 | tr -dc 0-9); [ "${a:-999}" -lt 10 ] && { echo "[$(date +%T)] ★ NUC 디스크 ${a}G — 녹화 정상 종료" | tee -a "$ev"; pkill -INT -f "[b]ag record.*$BAGNAME"; break; }
    if [ "$JREC" = 1 ]; then
      if [ -z "$(cam_proc)" ]; then echo "[$(date +%T)] ↺ 카메라 노드 죽음 → 다시 켬" | tee -a "$ev"; jrun $MODE "$(cam_launch initial_reset:=true)" "$LOGDIR/cam_restart.log"; fi
      if ! jrec_alive; then n=$((n+1)); echo "[$(date +%T)] ↺ 내부 PC 녹화기 죽음 → 조각 p$n 으로 이어 녹화" | tee -a "$ev"; jrec_start $n || echo "[$(date +%T)] ★ 조각 p$n 시작 실패" | tee -a "$ev"; fi
      j=$(rsh 'df -BG --output=avail ~ | tail -1' </dev/null 2>/dev/null | tr -dc 0-9); [ -n "$j" ] && [ "$j" -lt 5 ] && { echo "[$(date +%T)] ★ 내부 PC 디스크 ${j}G — 카메라 녹화 종료" | tee -a "$ev"; rsh "pkill -INT -f '[b]ag record -o .*cam_$TS'" </dev/null >/dev/null 2>&1; JREC=0; }
    fi
    [ "$(pgrep -c -x hesai_ros_drive)" = 0 ] && { echo "[$(date +%T)] ↺ XT16 드라이버 죽음 → 다시 켬" | tee -a "$ev"; xt_start w$(date +%H%M%S); }
    [ -f "$LOGDIR/probe.pid" ] && ! kill -0 "$(cat "$LOGDIR/probe.pid")" 2>/dev/null && { echo "[$(date +%T)] ↺ 시각 측정기 죽음 → 다시 켬" | tee -a "$ev"; probe_start; }
  done
}
watch & WPID=$!
ros2 bag record -s mcap --storage-preset-profile zstd_fast --all-topics \
  --exclude-regex "$EXCL" "${XTYPES[@]}" -d 300 \
  --custom-data "note=${MEMO//=/:}" "name=${NAME//=/:}" "host=$(hostname)" "camera=robotpc:${JPRE}_pN" \
  -o "$OUT"
RCREC=$?
touch "$LOGDIR/stop"; kill $WPID 2>/dev/null
[ -s "$LOGDIR/watch.log" ] && while IFS= read -r l; do printf '□ (녹화 중) %s\n\n' "$l" >> "$PROB"; done < "$LOGDIR/watch.log"
[ "$RCREC" = 0 ] || problem "NUC 녹화기 종료 코드 $RCREC" "디스크·DDS 오류" "로그 위쪽 확인 · ros2 bag info $OUT"

# ── 7) 마무리: 내부 PC 녹화 끝 → 측정 끝 → 옮기기(재시도) → 시각 보정 ─────────────────────────
log "NUC 녹화 끝. 내부 PC 카메라 녹화 마무리 중…"
if [ -s "$JPARTS" ]; then
  rsh "pkill -INT -f '[b]ag record -o .*cam_$TS'" </dev/null >/dev/null 2>&1
  for i in $(seq 1 60); do jrec_alive || break; sleep 1; done
fi
OFF1=$(clock_offset)
[ -f "$LOGDIR/probe.pid" ] && { kill -INT "$(cat "$LOGDIR/probe.pid")" 2>/dev/null; sleep 1; }
mkdir -p "$OUT"; cp "$LOGDIR/clock_probe.csv" "$OUT/" 2>/dev/null
if [ "$(grep -c . "$OUT/clock_probe.csv" 2>/dev/null)" -lt 4 ] && [ -s "$LOGDIR/clock_fallback.txt" ]; then   # 측정기가 거의 못 쟀으면 시작·끝 값으로
  [ -s "$OUT/clock_probe.csv" ] || echo "nuc_time,offset_s,rtt_s,samples_ok" > "$OUT/clock_probe.csv"
  awk '{printf "%.6f,%+.6f,%.6f,1\n", ($1+$2)/2, $3-($1+$2)/2, $2-$1}' "$LOGDIR/clock_fallback.txt" >> "$OUT/clock_probe.csv"
  problem "시각 측정 표본이 적어 시작·끝 값으로 보정(정밀도 수 ms)" "시각 측정기 중단" "로그 $LOGDIR/probe.log"
fi
NP=0
while IFS= read -r dir <&3; do   # fd 3 — 안에서 ssh/rsync 가 표준입력을 먹어도 목록이 안 빠지게
  [ -n "$dir" ] || continue; NP=$((NP+1)); pn=${dir##*_}
  rsh "ls ~/$dir/metadata.yaml" </dev/null >/dev/null 2>&1 || problem "내부 PC 조각 $pn metadata.yaml 없음" "녹화기가 비정상 종료" "옮긴 뒤 NUC 에서: ros2 bag reindex $OUT/camera_robotpc_$pn sqlite3"
  if [ "${TRANSFER:-0}" != 1 ]; then   # ★ 11-05: 끝날 때 안 옮김 — 나중에 go2_record_fetch.sh 로 한 번에
    printf '%s\t%s\t%s\n' "$OUT" "$dir" "$pn" >> "$TOOLS/pending_camera.tsv"
    log "카메라 조각 $pn 은 내부 PC 에 둠(~/$dir) — 나중에: go2_record_fetch.sh all"; continue
  fi
  SZ=$(rsh "du -sh ~/$dir | cut -f1" </dev/null 2>/dev/null | tail -1); log "카메라 조각 $pn (${SZ:-?}) → NUC 로 옮김"
  ok=0
  for k in 1 2 3; do
    rsync -a --partial --info=progress2 -e "ssh -o ControlPath=$CM -o BatchMode=yes" "$ROBOT:$dir/" "$OUT/camera_robotpc_$pn/" 2>&1 | tail -1
    [ -f "$OUT/camera_robotpc_$pn/metadata.yaml" ] || [ -n "$(ls "$OUT/camera_robotpc_$pn" 2>/dev/null)" ] && { ok=1; break; }
    log "  옮기기 실패($k/3) — 다시"; sleep 3
  done
  [ "$ok" = 1 ] || { problem "카메라 조각 $pn 옮기기 실패" "ssh 연결 끊김·NUC 디스크" "rsync -a --partial -e 'ssh' $ROBOT:$dir/ $OUT/camera_robotpc_$pn/   (비밀번호 물으면 입력)"; continue; }
  [ $k -gt 1 ] && fixed "카메라 조각 $pn 을 $k 번째에 옮김"
  if [ -f "$OUT/camera_robotpc_$pn/metadata.yaml" ]; then
    log "  시각 보정 중…"
    python3 "$TOOLS/go2_camera_bag_fix.py" "$OUT/camera_robotpc_$pn" "$OUT/clock_probe.csv" "$OUT/camera_nuctime_$pn" 2>&1 | grep -v '^\[INFO\]' | tail -1 | tee -a "$LOGDIR/auto.log" \
      || true
    [ -f "$OUT/camera_nuctime_$pn/time_fix.json" ] || problem "카메라 조각 $pn 시각 보정 실패" "bag 손상·형식" "python3 $TOOLS/go2_camera_bag_fix.py $OUT/camera_robotpc_$pn $OUT/clock_probe.csv $OUT/camera_nuctime_$pn"
  fi
done 3< "$JPARTS"

# ── 8) 기록·점검·요약 ───────────────────────────────────────────────────────────
{
  echo "이름     $NAME"; echo "메모     $MEMO"
  echo "녹화     $(date '+%F %T') 종료, 호스트 $(hostname), 도메인 ${ROS_DOMAIN_ID:-0}, NUC 녹화기 종료 코드 $RCREC"
  echo "카메라   요청 $P, 시작 때 컬러 ${RC:-?} Hz, 환경 ${MODE:-없음}, 저장 ${STO}, 조각 $NP 개 → camera_robotpc_pN/(원본) · camera_nuctime_pN/(NUC 시계)"
  echo; echo "── 내부 PC 시계 − NUC 시계"; echo "시작 $OFF0"; echo "끝   $OFF1"
  for f in "$OUT"/camera_nuctime_*/time_fix.json; do [ -f "$f" ] && { echo "보정 $f:"; cat "$f"; echo; }; done
  echo; echo "── XT16 ${RX:-?} Hz · NUC chrony"; chronyc tracking 2>/dev/null | grep -E 'Reference|System time|Last offset'
  echo; echo "── 커널 소켓 버퍼"; sysctl net.core.rmem_max net.core.wmem_max 2>/dev/null
  echo; echo "── 정의가 없어 제외한 형식"; echo "${MISSING:-없음}"
  echo; echo "── 녹화 시작 때 NUC 토픽 목록"; echo "$LIST"
  echo; echo "── NUC bag 정보"; ros2 bag info "$OUT" 2>/dev/null
  for c in "$OUT"/camera_nuctime_*; do [ -d "$c" ] && { echo; echo "── 카메라 $c"; ros2 bag info "$c" 2>/dev/null; }; done
} > "$OUT/meta.txt"
INFO=$(ros2 bag info "$OUT" 2>/dev/null)
for t in /lidar_points /lowstate /utlidar/cloud /utlidar/robot_odom /sportmodestate; do grep -q " $t " <<<"$INFO" || problem "$t 가 녹화에 없음" "드라이버·로봇 서비스·DDS" "ros2 topic hz $t 로 지금 나오는지 확인 · 안 나오면 로봇 재부팅 후 다시"; done
DUR=$(awk '/Duration:/{d=$2; sub(/s$/,"",d); print d}' <<<"$INFO")
LP=$(grep -o "/lidar_points | Type: [^|]*| Count: [0-9]*" <<<"$INFO" | grep -o '[0-9]*$')
[ -n "$DUR" ] && [ -n "$LP" ] && python3 -c "import sys; sys.exit(0 if $LP/max($DUR,1) >= 18 else 1)" || { [ -n "$LP" ] && problem "녹화된 XT16 $(python3 -c "print(round($LP/max(${DUR:-1},1),1))") Hz (20 Hz 기대)" "UDP 유실(rmem)·드라이버 중복·디스크 쓰기 지연" "pgrep -a hesai_ros_drive · sysctl net.core.rmem_max"; }
if [ "${CAM:-1}" != 0 ]; then
  CC=0; CD=0   # 카메라 장수·시간은 카메라 bag 자체 기준(카메라는 NUC 녹화보다 먼저 시작·늦게 끝난다 — NUC 시간으로 나누면 40 fps 로 잘못 나왔다)
  for c in "$OUT"/camera_nuctime_*; do [ -d "$c" ] || continue; ci=$(ros2 bag info "$c" 2>/dev/null)
    CC=$((CC + $(grep -o "/camera/color/image_raw | Type: [^|]*| Count: [0-9]*" <<<"$ci" | grep -o '[0-9]*$' | head -1)))
    CD=$(python3 -c "print($CD + float('$(awk '/Duration:/{d=$2; sub(/s$/,"",d); print d}' <<<"$ci")' or 0))"); done
  if [ "$CC" = 0 ]; then   # ★ 11-05: 영상을 안 옮겼으면 내부 PC 에 있는 조각의 metadata.yaml 로 센다
    while IFS= read -r dir <&3; do [ -n "$dir" ] || continue
      read -r c d < <(rsh "awk '/^  duration:/{getline; d=\$2} /name: \/camera\/color\/image_raw\$/{f=1} f&&/message_count:/{c=\$2; f=0} END{print c+0, d+0}' ~/$dir/metadata.yaml" </dev/null 2>/dev/null | tail -1)
      CC=$((CC + ${c:-0})); CD=$(python3 -c "print($CD + ${d:-0}/1e9)")
    done 3< "$JPARTS"
  fi
  [ "$CC" -gt 0 ] && { FPS=$(python3 -c "print(round($CC/max($CD,1),1))"); log "  카메라 컬러 ${CC}장 / ${CD%.*} s ≈ ${FPS} fps"
    python3 -c "import sys; sys.exit(0 if $FPS >= 27 else 1)" || problem "카메라 ${FPS} fps (30 기대)" "내부 PC 디스크 쓰기·CPU, USB 대역" "$SSHP 'tail -20 /tmp/go2_camrec_${TS}_p1.log' · 해상도 낮추기 CAM_PROFILE=640x480x30"; } \
    || problem "카메라 영상이 녹화에 없음" "위 카메라 문제 참고" "$SSHP 'lsusb | grep 8086; pgrep -af realsense'"
fi
cp "$LOGDIR"/*.log "$LOGDIR"/*.txt "$OUT/" 2>/dev/null
echo
log "요약: $(grep -E 'Duration|Bag size' <<<"$INFO" | tr -s ' ' | tr '\n' ' ')"
if grep -q '^■' "$PROB"; then
  echo "────────────────────────── 문제와 해결 명령 ($(grep -c '^■' "$PROB")건) ──────────────────────────"; grep -A2 '^■' "$PROB"
else log "✓ 해결 못 한 문제 없음"; fi
grep -q '^□' "$PROB" && { echo "── 자동 복구한 것"; grep '^□' "$PROB"; }
cp "$PROB" "$OUT/problems.txt"
log "끝. 저장: $OUT   (내부 PC 원본 ~/${JPRE}_p* 은 확인 후 지울 것)"
log "드라이버는 켜 둔 상태. 끄려면 — 카메라: $SSHP 'pkill -INT -f realsense2_camera_node'   XT16: pkill -INT -x hesai_ros_drive"
