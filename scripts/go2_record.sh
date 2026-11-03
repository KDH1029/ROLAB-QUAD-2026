#!/usr/bin/env bash
# Go2 전체 녹화 — "나중에 필요할지 모르는 것까지" 남긴다. (팀 v_gosu/tools/record_all.sh 를 이 저장소용으로 포크:
# 환경 설정은 go2_run.sh 와 같게, 미션 토픽(/mission/*, /route/*, /amcl_pose)을 점검 목록에 추가, 저장 기본 ~/bags)
#
# 이 녹화로 나중에(노트북에서, 별도 도메인 + 루프백 재생) 매핑·SLAM·AMCL·콘 위치·YOLO 학습을 디버깅한다:
#   - 콘을 0.85~2.2 m 거리로 여러 각도에서 지나가기 (카메라와 라이다가 둘 다 보는 거리)
#   - 정지선에서 신호등이 두 번 이상 바뀔 때까지 (10 s 이상) 서 있기
#   - 시작·끝은 같은 자리에서 10초 정지
#   전 토픽(--all-topics)을 녹화하고, 원본에서 다시 만들 수 있는 무거운 파생 토픽만 뺀다.
#   로봇이 움직일 수 있으므로 사용자 터미널에서 실행한다. Ctrl+C 로 끝내면 정리·점검까지 한다.
#
# 사용:  record_all.sh <이름> "<메모: 장소·주행 내용·특이사항>"
#   예:  record_all.sh stairs_1030 "누리관 X자 계단 3회 왕복, D435i 머리 위 장착"
#
# 카메라까지 남기려면 별도 터미널에서 먼저(IMU 는 끈다 — 켜면 USB 가 끊김):
#   ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true \
#     enable_infra1:=true enable_infra2:=true enable_gyro:=false enable_accel:=false \
#     depth_module.emitter_on_off:=true \
#     rgb_camera.color_profile:=848x480x30 depth_module.depth_profile:=848x480x30 depth_module.infra_profile:=848x480x30
#   (emitter_on_off: 투광 켠 프레임은 깊이용, 끈 프레임은 시각 오도메트리용으로 번갈아 남긴다)

NAME="${1:?사용: record_all.sh <이름> \"<메모>\"}"
NOTE="${2:-메모 없음}"
ROOT="${BAGROOT:-$HOME/bags}"           # BAGROOT 로 저장 폴더 지정 가능
OUT="$ROOT/${BAGNAME:-$(date +%m%d_%H%M)_$NAME}"   # BAGNAME 으로 이름 통째 지정 가능 (예: gosu_YYYYMMDD_HHMMSS)

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
CYCLONEDDS_WS="${CYCLONEDDS_WS:-$HOME/unitree_ros2/cyclonedds_ws/install}"
[ -f "$CYCLONEDDS_WS/setup.bash" ] && source "$CYCLONEDDS_WS/setup.bash"
[ -f "$WS_DIR/install/setup.bash" ] && source "$WS_DIR/install/setup.bash"   # unitree / livox 메시지 정의
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
iface=$(ip -o -4 addr show | awk '$4 ~ /^192\.168\.123\./ {print $2; exit}')
[ -z "$iface" ] && { echo "★ 192.168.123.x 주소가 없다. 로봇이 연결됐나?"; exit 1; }
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>${iface}</NetworkInterfaceAddress></General></Domain></CycloneDDS>"
case "${ROS_DOMAIN_ID:-0}" in 77|78) echo "★ 도메인 $ROS_DOMAIN_ID 은 재생용이다. 라이브 녹화는 도메인 0 에서."; exit 1;; esac

FREE=$(df --output=avail -BG ~ | tail -1 | tr -dc 0-9)
echo "디스크 여유 ${FREE}G  (카메라 포함 10분 약 35~50G, 라이다만 10분 약 6G)"
[ "$FREE" -lt 100 ] && { echo "★ 100G 미만이다. 옛 bag 을 외장으로 옮기고 다시."; exit 1; }

# 다시 만들 수 있는 무거운 파생 토픽만 뺀다(원본 센서·명령·상태는 전부 남긴다)
EXCL='^/(cloud_registered.*|Laser_map|path|velodyne_points|.*costmap.*|.*/compressedDepth.*|.*/theora.*|camera/.*/compressed|utlidar/voxel_map)$'   # 카메라 압축본은 원본에서 재생성 가능 + 구독하면 로봇 PC 가 압축하느라 링크·CPU 를 더 쓴다

# 녹화 전 점검 — 있어야 할 것과 있으면 좋은 것
MUST=(/lowstate /sportmodestate /utlidar/robot_odom /utlidar/cloud_base /utlidar/cloud_deskewed /utlidar/imu /lidar_points /wirelesscontroller /tf_static)
WANT=(/cmd_vel /api/sport/request /camera/color/image_raw /camera/depth/image_rect_raw /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw /camera/color/camera_info /camera/depth/camera_info /camera/extrinsics/depth_to_color /frontvideostream /Odometry \
      /amcl_pose /mission/traffic_light /mission/cone_detections_json /mission/cones /route/status)   # D435i 는 로봇 내부 PC 에 꽂혀 /camera/... 로 나온다
LIST=$(timeout 8 ros2 topic list 2>/dev/null)
echo "── 필수"; miss=0
for t in "${MUST[@]}"; do grep -qx "$t" <<<"$LIST" && echo "  ✓ $t" || { echo "  ✗ $t"; miss=1; }; done
echo "── 있으면 좋음"
for t in "${WANT[@]}"; do grep -qx "$t" <<<"$LIST" && echo "  ✓ $t" || echo "  · $t (없음)"; done
[ $miss = 1 ] && { read -r -p "필수 토픽이 빠졌다. 그래도 녹화할까? [y/N] " a; [ "$a" = y ] || exit 1; }

# 라이다 드라이버 중복 — 10-29 녹화는 드라이버가 두 개 돌아 XT16 이 9.4 Hz(한 바퀴 100 ms)로 들어간 것으로 보인다
nd=$(pgrep -c -x hesai_ros_drive)   # 프로세스 이름은 15자로 잘린다
np=$(timeout 8 ros2 topic info /lidar_points 2>/dev/null | awk '/Publisher count/{print $3}')
echo "── 라이다 드라이버: 이 NUC ${nd:-0}개, /lidar_points 발행자 ${np:-0}개 (다른 컴퓨터에서 도는 드라이버는 여기 안 잡힌다)"
if [ "${nd:-0}" -gt 1 ] || [ "${np:-0}" -gt 1 ]; then read -r -p "★ 드라이버가 하나가 아니다. 그래도 녹화할까? [y/N] " a; [ "$a" = y ] || exit 1; fi

# 이 NUC 에 메시지 정의가 없는 형식은 녹화기가 토픽 찾기를 멈추게 한다(10-29 확인) → 자동으로 뺀다
MISSING=$(comm -23 <(timeout 10 ros2 topic list -t 2>/dev/null | sed -E 's/.*\[(.*)\]/\1/' | tr ',' '\n' | sed 's/ //g' | sort -u) \
                   <(ros2 interface list 2>/dev/null | sed 's/^ *//' | sort -u) | tr '\n' ' ')
XTYPES=(); [ -n "${MISSING// /}" ] && { echo "── 정의 없는 형식(제외): $MISSING"; XTYPES=(--exclude-topic-types $MISSING); }
mkdir -p "$ROOT"
echo "저장: $OUT"
echo "주행 요령: 시작·끝은 같은 자리에서 10초 정지. 같은 구간 여러 번 지나기. 명령은 /cmd_vel 로 주면 같이 남는다."
trap ':' INT       # Ctrl+C 는 녹화만 끝내고 아래 정리는 계속한다
ros2 bag record -s mcap --storage-preset-profile zstd_fast --all-topics \
  --exclude-regex "$EXCL" "${XTYPES[@]}" -d 300 \
  --custom-data "note=$NOTE" "host=$(hostname)" "domain=${ROS_DOMAIN_ID:-0}" \
  -o "$OUT"
trap - INT

# 정리 — 나중에 이 bag 을 쓸 사람이 조건을 알 수 있게 남긴다
META="$OUT/meta.txt"
{
  echo "이름     $NAME"
  echo "메모     $NOTE"
  echo "녹화     $(date '+%F %T') 종료, 호스트 $(hostname), 도메인 ${ROS_DOMAIN_ID:-0}"
  echo "커널     $(uname -r)"
  echo; echo "── 시각 동기 (chrony)"; chronyc tracking 2>/dev/null | grep -E 'Reference|System time|Last offset' || echo "chrony 정보 없음"
  echo; echo "── 커널 소켓 버퍼"; sysctl net.core.rmem_max net.core.wmem_max 2>/dev/null
  echo; echo "── 설정 파일 md5 (이 bag 을 만든 당시 설정)"
  md5sum "$WS_DIR"/src/*/*/config/*.yaml "$WS_DIR"/config/*.yaml "$WS_DIR"/config/fastlio/*.yaml 2>/dev/null
  echo "git $(git -C "$WS_DIR" rev-parse --short HEAD 2>/dev/null) $(git -C "$WS_DIR" status --short 2>/dev/null | wc -l) 개 변경 파일"
  echo; echo "── 정의가 없어 제외한 형식"; echo "${MISSING:-없음}"
  echo; echo "── 녹화 시점 토픽 목록"; echo "$LIST"
  echo; echo "── bag 정보"; ros2 bag info "$OUT" 2>/dev/null
} > "$META"
echo; echo "끝. 요약:"
ros2 bag info "$OUT" 2>/dev/null | grep -E 'Duration|Bag size|Messages'
for t in "${MUST[@]}" "${WANT[@]}"; do grep -q " $t " <(ros2 bag info "$OUT" 2>/dev/null) || echo "  · 기록 안 됨: $t"; done
# 빈도 점검 — 10-29 녹화에서 드라이버는 20 Hz 인데 bag 에는 /lidar_points 9.4 Hz 만 들어갔다
ros2 bag info "$OUT" 2>/dev/null | awk '
  BEGIN { ref["/lidar_points"]=20; ref["/utlidar/cloud"]=15; ref["/lowstate"]=500; ref["/camera/depth/image_rect_raw"]=15; ref["/camera/color/image_raw"]=15 }   # 카메라는 링크 예산상 15 fps 권장
  /Duration:/ { d=$2; sub(/s$/, "", d) }
  match($0, /Topic: [^ ]+ /) { t=substr($0, RSTART+7, RLENGTH-8)
    if ((t in ref) && d > 0 && match($0, /Count: [0-9]+/)) { hz=substr($0, RSTART+7, RLENGTH-7)/d
      printf "  %s %s %.1f Hz (정상 %d Hz)\n", (hz < 0.9*ref[t] ? "★" : "✓"), t, hz, ref[t] } }'
echo "조건 기록: $META"
