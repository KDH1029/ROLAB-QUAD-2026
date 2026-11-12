# gosu 주행 환경 — 로봇과 말해야 하므로 격리하지 않는다.
#
#   source ~/gosu/env_drive.sh
#
# 【env.sh 와 무엇이 다른가】
#   env.sh        도메인 77 + lo 전용. 재생·분석용. **로봇에 한 바이트도 안 나간다**
#   env_drive.sh  도메인 0 + eno1. 주행용. 로봇과 말한다
#   같이 쓰면 안 된다. 하나를 source 한 셸에서 다른 것을 source 하지 마라.
#
# 【무엇을 gosu 것으로 쓰나】
#   gosu 것:   ~/gosu/go2_ws/install   (팀원 코드를 복사해 여기서 빌드한 것)
#              ~/gosu/lab_visual       (내 위치추정·감시·도구)
#   빌려 쓰는 것: unitree_ros2 (DDS·메시지), hesai_ws (라이다 드라이버)
#              둘 다 벤더 드라이버라 바뀔 일이 없다. 복사해 봐야 빌드 부작용만 크다.
#
#   ~/go2_ws/install 은 **쓰지 않는다.** 팀원이 리빌드해도 여기 동작은 안 바뀐다.
set +u
source /opt/ros/humble/setup.bash
[ -f "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash" ] && source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
[ -f "$HOME/ws/hesai_ws/install/setup.bash" ]                && source "$HOME/ws/hesai_ws/install/setup.bash"
if [ -f "$HOME/gosu/go2_ws/install/setup.bash" ]; then
    source "$HOME/gosu/go2_ws/install/setup.bash"
    GOSU_BUILT=1
else
    GOSU_BUILT=0
fi
set -u

# 로봇은 eno1(192.168.123.99 -> .18) 위 도메인 0 으로만 말한다.
unset ROS_DOMAIN_ID ROS_LOCALHOST_ONLY 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -f "$HOME/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml" ] \
   && [ "$(cat /sys/class/net/eno1/carrier 2>/dev/null)" = "1" ]; then
    export CYCLONEDDS_URI="file://$HOME/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml"
else
    unset CYCLONEDDS_URI 2>/dev/null || true
    echo "  ★ eno1 링크가 없다 — 로봇에 못 붙는다"
fi
export ROS_LOG_DIR="$HOME/gosu/go2_ws/log"
mkdir -p "$ROS_LOG_DIR"
export GOSU=~/gosu
export GOSU_LAB=~/gosu/lab_visual

if [ "$GOSU_BUILT" = "1" ]; then
    echo "  [gosu-drive] 도메인 0, eno1, gosu 빌드 사용 — 로봇과 말한다"
else
    echo "  ★ [gosu-drive] ~/gosu/go2_ws/install 이 없다. 먼저 빌드할 것:"
    echo "     ~/gosu/build.sh"
fi
