#!/usr/bin/env bash
# 단위 테스트를 다른 노드와 발견(discovery) 안 되게(시뮬과 같은 도메인 80 이어도 섞이지 않음) 돌림. 사용: run_tests.sh orig|patched
H=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/jazzy/setup.bash; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1; source ~/ws/merge_tmp_20261102/review/install/setup.bash >/dev/null 2>&1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=80 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>none</ParticipantIndex><Peers></Peers></Discovery></Domain></CycloneDDS>'
case "$1" in orig) SRC=$HOME/ws/junsu_v2/src;; *) SRC=$H/mirror/src;; esac
cd $H && PYTHONPATH=$SRC:$PYTHONPATH timeout 300 python3 test_slalom_extra.py 2>&1 | grep -v '^\[WARN\]\|^\[INFO\]\|^\[ERROR\]'
if [ "$1" != orig ]; then mkdir -p $H/tmp_orig_test && cp $HOME/ws/junsu_v2/src/test_real_course_runner.py $H/tmp_orig_test/ && cd $H/tmp_orig_test && PYTHONPATH=$SRC:$PYTHONPATH timeout 300 python3 test_real_course_runner.py 2>&1 | grep -v '^\[WARN\]\|^\[INFO\]\|^\[ERROR\]' | tail -3; fi
