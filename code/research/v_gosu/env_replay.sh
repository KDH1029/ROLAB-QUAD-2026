# v_gosu 재생 전용 환경 — 도메인 78, lo 전용, 멀티캐스트 끔 (밖으로 안 나간다)
# 기존 lab(77)과 분리. 팀 빌드(~/ws/lab/install)는 읽기 전용 밑바탕으로만 쓴다.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=${V_DOMAIN:-78}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
set +u
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash" ] && source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
[ -f "$HOME/ws/livox/ws_livox/install/setup.bash" ] && source "$HOME/ws/livox/ws_livox/install/setup.bash"
[ -f "$HOME/ws/lab/install/setup.bash" ] && source "$HOME/ws/lab/install/setup.bash"
[ -f "$HOME/ws/v_gosu/ws_livo/install/setup.bash" ] && source "$HOME/ws/v_gosu/ws_livo/install/setup.bash"
