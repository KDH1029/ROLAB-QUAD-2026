# 시뮬 격리: 루프백 전용 · 도메인 93 (로봇과 안 섞임)
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID=93 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer address="127.0.0.1"/></Peers><ParticipantIndex>auto</ParticipantIndex></Discovery></Domain></CycloneDDS>'
