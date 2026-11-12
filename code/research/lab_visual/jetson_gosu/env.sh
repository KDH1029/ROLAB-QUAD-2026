# 젯슨 위의 격리 작업공간 — 팀원 스택과 절대 섞이지 않는다.
#
#   source ~/gosu/lab_visual/env.sh
#
# 두 겹으로 막는다:
#   ROS_DOMAIN_ID=77   팀원 스택(도메인 0)과 다른 도메인. 도메인이 다르면 서로 안 보인다.
#   CYCLONEDDS_URI     DDS 를 루프백(lo)에만 묶고 멀티캐스트를 끈다.
#                      -> 이 젯슨 밖으로 한 바이트도 안 나간다. 로봇 내부PC 도 못 본다.
#
# 이 둘 중 하나만 있어도 격리되지만, 둘 다 건다. 실수 한 번으로 로봇이 움직이면 안 되기 때문이다.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=77
# ROS_LOCALHOST_ONLY 는 쓰지 않는다. rmw_cyclonedds 가 lo 를 스스로 넣는데
# 아래 URI 도 lo 를 넣어서 "the same interface may not be selected twice" 로 노드가 안 뜬다(실측).
# URI 만으로도 lo 전용 + 멀티캐스트 끔이 걸리므로 기능은 같다.
unset ROS_LOCALHOST_ONLY
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'

source /opt/ros/humble/setup.bash
# 팀원 메시지 정의(unitree_go 등)만 빌려 온다. 노드는 이 작업공간 것만 쓴다.
[ -f ~/go2_ws/install/setup.bash ] && source ~/go2_ws/install/setup.bash

export GOSU_LAB=~/gosu/lab_visual
echo "  [gosu] 도메인 $ROS_DOMAIN_ID, lo 전용, 멀티캐스트 끔 — 팀원 스택(도메인 0)과 격리됨"
