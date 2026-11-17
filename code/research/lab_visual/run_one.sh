#!/usr/bin/env bash
export MAP=go2_prior_map_v05.pcd POSE="${POSE:-x=0 y=0 yaw=0}" POSE_AT=14
HZ=${HZ:-5} PTS=${PTS:-800} WIDE=${WIDE:-0} TAG=${TAG:-one} ~/ws/lab_visual/verify_localization.sh ${BAG:-data1} ${SEC:-130}
