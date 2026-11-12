import subprocess, time, os, resource
pats = ['hesai_ros_driver_node', 'cloud_bridge_node|imu_bridge_node', 'fastlio_mapping', 'wall_filter_node', 'obstacle_fusion_node', 'ego_planner_node',
        'junsu_vision/jetson_vision.sh start', 'cmd_vel_to_sport_node', 'real_course_runner.py', 'junsu_loc_node.py|junsu/amcl|junsu/map_server|cone_tracker_node|__ns:=/junsu', 'footprint_guard_node.py']
t0 = time.monotonic(); r0 = resource.getrusage(resource.RUSAGE_CHILDREN); s0 = resource.getrusage(resource.RUSAGE_SELF); n = 0
while time.monotonic() - t0 < 3.0:
    for p in pats:
        subprocess.run(['pgrep', '-f', p], stdout=subprocess.DEVNULL)
    n += 1
dt = time.monotonic() - t0; r1 = resource.getrusage(resource.RUSAGE_CHILDREN); s1 = resource.getrusage(resource.RUSAGE_SELF)
cpu = (r1.ru_utime + r1.ru_stime - r0.ru_utime - r0.ru_stime + s1.ru_utime + s1.ru_stime - s0.ru_utime - s0.ru_stime)
print(f'바쁜 루프 흉내: {n / dt:.1f} 루프/초, pgrep {11 * n / dt:.0f}회/초, CPU {cpu / dt * 100:.0f}% (한 코어=100%)')
