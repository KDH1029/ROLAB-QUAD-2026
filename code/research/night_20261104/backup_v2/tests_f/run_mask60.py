"""[F] 비교용: 가림 최대를 60초(예전 값, 노드 --align-mask-max 60)로 두고 C 녹화본 시험의 큰 씨앗 경우만 다시 — 결과 result_frame_align_F_mask60.json"""
import sys, functools
sys.path.insert(0, '/home/ilovehan9283/ws/night_20261104/backup_v2')
import frame_align
_orig = frame_align.FrameAligner.__init__
frame_align.FrameAligner.__init__ = functools.partialmethod(_orig, mask_max_s=60.0)
import test_frame_align as T
T.FrameAligner = frame_align.FrameAligner
sys.argv = ['x', '--tag', '_F_mask60', '--only', 'y3_t03,y5_t03,seed_y45_t0.3,seed_y3_t1.5,seed_y3_t2.0,seed_y3_t3.0,seed_y3_t3.0_floorstart,seed_y3_t4.0_floorstart']
T.main()
