"""[G] 비교용: 출발 오차 상한을 끈(max_offset_m=0) G 로 C 녹화본 시험 전부 — 큰 지도변환(씨앗 오차 시험)이 F 와 같게 고정되는지. 결과 result_frame_align_G_nooffset.json"""
import sys, functools
sys.path.insert(0, '/home/ilovehan9283/ws/night_20261104/backup_v2')
import frame_align
_orig = frame_align.FrameAligner.__init__
frame_align.FrameAligner.__init__ = functools.partialmethod(_orig, max_offset_m=0.0)
import test_frame_align as T
T.FrameAligner = frame_align.FrameAligner
sys.argv = ['x', '--tag', '_G_nooffset']
T.main()
