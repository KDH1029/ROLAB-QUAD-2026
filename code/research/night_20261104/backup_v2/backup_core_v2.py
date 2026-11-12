#!/usr/bin/env python3
"""백업 위치추정 v2 = v_gosu/backup/backup_core.BackupEstimator (읽기만, 고치지 않음) + 부품 두 개 교체. (+ 11-04 match_correction: frame_align 용)
  정합기: ICP 3자유도 → GICP (small_gicp 1.0.1, 야간 venv)      ../backup_eval/RESULTS.md
  지면표: GroundGrid → RobustGroundGrid (천장 칸을 지면으로 내던 문제)
공정 검증(앞 270초 지도 → 뒤 270초): p95 0.099→0.050 m, 최대 0.282→0.086 m, 경사 위 p95 0.23→0.054 m, 거부 9.5%→0%.
"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(1, os.path.expanduser('~/ws/v_gosu/backup'))
from backup_core import BackupEstimator, MuxLogic, load_map, q2R, rpy_of, voxel   # noqa: F401 (노드가 그대로 가져다 씀)
from gicp_matcher import GicpMatcher
from robust_ground import RobustGroundGrid


def make_estimator(map_pts, rms_ok=0.08):
    est = BackupEstimator(map_pts, rms_ok=rms_ok)          # 원본 생성자가 만든 ICP·GroundGrid 는 바로 교체
    mp = voxel(map_pts, 0.05)
    est.matcher = GicpMatcher(mp, need_converged=False, iters=60)
    est.ground = RobustGroundGrid(mp)
    return est


# ── 11-04 07:40 추가 (frame_align 용): 정합 한 번의 '씨앗 대비 보정량'. 노드·시험이 같은 정의를 쓴다.
#    크면(> frame_align.corr_ok_*) 아직 수렴 중 — 정렬 쌍을 모으지 않는다.
def match_correction(prep, T):
    import math
    from backup_core import wrap
    sx, sy, syaw = prep['seed']
    return math.hypot(T[0, 3] - sx, T[1, 3] - sy), wrap(math.atan2(T[1, 0], T[0, 0]) - syaw)
