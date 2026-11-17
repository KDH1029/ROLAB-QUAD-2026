"""시험 상황 — FAST-LIO 위치를 일부러 틀어 '보정이 필요한' 경우를 만든다(실측 밀림은 4 cm 라 그대로면 보정 필요 없음).
틀린 자세 = 피벗 둘레로 dth 돌리고 (dx, dy) 옮긴 것. t = 첫 자세 기준 초.
  S0 그대로       : 보정이 오히려 망가뜨리지 않나
  S1 순간 튐      : 100초에 0.30, -0.20 m, +8° (그 순간 로봇 자리 기준) — 넘어짐·충격 흉내(36 cm)
  S2 서서히 밀림  : 20초부터 200초 동안 0.40, 0.30 m, +6° 까지 (코스 가운데 기준)"""
import math

import numpy as np

SCEN = {'S0': None, 'S1': dict(t=100.0, d=(0.30, -0.20, math.radians(8.0))), 'S2': dict(t=20.0, dur=200.0, d=(0.40, 0.30, math.radians(6.0)), pivot=(3.0, 1.5))}


class Corrupt:
    def __init__(self, name):
        self.name = name; self.s = SCEN[name]; self.pivot = None

    def params(self, t, x, y):
        """시각 t(상대)·참 위치 → (dx, dy, dth, 피벗). S1 피벗은 처음 t 를 넘은 순간의 로봇 자리."""
        s = self.s
        if s is None:
            return 0.0, 0.0, 0.0, (0.0, 0.0)
        if self.name == 'S1':
            if t < s['t']:
                return 0.0, 0.0, 0.0, (0.0, 0.0)
            if self.pivot is None:
                self.pivot = (x, y)
            return (*s['d'], self.pivot)
        f = float(np.clip((t - s['t']) / s['dur'], 0.0, 1.0))
        return s['d'][0] * f, s['d'][1] * f, s['d'][2] * f, s['pivot']

    def apply(self, t, x, y, yaw):
        dx, dy, dth, (px, py) = self.params(t, x, y)
        c, s_ = math.cos(dth), math.sin(dth)
        return c * (x - px) - s_ * (y - py) + px + dx, s_ * (x - px) + c * (y - py) + py + dy, yaw + dth
