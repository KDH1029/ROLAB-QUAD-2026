#!/usr/bin/env python3
"""live_eval.py 결과(npz)를 같은 지표로 계산하고, 여러 개면 나란히 비교한다.

지표
  경로      오도메트리 3D 경로 길이와 다리 오도 경로의 비 (1에 가까울수록 미끄러짐 적음)
  범위      xy 범위 (경기장 밖으로 나가면 미끄러진 것)
  충격 튐   IMU 충격 상위 10% 구간에서 속도·고도 튐이 나머지 구간의 몇 배인가 (작을수록 좋음)
  재방문    30초 이상 간격으로 다시 지난 5 cm 바닥 칸의 높이 표준편차 (작을수록 일관)
사용: analyze.py out/A.npz [out/B.npz ...]
"""
import sys, json, numpy as np


def med_filter(x, k):
    pad = k // 2
    xp = np.pad(x, pad, mode='edge')
    return np.array([np.median(xp[i:i + k]) for i in range(len(x))])


def path_len(p):
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1))) if len(p) > 1 else float('nan')


def analyze(fn):
    d = np.load(fn)
    od, leg, imu = d['odom'], d['leg'], d['imu']
    r = {'파일': fn.split('/')[-1], '오도 표본': int(len(od))}
    if len(od) < 50:
        r['오류'] = '오도메트리 표본 부족'; return r
    t, p = od[:, 0], od[:, 1:4]
    r['재생 길이 s'] = round(float(t[-1] - t[0]), 1)
    r['오도 주기 Hz'] = round(len(t) / (t[-1] - t[0]), 2)
    L = path_len(p); r['경로 m'] = round(L, 1)
    if len(leg) > 50:
        Lleg = path_len(leg[:, 1:4]); r['다리오도 경로 m'] = round(Lleg, 1); r['경로/다리오도'] = round(L / Lleg, 3)
    r['x 범위 m'] = [round(float(p[:, 0].min()), 2), round(float(p[:, 0].max()), 2)]
    r['y 범위 m'] = [round(float(p[:, 1].min()), 2), round(float(p[:, 1].max()), 2)]
    r['z 범위 m'] = [round(float(p[:, 2].min()), 2), round(float(p[:, 2].max()), 2)]

    # 순간 속도·고도 속도와 그 튐(±0.5초 중앙값 대비 차이)
    dt = np.diff(t); dt[dt < 1e-3] = 1e-3
    v = np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1) / dt
    vz = np.abs(np.diff(p[:, 2])) / dt
    tm = t[1:]
    k = max(3, int(round(1.0 * r['오도 주기 Hz'])) | 1)
    jv = np.abs(v - med_filter(v, k)); jz = np.abs(vz - med_filter(vz, k))
    # IMU 충격: |a| 의 중앙값 대비 이탈을 0.5초 창으로 평균 → 상위 10% 창
    if len(imu) > 500:
        ti, a = imu[:, 0], np.linalg.norm(imu[:, 1:4], axis=1)
        dev = np.abs(a - np.median(a))
        edges = np.arange(ti[0], ti[-1], 0.5)
        idx = np.digitize(ti, edges)
        win = np.array([dev[idx == i].mean() if np.any(idx == i) else 0 for i in range(1, len(edges) + 1)])
        thr = np.percentile(win, 90)
        wi = np.clip(np.digitize(tm, edges), 1, len(win)) - 1
        hot = win[wi] >= thr
        r['충격 구간 속도튐 배율'] = round(float(jv[hot].mean() / max(jv[~hot].mean(), 1e-9)), 2)
        r['충격 구간 고도튐 배율'] = round(float(jz[hot].mean() / max(jz[~hot].mean(), 1e-9)), 2)
        r['충격 구간 속도튐 m/s'] = round(float(jv[hot].mean()), 3)
    r['속도튐 p99 m/s'] = round(float(np.percentile(jv, 99)), 3)

    # 재방문 바닥 높이 표준편차
    cnt, s, ss, t0, t1 = d['cnt'], d['s'], d['ss'], d['t0'], d['t1']
    m = (cnt >= 20) & ((t1 - t0) > 30)
    if m.sum() > 50:
        mu = s[m] / cnt[m]; sd = np.sqrt(np.maximum(ss[m] / cnt[m] - mu * mu, 0))
        r['재방문 칸 수'] = int(m.sum())
        r['재방문 바닥 높이 σ 중앙 mm'] = round(float(np.median(sd) * 1000), 1)
        r['재방문 바닥 높이 σ p90 mm'] = round(float(np.percentile(sd, 90) * 1000), 1)
    return r


def common_sigma(files):
    """모든 실행에서 공통으로 재방문된 칸만으로 σ 를 다시 잰다 (칸 구성 차이를 없앤 공정 비교)."""
    ds = [np.load(f) for f in files]
    masks = [(d['cnt'] >= 20) & ((d['t1'] - d['t0']) > 30) for d in ds]
    common = np.logical_and.reduce(masks)
    out = []
    for d in ds:
        c, s, ss = d['cnt'][common], d['s'][common], d['ss'][common]
        mu = s / c; sd = np.sqrt(np.maximum(ss / c - mu * mu, 0))
        out.append((int(common.sum()), round(float(np.median(sd) * 1000), 1), round(float(np.percentile(sd, 90) * 1000), 1)))
    return out


def main():
    res = [analyze(f) for f in sys.argv[1:]]
    if len(sys.argv) > 2:
        for r, (n, med, p90) in zip(res, common_sigma(sys.argv[1:])):
            r['공통 칸 수'] = n; r['공통 칸 σ 중앙 mm'] = med; r['공통 칸 σ p90 mm'] = p90
    keys = []
    for r in res:
        for k in r:
            if k not in keys: keys.append(k)
    w = max(len(k) for k in keys) + 2
    for k in keys:
        print(k.ljust(w) + ' | '.join(str(r.get(k, '-')).ljust(22) for r in res))
    json.dump(res, open(sys.argv[1].rsplit('/', 1)[0] + '/summary.json', 'w'), ensure_ascii=False, indent=1)


if __name__ == '__main__':
    main()
