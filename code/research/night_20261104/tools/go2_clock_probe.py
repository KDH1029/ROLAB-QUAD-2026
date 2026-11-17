#!/usr/bin/env python3
"""로봇 PC(내부 PC) 시계 − NUC 시계를 녹화 내내 잰다 (NTP 방식, 시스템 설정 변경 없음).

ssh 세션 하나를 계속 열어 두고(원격에서 파이썬이 줄을 받으면 자기 시각을 돌려준다), period 초마다 n 번 왕복해
가장 빠른 왕복 하나만 쓴다: offset = 원격시각 − (보낸 시각 + 받은 시각)/2, 오차 ≤ 왕복/2.
매번 ssh 를 새로 띄우지 않으므로 왕복이 LAN 에서 1 ms 아래로 나온다.

사용:  go2_clock_probe.py <out.csv> [period_s=2] [n=20]        (SIGINT/SIGTERM 으로 끝냄)
환경:  ROBOT=unitree@192.168.123.18, CM=~/.ssh/go2_cm_%r@%h:%p (go2_record_* 가 연 ssh 연결 재사용)
       PROBE_LOCAL=1 이면 ssh 대신 이 기계에서 원격 쪽 파이썬을 돌린다(시험용)
CSV:   nuc_time, offset_s(로봇PC−NUC), rtt_s, samples_ok
"""
import os, sys, time, select, signal, subprocess, shlex

OUT = sys.argv[1]; PERIOD = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0; N = int(sys.argv[3]) if len(sys.argv) > 3 else 20
ROBOT = os.environ.get('ROBOT', 'unitree@192.168.123.18'); CM = os.path.expanduser(os.environ.get('CM', '~/.ssh/go2_cm_%r@%h:%p'))
REMOTE_PY = 'import sys,time\nwhile True:\n l=sys.stdin.readline()\n if not l: break\n sys.stdout.write("T%.6f\\n" % time.time()); sys.stdout.flush()\n'
stop = False


def on_sig(*_):
    global stop
    stop = True


signal.signal(signal.SIGINT, on_sig); signal.signal(signal.SIGTERM, on_sig)


def start():
    if os.environ.get('PROBE_LOCAL') == '1':
        cmd = [sys.executable, '-u', '-c', REMOTE_PY]
    else:
        cmd = ['ssh', '-o', f'ControlPath={CM}', '-o', 'ConnectTimeout=6', '-o', 'BatchMode=yes', ROBOT,
               'python3 -u -c ' + shlex.quote(REMOTE_PY)]
    return subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)


def ping(p, timeout=1.0):
    """한 번 왕복 → (t0, t1, 원격시각) 또는 None. 'T' 로 시작하지 않는 줄(로그인 메뉴 출력 등)은 건너뛴다."""
    t0 = time.time(); p.stdin.write(b'x\n'); p.stdin.flush()
    buf = b''; end = t0 + timeout
    while time.time() < end:
        r, _, _ = select.select([p.stdout], [], [], max(0.0, end - time.time()))
        if not r: return None
        ch = os.read(p.stdout.fileno(), 256)
        if not ch: return None
        buf += ch
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            if line.startswith(b'T'):
                t1 = time.time()
                try: return t0, t1, float(line[1:])
                except ValueError: pass
    return None


new = not os.path.exists(OUT)
f = open(OUT, 'a', buffering=1)
if new: f.write('nuc_time,offset_s,rtt_s,samples_ok\n')
p = start(); fails = 0
while not stop:
    tick = time.time(); best = None; ok = 0
    for _ in range(N):
        if stop: break
        try: s = ping(p)
        except (BrokenPipeError, OSError): s = None
        if s is None:
            fails += 1
            try: p.kill()
            except Exception: pass
            time.sleep(0.5); p = start(); break
        ok += 1; t0, t1, r = s
        if best is None or (t1 - t0) < (best[1] - best[0]): best = s
    if best:
        t0, t1, r = best
        f.write(f'{(t0 + t1) / 2:.6f},{r - (t0 + t1) / 2:+.6f},{t1 - t0:.6f},{ok}\n')
    while not stop and time.time() - tick < PERIOD: time.sleep(0.05)
try: p.stdin.close(); p.kill()
except Exception: pass
f.close()
