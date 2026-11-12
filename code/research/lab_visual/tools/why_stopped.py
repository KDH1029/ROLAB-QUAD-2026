#!/usr/bin/env python3
"""로봇이 왜 멈췄나 — 멈출 수 있는 모든 경로를 한 번에 시간순으로 보여준다.

  python3 tools/why_stopped.py            최근 120초
  python3 tools/why_stopped.py --sec 600  최근 10분
  python3 tools/why_stopped.py --follow   실시간

【2026-09-13 바뀐 것】
  z·점프·표류는 FAST-LIO 의 /Odometry 를 보는데, 그 값은 주행에 쓰이지 않는다
  (odom 은 다리 기구학이다). 그래서 **경고만 하고 멈추지 않는다.**
  '⓪ LIO 경고' 로 나오면 로봇은 계속 간 것이다 — 멈춤의 원인이 아니다.
  대신 그때 /scan 의 왜곡보정이 나빴을 수 있으니 정합률을 같이 볼 것.
  예전처럼 전부 막으려면: GO2_LIO_WARN_ONLY= ./go2_run.sh

【멈출 수 있는 경로 — 여섯 군데】
  ① 브리지 보류    NaN / 폭주 / 점프 / 표류 / z 이탈 / 오도메트리 끊김
                   -> cmd_vel_to_sport 가 정지 명령을 낸다. 조건이 풀리면 자동 재개.
  ② 장애물 감시    진행 방향 폭 안에 뭔가 있으면 전진 성분을 0 으로. 회전은 남는다.
  ③ 신호 게이트    횡단보도 앞에서 빨강이면 전진 0 (crosswalks.json 이 있을 때만)
  ④ Nav2          목표 실패 / 진행 없음 / 회복 동작(Spin·Wait·BackUp)
  ⑤ 데드맨        /cmd_vel 이 끊기면 정지. Nav2 나 미션이 죽은 것이다.
  ⑥ 감시자 재정위  모터는 안 멈추지만 AMCL 이 튀어 Nav2 가 경로를 다시 낸다

이 여섯을 각각 다른 로그에서 찾아야 해서 사람이 뒤지면 오래 걸린다. 한 번에 모은다.
"""
import argparse
import io
import os
import re
import subprocess
import time

LOGD = '/tmp/go2_run'
C0 = '\033[0m'; CR = '\033[1;31m'; CY = '\033[1;33m'; CG = '\033[1;32m'
CB = '\033[1;36m'; CD = '\033[2m'

# (로그파일, 정규식, 분류, 색)
RULES = [
    ('ctrl.log', r'주행 보류 \[(.+?)\]', '① 브리지 보류', CR),
    # 2026-09-13: z·점프·표류는 이제 경고만 한다(막지 않는다). 로그 문구가
    # 달라졌으므로 따로 잡는다 — 멈춤이 아니라 '스캔 품질 주의' 로 분류한다.
    ('ctrl.log', r'\*\*주행은 계속한다\*\*', '⓪ LIO 경고(안 멈춤)', CD),
    ('ctrl.log', r'멈추지 않는다', '⓪ LIO 경고(안 멈춤)', CD),
    ('ctrl.log', r'z 이탈로 주행 보류', '① z 이탈', CR),
    ('ctrl.log', r'오도메트리가 .*끊겨', '① 오도메트리 끊김', CR),
    ('ctrl.log', r'정상 복귀|주행 재개', '① 재개', CG),
    ('ctrl.log', r'장애물 근접 — 전진 차단', '② 장애물', CY),
    ('ctrl.log', r'제자리걸음 방지로', '② 완화 중', CY),
    ('ctrl.log', r'횡단보도 대기', '③ 신호 게이트', CY),
    ('ctrl.log', r'스캔이 .*끊김', '② 스캔 끊김(무방비)', CY),
    ('nav2.log', r'Failed to make progress', '④ 진행 없음', CR),
    ('nav2.log', r'\[follow_path\].*Aborting', '④ 경로추종 실패', CR),
    ('nav2.log', r'Running \w+', '④ 회복 동작', CY),
    ('nav2.log', r'Collision Ahead', '④ 회복 중 충돌위험', CY),
    ('nav2.log', r'goal.*[Aa]bort|Goal failed', '④ 목표 실패', CR),
    ('nav2.log', r'tick rate .* was exceeded', '④ CPU 부족', CY),
    ('nav2.log', r'Control loop missed', '④ CPU 부족', CY),
    ('guard.log', r'복구 실행: (\w+)', '⑥ 재정위 실행', CR),
    ('guard.log', r'-> (seed|global)', '⑥ 재정위 판단', CY),
    ('guard.log', r'엎드려 있음', '⑥ 판정 보류(앉음)', CD),
]


def env_check():
    """로그에 안 남는 고장을 먼저 본다.

    주행 중 갑자기 멈추는 원인 중 로그만 봐서는 절대 안 보이는 것이 있다.
    라이다 한 대를 드라이버 두 개가 동시에 물는 경우다. 커널이 UDP 패킷을
    둘로 나눠주고, 반쪽짜리 스캔 두 줄기가 같은 토픽에 섞여 LIO 로 들어가
    타임스탬프가 계속 뒤로 간다. LIO 가 발산하고 가드가 로봇을 세운다.
    ctrl.log 에는 '주행 보류' 만 찍히니 원인을 영원히 못 찾는다.
    2026-09-13 에 실제로 이것 때문에 주행이 반복해서 끊겼다.
    """
    bad = []
    print('  %s환경 점검%s' % (CB, C0))

    # ① 라이다 UDP 포트를 몇이 듣고 있는가
    try:
        out = subprocess.run(['ss', '-ulnp'], capture_output=True,
                             text=True, timeout=10).stdout
        own = [l for l in out.splitlines() if ':2368' in l]
        if len(own) > 1:
            print('    %s치명  라이다 포트 2368 을 %d 개가 동시에 듣는다%s'
                  % (CR, len(own), C0))
            for l in own:
                print('          %s' % re.sub(r'\s+', ' ', l.strip())[:120])
            print('          %s-> 스캔이 반쪽씩 섞인다. LIO 가 발산하고 로봇이 선다.%s'
                  % (CD, C0))
            bad.append('라이다 드라이버 중복')
        elif len(own) == 1:
            print('    %s정상  라이다 포트 2368 단독 점유%s' % (CG, C0))
        else:
            print('    %s주의  라이다 포트 2368 을 아무도 안 듣는다 (스택 꺼짐?)%s'
                  % (CY, C0))
    except Exception as e:
        print('    %s확인 못 함 (ss): %s%s' % (CD, e, C0))

    # ② LIO 가 타임스탬프 역행을 겪고 있는가 — 위 증상의 직접 증거
    lio = os.path.join(LOGD, 'lio.log')
    if os.path.exists(lio):
        try:
            txt = subprocess.run(['tail', '-20000', lio], capture_output=True,
                                 text=True, timeout=15).stdout
            loop = txt.count('loop back')
            nopt = txt.count('No point')
            if loop > 50 or nopt > 50:
                print('    %s치명  LIO 타임스탬프 역행 %d회, 빈 스캔 %d회%s'
                      % (CR, loop, nopt, C0))
                print('          %s-> 라이다 입력이 깨졌다. 오도메트리를 믿을 수 없다.%s'
                      % (CD, C0))
                bad.append('LIO 입력 깨짐')
            elif loop or nopt:
                print('    %s주의  LIO 역행 %d회, 빈 스캔 %d회%s'
                      % (CY, loop, nopt, C0))
            else:
                print('    %s정상  LIO 입력 깨끗함%s' % (CG, C0))
        except Exception:
            pass

    # ③ 표식 없는 노드 = 이 스택이 띄우지 않은 것
    try:
        pats = ('hesai_ros_driver|cloud_bridge|imu_bridge|nav_bridge'
                '|fastlio_mapping|laser_mapping|cmd_vel_to_sport')
        pids = subprocess.run(['pgrep', '-f', pats], capture_output=True,
                              text=True, timeout=10).stdout.split()
        foreign = []
        for p in pids:
            try:
                comm = io.open('/proc/%s/comm' % p).read().strip()
                env = io.open('/proc/%s/environ' % p, 'rb').read()
            except Exception:
                continue
            if comm in ('grep', 'ss', 'sshd', 'bash', 'tail', 'sh'):
                continue
            if b'GO2_STACK_OWNER=go2_run' not in env:
                foreign.append((p, comm))
        if foreign:
            print('    %s주의  이 스택이 띄우지 않은 노드 %d개%s'
                  % (CY, len(foreign), C0))
            for p, c in foreign[:6]:
                print('          PID %-7s %s' % (p, c))
            print('          %s-> 남의 스택이거나, 패치 이전 실행의 잔재다.%s'
                  % (CD, C0))
        else:
            print('    %s정상  떠 있는 노드가 전부 이 스택 소유%s' % (CG, C0))
    except Exception:
        pass

    print()
    return bad


def stamp(line):
    m = re.search(r'\[(\d{10})\.(\d+)\]', line)
    if m:
        return float(m.group(1) + '.' + m.group(2)[:3])
    m = re.search(r'^\s*(\d+)초', line)
    return None if not m else -float(m.group(1))


def scan(sec, quiet):
    env_bad = env_check()
    now = time.time()
    rows = []
    for fn in sorted({r[0] for r in RULES}):
        p = os.path.join(LOGD, fn)
        if not os.path.exists(p):
            continue
        try:
            txt = subprocess.run(['tail', '-4000', p], capture_output=True,
                                 text=True, timeout=15).stdout
        except Exception:
            continue
        for ln in txt.splitlines():
            if 'userwarning' in ln.lower() or 'warnings.warn' in ln.lower():
                continue
            for f, pat, kind, col in RULES:
                if f != fn or not re.search(pat, ln):
                    continue
                t = stamp(ln)
                if t is not None and t > 0 and now - t > sec:
                    break
                rows.append((t if (t and t > 0) else now, kind, col,
                             re.sub(r'\s+', ' ', ln.strip())[:150]))
                break
    rows.sort(key=lambda r: r[0])
    if not rows:
        print('  최근 %d초에 멈출 만한 사건이 없다.' % sec)
        print('  로그가 비었으면 스택이 안 떠 있는 것이다: ls %s' % LOGD)
        return
    print('  경과   분류                 내용')
    for t, kind, col, ln in rows[-80:]:
        age = now - t
        print('  %s-%4.0f초%s %s%-20s%s %s' % (CD, age, C0, col, kind, C0, ln))
    print()
    cnt = {}
    for _, kind, _, _ in rows:
        cnt[kind] = cnt.get(kind, 0) + 1
    print('  %s요약%s  %s' % (CB, C0,
          '  '.join('%s %d회' % (k, v) for k, v in
                    sorted(cnt.items(), key=lambda x: -x[1]))))
    top = max(cnt.items(), key=lambda x: x[1])[0]
    print('  %s가장 잦은 것: %s%s' % (CY, top, C0))
    if env_bad:
        print()
        print('  %s★ 위 사건들은 결과일 뿐이다. 진짜 원인은 환경 점검의 %s 다.%s'
              % (CR, ' / '.join(env_bad), C0))
        print('  %s  이걸 먼저 고치지 않으면 로그의 보류·회복은 계속 반복된다.%s'
              % (CR, C0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sec', type=float, default=120.0)
    ap.add_argument('--follow', action='store_true')
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args()
    if not a.follow:
        scan(a.sec, a.quiet)
        return 0
    print('  실시간 — Ctrl-C 로 끝냄')
    seen = 0
    try:
        while True:
            os.system('clear')
            scan(a.sec, a.quiet)
            seen += 1
            time.sleep(3.0)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
