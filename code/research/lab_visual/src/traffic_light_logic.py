#!/usr/bin/env python3
"""신호등 판정 로직. ROS 도 YOLO 도 모른다 — 그래서 사진만으로 시험할 수 있다.

노드(`traffic_light_detector.py`)와 시험 도구(`tools/test_traffic_light.py`)가
**같은 이 코드**를 쓴다. 그래야 시험한 것과 로봇에 올라가는 것이 같다.

넣는 것: 매 프레임의 원시 판정 (RED / GREEN / UNKNOWN) 과 그때의 시각(초)
내는 것: 걸러진 상태, 초록 경과, 건너도 되는가

【왜 한 프레임으로 판정하지 않는가】
사람이 앞을 가리거나 햇빛에 날아가면 한 프레임이 통째로 틀린다.
최근 몇 프레임 중 몇 개 이상이 같은 답을 낼 때만 상태를 바꾼다.

【왜 초록만 보면 안 되는가】
규정은 "30초간 켜져 있는 초록불일 때만 건넌다" 다. 그런데 도착했을 때 이미 초록이면
**그게 켜진 지 1초인지 29초인지 알 수 없다.** 29초째에 출발하면 건너는 중에 빨간불이 된다.
그래서 빨강->초록 **전환을 직접 본 경우에만** 남은 시간을 안다고 판단한다.
전환을 못 봤으면 다음 주기를 기다린다. 한 주기 손해 보는 대신 확실하다.
"""
from collections import deque

RED, GREEN, UNKNOWN = 'RED', 'GREEN', 'UNKNOWN'


class TrafficLightLogic:

    def __init__(self, vote_of=5, vote_need=3,
                 green_seconds=30.0, cross_seconds=12.0, margin=5.0):
        if vote_need > vote_of:
            raise ValueError('vote_need 가 vote_of 보다 클 수 없다')
        if cross_seconds + margin >= green_seconds:
            raise ValueError('건너는 시간 %.0f + 여유 %.0f 가 초록 %.0f 초 이상이다 — 절대 못 건넌다'
                             % (cross_seconds, margin, green_seconds))
        self.vote_of = vote_of
        self.vote_need = vote_need
        self.green_seconds = green_seconds
        self.cross_seconds = cross_seconds
        self.margin = margin
        self.deadline = green_seconds - cross_seconds - margin

        self.votes = deque(maxlen=vote_of)
        self.state = UNKNOWN
        self.green_since = None    # 빨강->초록 전환을 본 시각. 못 봤으면 None
        self.saw_red = False       # 초록 전에 빨강을 실제로 봤는가
        self.changes = []          # (시각, 이전, 이후) 기록

    def vote(self, raw):
        """최근 프레임의 다수결. 흔들리는 한 프레임으로 상태를 바꾸지 않는다."""
        self.votes.append(raw)
        if len(self.votes) < self.vote_of:
            return self.state
        for s in (RED, GREEN):
            if sum(1 for v in self.votes if v == s) >= self.vote_need:
                return s
        # 어느 쪽도 문턱을 못 넘으면 직전 상태를 지킨다. 잠깐 가려진 것일 수 있다.
        return self.state

    def update(self, raw, now):
        """한 프레임을 넣는다. 돌려주는 것: (상태, 초록경과, 건너도되는가, 바뀐이유)"""
        new = self.vote(raw)
        note = ''
        if new != self.state:
            old = self.state
            if new == RED:
                self.saw_red = True
                self.green_since = None
            elif new == GREEN:
                # 빨강을 보고 있다가 초록이 됐다면 지금이 시작점이다.
                # 처음부터 초록이었다면(빨강을 본 적 없다면) 언제 켜졌는지 모른다.
                self.green_since = now if self.saw_red else None
                note = ('전환을 봤다 — 경과를 안다' if self.green_since is not None
                        else '도착했을 때 이미 초록 — 경과를 모른다. 다음 주기를 기다린다')
            self.state = new
            self.changes.append((now, old, new, note))

        el = -1.0
        ok = False
        if self.state == GREEN and self.green_since is not None:
            el = now - self.green_since
            ok = el <= self.deadline
        return self.state, el, ok, note

    def explain(self):
        return ('건너는 데 %.0f초, 초록 %.0f초 지속, 여유 %.0f초 '
                '-> 켜진 지 %.0f초 안에 출발해야 한다'
                % (self.cross_seconds, self.green_seconds, self.margin, self.deadline))
