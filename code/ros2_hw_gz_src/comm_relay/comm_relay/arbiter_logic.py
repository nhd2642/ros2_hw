"""cmd_vel 우선순위 선택 (ROS 의존 없음, 단위 시험 대상)."""


class Arbiter:
    def __init__(self, sources, timeout=0.5):
        """sources: [(이름, 우선순위)] — 숫자가 클수록 우선."""
        self.prio = dict(sources)
        self.timeout = timeout
        self.latest = {}             # 이름 -> (t, (vx, vy, wz))

    def on_cmd(self, name, t, cmd):
        self.latest[name] = (t, cmd)

    def select(self, t):
        """(선택된 이름 또는 'idle', 명령). 유효한 입력이 없으면 정지 명령."""
        best = None
        for name, (ts, cmd) in self.latest.items():
            if t - ts <= self.timeout and (best is None or self.prio[name] > self.prio[best[0]]):
                best = (name, cmd)
        return best if best else ('idle', (0.0, 0.0, 0.0))
