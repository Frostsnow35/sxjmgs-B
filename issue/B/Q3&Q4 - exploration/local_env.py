"""
Deterministic local emulator of the simulator rules.

This is NOT the official emulator.  It implements the same virtual-time rules
and physical rules described in the problem statement / attachments so that we
can test strategies before a real drill test is started by the user.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

SPEED = 5.0
MEASURE_TIME = 5.0
SWITCH_TIME = 1.0
CLEAR_NO_TARGET_TIME = 3.0
CLEAR_SUCCESS_TIME = 5.0
CLEAR_RADIUS = 20.0
NEAR_RADIUS = 5.0
ARENA_R = 1800.0
R_MIN = 1000.0
R_MAX = 1500.0
CHANNELS = list(range(1, 21))


@dataclass
class Source:
    channel: int
    x: float
    y: float
    radius: float
    direction: float | None = None  # None for omni, degrees for directional


@dataclass
class ActionResult:
    accepted: bool = True
    measure_result: str | None = None
    svd_deg: float | None = None
    clear_result: str | None = None
    virtual_time_s: float = 0.0
    move_time: float = 0.0
    switch_time: float = 0.0
    action_time: float = 0.0


class LocalEnv:
    """A tiny simulator used for Monte-Carlo rehearsal of robot strategies."""

    def __init__(self, mode: str = "q3", seed: int = 0,
                 n_sources: int | None = None,
                 direction_ratio: float | None = None,
                 error_sigma: float = 1.0):
        """mode: 'q3' (all omni) or 'q4' (mixed omni/directional)."""
        assert mode in ("q3", "q4")
        self.mode = mode
        self.rng = random.Random(seed)
        if n_sources is None:
            n_sources = self.rng.randint(10, 16)
        channels = self.rng.sample(CHANNELS, n_sources)
        self.sources: list[Source] = []
        for ch in channels:
            # uniform in the disk of radius 1800
            r = ARENA_R * math.sqrt(self.rng.random())
            a = 2.0 * math.pi * self.rng.random()
            x, y = r * math.cos(a), r * math.sin(a)
            radius = self.rng.uniform(R_MIN, R_MAX)
            direction = None
            if mode == "q4":
                # Mixed case.  ratio 0.5 by default.
                ratio = 0.5 if direction_ratio is None else direction_ratio
                if self.rng.random() < ratio:
                    direction = self.rng.uniform(0.0, 360.0)
            self.sources.append(Source(ch, x, y, radius, direction))
        self.sources.sort(key=lambda s: s.channel)

        self.virtual_time = 0.0
        self.pos = (0.0, 0.0)
        self.current_channel = 1
        self.cleared_channels: set[int] = set()
        self.entered = False
        self.exited = False
        self.requests = 0
        self.measure_count = 0
        self.clear_count = 0
        # per (source channel, measurement point) fixed random error in [-1,1]
        self._error_cache: dict[tuple[int, tuple[float, float]], float] = {}

    # ------------------------------------------------------------------
    def enter(self):
        self.entered = True
        self.virtual_time = 0.0
        self.pos = (0.0, 0.0)
        self.current_channel = 1
        self.cleared_channels.clear()
        self.requests = 0
        self.measure_count = 0
        self.clear_count = 0
        return {"accepted": True, "virtual_time_s": 0.0,
                "remaining_real_duration_s": 1200.0}

    def exit(self):
        self.exited = True
        return {"accepted": True, "virtual_time_s": self.virtual_time,
                "exit_reason": "user_exit"}

    def _active(self, channel: int) -> Source | None:
        if channel in self.cleared_channels:
            return None
        for s in self.sources:
            if s.channel == channel:
                return s
        return None

    @staticmethod
    def _norm_deg(a: float) -> float:
        a = math.fmod(a, 360.0)
        return a + 360.0 if a < 0 else a

    def _covered(self, src: Source, pos: tuple[float, float]) -> bool:
        if src.direction is None:
            return True
        # direction from source to detector
        ang = math.degrees(math.atan2(pos[1] - src.y, pos[0] - src.x))
        diff = abs((ang - src.direction + 180.0) % 360.0 - 180.0)
        return diff <= 90.0 + 1e-9

    def _measurement_error(self, src: Source, pos: tuple[float, float]) -> float:
        key = (src.channel, pos)
        if key not in self._error_cache:
            self._error_cache[key] = self.rng.uniform(-1.0, 1.0)
        return self._error_cache[key]

    # ------------------------------------------------------------------
    def measure(self, pos: tuple[float, float], channel: int):
        assert self.entered and not self.exited
        x, y = pos
        if not (math.isfinite(x) and math.isfinite(y)):
            return ActionResult(accepted=False)
        move_time = math.dist(self.pos, pos) / SPEED
        switch_time = SWITCH_TIME if channel != self.current_channel else 0.0
        self.virtual_time += move_time + switch_time + MEASURE_TIME
        self.pos = pos
        self.current_channel = channel
        self.requests += 1
        self.measure_count += 1
        src = self._active(channel)
        res = ActionResult(accepted=True, measure_result="no_signal",
                           virtual_time_s=self.virtual_time,
                           move_time=move_time, switch_time=switch_time,
                           action_time=MEASURE_TIME)
        if src is None:
            return res
        d = math.dist(pos, (src.x, src.y))
        if d > src.radius + 1e-9 or not self._covered(src, pos):
            return res
        if d <= NEAR_RADIUS + 1e-9:
            res.measure_result = "near"
            return res
        true_ang = self._norm_deg(math.degrees(math.atan2(src.y - y, src.x - x)))
        err = self._measurement_error(src, pos)
        res.svd_deg = self._norm_deg(true_ang + err)
        res.measure_result = "direction"
        return res

    def clear(self, pos: tuple[float, float], channel: int):
        assert self.entered and not self.exited
        x, y = pos
        move_time = math.dist(self.pos, pos) / SPEED
        src = self._active(channel)
        ok = src is not None and math.dist(pos, (src.x, src.y)) <= CLEAR_RADIUS + 1e-9
        action_time = CLEAR_SUCCESS_TIME if ok else CLEAR_NO_TARGET_TIME
        self.virtual_time += move_time + action_time
        self.pos = pos
        # /clear does not change the receiver channel
        self.requests += 1
        self.clear_count += 1
        if ok:
            self.cleared_channels.add(channel)
        return ActionResult(accepted=True,
                            clear_result="success" if ok else "no_target_in_range",
                            virtual_time_s=self.virtual_time,
                            move_time=move_time, switch_time=0.0,
                            action_time=action_time)

    # ------------------------------------------------------------------
    def source_state(self):
        return {
            "total": len(self.sources),
            "cleared": len(self.cleared_channels),
            "omni": sum(1 for s in self.sources if s.direction is None),
            "directional": sum(1 for s in self.sources if s.direction is not None),
        }
