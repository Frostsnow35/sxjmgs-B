"""问题四求解程序"""

import json
import math
import time
import uuid
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
from urllib.request import Request, urlopen
from urllib.error import URLError, HTTPError

BASE_URL = "http://127.0.0.1:2026"
ROBOT_ID = "<参赛队号>"

ANGLE_ERR_DEG = 1.0
MAX_RECV_R = 1500.0
ARENA_R = 1800.0
CLEAR_R = 20.0
NEAR_R = 5.0

SEARCH_MODE = "v4_budget_rescue"
SECOND_FORWARD = 400.0        # 第一测向点后，第二测向候选沿示向度前进
SECOND_LATERAL = 420.0        # 再向左右侧移，保证较大的交会角
MIN_CROSS_ANGLE = 18.0        # 两条中心线交角达到该值后，允许直接冲向LS估计点
MAX_REFINE_ROUNDS = 3
GUARANTEE_CLEAR_R = 18.0      # 有界误差区域最小包围圆半径<=18m，可保证中心清除
SOFT_CLEAR_R = 32.0           # <=32m时允许中心试清除

HTTP_TIMEOUT = 4.0
HTTP_RETRIES = 8
LOG_FILE = "q4_robot_trace.jsonl"
SUMMARY_FILE = "q4_run_summary.json"

Point = Tuple[float, float]
EPS = 1e-9


def dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def add(a: Point, b: Point) -> Point:
    return (a[0] + b[0], a[1] + b[1])


def sub(a: Point, b: Point) -> Point:
    return (a[0] - b[0], a[1] - b[1])


def mul(a: Point, k: float) -> Point:
    return (a[0] * k, a[1] * k)


def dot(a: Point, b: Point) -> float:
    return a[0] * b[0] + a[1] * b[1]


def cross(a: Point, b: Point) -> float:
    return a[0] * b[1] - a[1] * b[0]


def unit_from_deg(deg: float) -> Point:
    r = math.radians(deg)
    return (math.cos(r), math.sin(r))


def normalize(v: Point) -> Point:
    n = math.hypot(v[0], v[1])
    if n < EPS:
        return (1.0, 0.0)
    return (v[0] / n, v[1] / n)


def perp(v: Point) -> Point:
    return (-v[1], v[0])


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def polygon_centroid(poly: List[Point]) -> Point:
    if not poly:
        return (0.0, 0.0)
    if len(poly) < 3:
        return (sum(p[0] for p in poly) / len(poly), sum(p[1] for p in poly) / len(poly))
    a2 = 0.0
    cx = cy = 0.0
    for i, p in enumerate(poly):
        q = poly[(i + 1) % len(poly)]
        cr = cross(p, q)
        a2 += cr
        cx += (p[0] + q[0]) * cr
        cy += (p[1] + q[1]) * cr
    if abs(a2) < 1e-8:
        return (sum(p[0] for p in poly) / len(poly), sum(p[1] for p in poly) / len(poly))
    return (cx / (3.0 * a2), cy / (3.0 * a2))


def clip_halfplane(poly: List[Point], A: float, B: float, C: float) -> List[Point]:
    """保留 A*x+B*y+C >= 0 的部分。"""
    if not poly:
        return []
    out: List[Point] = []
    for i, P in enumerate(poly):
        Q = poly[(i + 1) % len(poly)]
        fP = A * P[0] + B * P[1] + C
        fQ = A * Q[0] + B * Q[1] + C
        inP = fP >= -1e-9
        inQ = fQ >= -1e-9
        if inP:
            out.append(P)
        if inP != inQ:
            den = fP - fQ
            if abs(den) > EPS:
                t = fP / den
                X = (P[0] + t * (Q[0] - P[0]), P[1] + t * (Q[1] - P[1]))
                out.append(X)
    return out


def circumscribed_disk_polygon(center: Point, radius: float, n: int = 64) -> List[Point]:
    """外接正n边形，保证真实圆盘被包含，不会错误排除真实源。"""
    R = radius / math.cos(math.pi / n)
    return [
        (center[0] + R * math.cos(2 * math.pi * k / n),
         center[1] + R * math.sin(2 * math.pi * k / n))
        for k in range(n)
    ]


def clip_by_circle_outer(poly: List[Point], center: Point, radius: float, n: int = 48) -> List[Point]:
    """用圆的切线半平面构成外接多边形，安全地加入 distance<=radius 信息。"""
    out = poly
    for k in range(n):
        ang = 2 * math.pi * k / n
        nx, ny = math.cos(ang), math.sin(ang)
        A, B = -nx, -ny
        C = nx * center[0] + ny * center[1] + radius
        out = clip_halfplane(out, A, B, C)
        if not out:
            break
    return out


def clip_by_bearing_wedge(poly: List[Point], s: Point, theta_deg: float, err_deg: float = ANGLE_ERR_DEG) -> List[Point]:
    """真实方位角位于 [theta-err, theta+err]，将多边形与该楔形相交。"""
    lo = unit_from_deg(theta_deg - err_deg)
    hi = unit_from_deg(theta_deg + err_deg)
    A1, B1 = -lo[1], lo[0]
    C1 = lo[1] * s[0] - lo[0] * s[1]
    out = clip_halfplane(poly, A1, B1, C1)
    A2, B2 = hi[1], -hi[0]
    C2 = -hi[1] * s[0] + hi[0] * s[1]
    out = clip_halfplane(out, A2, B2, C2)
    return out


def _circle2(a: Point, b: Point):
    c = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    return c, dist(c, a)


def _circle3(a: Point, b: Point, c: Point):
    ax, ay = a; bx, by = b; cx, cy = c
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-10:
        pairs = [(a, b), (a, c), (b, c)]
        return max((_circle2(x, y) for x, y in pairs), key=lambda z: z[1])
    ux = ((ax*ax + ay*ay)*(by-cy) + (bx*bx + by*by)*(cy-ay) + (cx*cx + cy*cy)*(ay-by)) / d
    uy = ((ax*ax + ay*ay)*(cx-bx) + (bx*bx + by*by)*(ax-cx) + (cx*cx + cy*cy)*(bx-ax)) / d
    cc = (ux, uy)
    return cc, dist(cc, a)


def _inside(circle, p: Point) -> bool:
    c, r = circle
    return dist(c, p) <= r + 1e-7


def min_enclosing_circle(points: List[Point]):
    if not points:
        return (0.0, 0.0), float("inf")
    c = (points[0], 0.0)
    for i in range(1, len(points)):
        if not _inside(c, points[i]):
            c = (points[i], 0.0)
            for j in range(i):
                if not _inside(c, points[j]):
                    c = _circle2(points[i], points[j])
                    for k in range(j):
                        if not _inside(c, points[k]):
                            c = _circle3(points[i], points[j], points[k])
    return c



class RobotAPI:
    def __init__(self):
        self.pos: Point = (0.0, 0.0)
        self.virtual_time = 0.0
        self.current_channel = 1
        self.seq = 0
        self.real_deadline = None
        self.log_fp = open(LOG_FILE, "w", encoding="utf-8")
        self.move_distance = 0.0
        self.measure_count = 0
        self.clear_count = 0
        self.clear_success = 0
        self.clear_fail = 0
        self.switch_count = 0

    def _rid(self, prefix: str) -> str:
        self.seq += 1
        return f"{prefix}-{self.seq}-{uuid.uuid4().hex[:8]}"

    def _base(self, rid: str) -> dict:
        return {"arena_id": "default", "robot_id": ROBOT_ID, "request_id": rid}

    def _post(self, path: str, payload: dict) -> dict:
        raw = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        last_err = None
        for attempt in range(HTTP_RETRIES):
            try:
                req = Request(BASE_URL + path, data=raw,
                              headers={"Content-Type": "application/json"}, method="POST")
                with urlopen(req, timeout=HTTP_TIMEOUT) as resp:
                    body = resp.read().decode("utf-8")
                    ans = json.loads(body)
                self.log_fp.write(json.dumps(
                    {"path": path, "payload": payload, "response": ans},
                    ensure_ascii=False) + "\n")
                self.log_fp.flush()
                if ans.get("accepted") is not True:
                    raise RuntimeError(f"请求未执行: {path} {ans}")
                if "virtual_time_s" in ans:
                    self.virtual_time = float(ans["virtual_time_s"])
                return ans
            except (URLError, HTTPError, TimeoutError, ConnectionError, json.JSONDecodeError) as e:
                last_err = e
                time.sleep(min(0.25 * (2 ** attempt), 2.0))
        raise RuntimeError(f"HTTP连续失败，最后错误: {last_err}")

    def enter(self):
        ans = self._post("/enter", self._base(self._rid("enter")))
        remain = float(ans.get("remaining_real_duration_s", 1200))
        self.real_deadline = time.time() + max(0.0, remain - 10.0)
        print("=" * 72)
        print(f"[ENTER] 本局现实时间预算 {remain:.0f}s")
        print(f"[MODE] {SEARCH_MODE} | 二次测向 forward={SECOND_FORWARD:.0f}m lateral={SECOND_LATERAL:.0f}m")
        print("=" * 72)
        return ans

    def measure(self, p: Point, ch: int):
        old = self.pos
        old_ch = self.current_channel
        rid = self._rid("measure")
        payload = self._base(rid)
        payload["position"] = {"x": round(float(p[0]), 6), "y": round(float(p[1]), 6)}
        payload["channel"] = int(ch)
        ans = self._post("/measure", payload)
        self.move_distance += dist(old, p)
        self.measure_count += 1
        if old_ch != ch:
            self.switch_count += 1
        self.pos = p
        self.current_channel = ch
        return ans

    def clear(self, p: Point, ch: int):
        old = self.pos
        rid = self._rid("clear")
        payload = self._base(rid)
        payload["position"] = {"x": round(float(p[0]), 6), "y": round(float(p[1]), 6)}
        payload["channel"] = int(ch)
        ans = self._post("/clear", payload)
        self.move_distance += dist(old, p)
        self.clear_count += 1
        if ans.get("clear_result") == "success":
            self.clear_success += 1
        else:
            self.clear_fail += 1
        self.pos = p
        return ans

    def exit(self):
        try:
            ans = self._post("/exit", self._base(self._rid("exit")))
            return ans
        finally:
            try:
                self.log_fp.close()
            except Exception:
                pass

    def enough_real_time(self, margin=25.0) -> bool:
        return self.real_deadline is None or time.time() < self.real_deadline - margin

    def summary(self, cleared: int) -> dict:
        avg = self.virtual_time / max(1, cleared)
        out = {
            "cleared": int(cleared),
            "virtual_time_s": round(self.virtual_time, 6),
            "average_time_s_per_source": round(avg, 6),
            "move_distance_m": round(self.move_distance, 3),
            "move_time_s_est": round(self.move_distance / 5.0, 3),
            "measure_count": self.measure_count,
            "channel_switch_count": self.switch_count,
            "clear_calls": self.clear_count,
            "clear_success": self.clear_success,
            "clear_fail": self.clear_fail,
            "search_mode": SEARCH_MODE,
        }
        with open(SUMMARY_FILE, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=2)
        return out


@dataclass
class BearingObs:
    p: Point
    deg: float


@dataclass
class TargetState:
    ch: int
    obs: List[BearingObs] = field(default_factory=list)
    poly: List[Point] = field(default_factory=lambda: circumscribed_disk_polygon((0.0, 0.0), ARENA_R, 72))
    cleared: bool = False
    last_signal_pos: Optional[Point] = None
    last_bearing: Optional[float] = None
    clear_attempted_points: List[Point] = field(default_factory=list)

    def add_bearing(self, p: Point, deg: float):
        for o in self.obs:
            if dist(o.p, p) < 1.0:
                return
        self.obs.append(BearingObs(p, deg))
        self.last_signal_pos = p
        self.last_bearing = deg
        self.poly = clip_by_bearing_wedge(self.poly, p, deg)
        self.poly = clip_by_circle_outer(self.poly, p, MAX_RECV_R, 40)

    def center_radius(self):
        if not self.poly:
            c = self.line_ls_estimate()
            return c, 9999.0
        return min_enclosing_circle(self.poly)

    def centroid(self):
        return polygon_centroid(self.poly) if self.poly else self.line_ls_estimate()

    def line_ls_estimate(self):
        a11 = a12 = a22 = b1 = b2 = 0.0
        for o in self.obs:
            u = unit_from_deg(o.deg)
            m11 = 1-u[0]*u[0]
            m12 = -u[0]*u[1]
            m22 = 1-u[1]*u[1]
            a11 += m11; a12 += m12; a22 += m22
            b1 += m11*o.p[0] + m12*o.p[1]
            b2 += m12*o.p[0] + m22*o.p[1]
        detm = a11*a22-a12*a12
        if abs(detm) < 1e-9:
            if self.obs:
                o = self.obs[-1]
                u = unit_from_deg(o.deg)
                return add(o.p, mul(u, 800.0))
            return (0.0, 0.0)
        x = (b1*a22-a12*b2)/detm
        y = (a11*b2-a12*b1)/detm
        rr = math.hypot(x, y)
        if rr > 1950:
            x *= 1950/rr; y *= 1950/rr
        return (x, y)

    @staticmethod
    def _acute_angle(a: float, b: float) -> float:
        d = abs((a - b + 180.0) % 360.0 - 180.0)
        return min(d, 180.0-d)

    def crossing_angle(self) -> float:
        if len(self.obs) < 2:
            return 0.0
        best = 0.0
        for i in range(len(self.obs)):
            for j in range(i):
                best = max(best, self._acute_angle(self.obs[i].deg, self.obs[j].deg))
        return best

    def has_observation_near(self, p: Point, tol=20.0) -> bool:
        return any(dist(o.p, p) <= tol for o in self.obs)


class Q4Solver:
    def __init__(self, api: RobotAPI):
        self.api = api
        self.targets: Dict[int, TargetState] = {ch: TargetState(ch) for ch in range(1, 21)}
        self.discovered = set()
        self.cleared = set()
        self.visited_search: List[Point] = []
        self.empty_hub_streak = 0

    def search_points(self):
        primary: List[Point] = [(0.0, 0.0)]
        for k in range(6):
            a = math.radians(60*k)
            primary.append((880*math.cos(a), 880*math.sin(a)))
        for k in range(6):
            a = math.radians(30 + 60*k)
            primary.append((1650*math.cos(a), 1650*math.sin(a)))

        extra: List[Point] = []
        for k in range(6):
            a = math.radians(30 + 60*k)
            extra.append((1250*math.cos(a), 1250*math.sin(a)))
        for k in range(6):
            a = math.radians(60*k)
            extra.append((1720*math.cos(a), 1720*math.sin(a)))
        return primary, extra

    def route_2opt(self, start: Point, pts: List[Point]) -> List[Point]:
        if not pts:
            return []
        remaining = pts[:]
        route = []
        cur = start
        while remaining:
            j = min(range(len(remaining)), key=lambda i: dist(cur, remaining[i]))
            cur = remaining.pop(j)
            route.append(cur)
        for _ in range(15):
            improved = False
            full = [start] + route
            for i in range(1, len(full)-2):
                for j in range(i+1, len(full)-1):
                    old = dist(full[i-1], full[i]) + dist(full[j], full[j+1])
                    new = dist(full[i-1], full[j]) + dist(full[i], full[j+1])
                    if new + 1e-8 < old:
                        full[i:j+1] = reversed(full[i:j+1])
                        improved = True
            route = full[1:]
            if not improved:
                break
        return route

    def pending_order_dp(self, channels: List[int]) -> List[int]:
        channels = [ch for ch in channels if ch not in self.cleared]
        n = len(channels)
        if n <= 1:
            return channels[:]
        proxy = []
        for ch in channels:
            t = self.targets[ch]
            proxy.append(t.line_ls_estimate() if len(t.obs) >= 2 else t.centroid())
        if n > 12:
            rem = list(range(n)); out=[]; cur=self.api.pos
            while rem:
                j=min(rem,key=lambda k:dist(cur,proxy[k]))
                out.append(channels[j]);cur=proxy[j];rem.remove(j)
            return out
        dp = {}
        for j in range(n):
            dp[(1<<j,j)] = (dist(self.api.pos, proxy[j]), -1)
        for mask in range(1,1<<n):
            for j in range(n):
                if (mask,j) not in dp:
                    continue
                cost,_=dp[(mask,j)]
                for k in range(n):
                    if mask&(1<<k): continue
                    nk=(mask|(1<<k),k)
                    nc=cost+dist(proxy[j],proxy[k])
                    if nk not in dp or nc<dp[nk][0]:
                        dp[nk]=(nc,j)
        full=(1<<n)-1
        end=min(range(n),key=lambda j:dp[(full,j)][0])
        order=[];mask=full;j=end
        while j!=-1:
            order.append(j)
            _,pj=dp[(mask,j)]
            mask^=(1<<j);j=pj
        order.reverse()
        return [channels[i] for i in order]

    def second_pair(self, t: TargetState) -> Tuple[Point, Point]:
        o = t.obs[0]
        u = unit_from_deg(o.deg)
        n = perp(u)
        base = add(o.p, mul(u, SECOND_FORWARD))
        return (add(base, mul(n, SECOND_LATERAL)),
                add(base, mul(n, -SECOND_LATERAL)))

    def refine_pair(self, t: TargetState) -> Tuple[Point, Point]:
        last = t.last_signal_pos
        if last is None or t.last_bearing is None:
            return (self.api.pos, self.api.pos)
        est = t.line_ls_estimate() if len(t.obs) >= 2 else add(last, mul(unit_from_deg(t.last_bearing), 600))
        d = dist(last, est)
        u = unit_from_deg(t.last_bearing)
        n = perp(u)
        f = clamp(0.35*d, 120.0, 320.0)
        b = clamp(0.42*d, 160.0, 320.0)
        base = add(last, mul(u, f))
        return add(base, mul(n,b)), add(base, mul(n,-b))

    def try_clear(self, t: TargetState, p: Point) -> bool:
        if any(dist(p,q)<1.0 for q in t.clear_attempted_points):
            return False
        t.clear_attempted_points.append(p)
        ans = self.api.clear(p, t.ch)
        if ans.get("clear_result") == "success":
            t.cleared = True
            self.cleared.add(t.ch)
            avg = self.api.virtual_time / max(1, len(self.cleared))
            print(f"  [CLEAR OK] ch={t.ch:2d} total={len(self.cleared):2d} "
                  f"vt={self.api.virtual_time:8.1f}s  当前平均={avg:6.1f}s/源")
            return True
        return False

    def measure_target_at(self, t: TargetState, p: Point) -> str:
        ans = self.api.measure(p, t.ch)
        mr = ans["measure_result"]
        if mr == "near":
            if self.try_clear(t, p):
                return "cleared"
            return "near"
        if mr == "direction":
            t.add_bearing(p, float(ans["svd_deg"]))
        return mr

    def batch_second_bearings(self, channels: List[int]):
        todo = [ch for ch in channels if ch not in self.cleared and len(self.targets[ch].obs)==1]
        if not todo:
            return
        print(f"[BATCH-2] 对 {len(todo)} 个新目标统一规划第二测向点，避免逐个回到发现点")

        remaining = set(todo)
        fallback = []

        while remaining and self.api.enough_real_time(25):
            best = None
            for ch in remaining:
                t = self.targets[ch]
                a,b = self.second_pair(t)
                da,db = dist(self.api.pos,a),dist(self.api.pos,b)
                p,opp = (a,b) if da<=db else (b,a)
                score=min(da,db)
                if best is None or score<best[0]:
                    best=(score,ch,p,opp)
            _,ch,p,opp=best
            remaining.remove(ch)
            t=self.targets[ch]
            mr=self.measure_target_at(t,p)
            if mr=="cleared":
                continue
            if mr=="no_signal":
                fallback.append((ch,opp))
            elif mr=="direction":
                if t.crossing_angle() < MIN_CROSS_ANGLE:
                    fallback.append((ch,opp))

        while fallback and self.api.enough_real_time(25):
            j=min(range(len(fallback)),key=lambda i:dist(self.api.pos,fallback[i][1]))
            ch,p=fallback.pop(j)
            if ch in self.cleared:
                continue
            t=self.targets[ch]
            if len(t.obs)>=2 and t.crossing_angle()>=MIN_CROSS_ANGLE:
                continue
            self.measure_target_at(t,p)

    def clear_ready_target(self, ch: int) -> bool:
        t=self.targets[ch]
        if t.cleared:
            return True
        if len(t.obs)<2:
            return False

        for _ in range(MAX_REFINE_ROUNDS+1):
            if not self.api.enough_real_time(20):
                return False
            angle=t.crossing_angle()
            center,rad=t.center_radius()
            ls=t.line_ls_estimate()
            print(f"  [LOC] ch={ch:2d} obs={len(t.obs)} angle={angle:5.1f}° regionR≈{rad:6.1f}m")

            if rad<=GUARANTEE_CLEAR_R:
                if self.try_clear(t,center):
                    return True

            if angle>=MIN_CROSS_ANGLE:
                if self.try_clear(t,ls):
                    return True

                mr=self.measure_target_at(t,self.api.pos)
                if mr=="cleared":
                    return True
                if mr=="direction":
                    ls2=t.line_ls_estimate()
                    if self.try_clear(t,ls2):
                        return True
                    center,rad=t.center_radius()
                    if rad<=SOFT_CLEAR_R and self.try_clear(t,center):
                        return True

            if (t.last_signal_pos is not None
                    and dist(self.api.pos,t.last_signal_pos)>=180
                    and not t.has_observation_near(self.api.pos,30)):
                mr=self.measure_target_at(t,self.api.pos)
                if mr=="cleared":
                    return True
                if mr=="direction" and t.crossing_angle()>=MIN_CROSS_ANGLE:
                    continue

            a,b=self.refine_pair(t)
            cands=sorted([a,b],key=lambda p:dist(self.api.pos,p))
            got=False
            for p in cands:
                mr=self.measure_target_at(t,p)
                if mr=="cleared":
                    return True
                if mr=="direction":
                    got=True
                    break
            if not got:
                return False

        center,rad=t.center_radius()
        if rad<=60 and self.try_clear(t,center):
            return True
        return False

    def process_channels(self, channels: List[int]):
        pending=[ch for ch in channels if ch not in self.cleared]
        ready=[ch for ch in pending if len(self.targets[ch].obs)>=2]
        for ch in self.pending_order_dp(ready):
            if ch in self.cleared:
                continue
            self.clear_ready_target(ch)

        ones=[ch for ch in pending if ch not in self.cleared and len(self.targets[ch].obs)==1]
        for ch in ones:
            if not self.api.enough_real_time(20):
                break
            t=self.targets[ch]
            if dist(self.api.pos,t.obs[0].p)>=180:
                self.measure_target_at(t,self.api.pos)
        ready=[ch for ch in ones if ch not in self.cleared and len(self.targets[ch].obs)>=2]
        for ch in self.pending_order_dp(ready):
            self.clear_ready_target(ch)

    def scan_at(self, p: Point):
        self.visited_search.append(p)
        before=set(self.discovered)
        channels=[ch for ch in range(1,21) if ch not in self.discovered and ch not in self.cleared]
        cc=self.api.current_channel
        channels.sort(key=lambda ch:(abs(ch-cc),ch))
        newly=[]
        print(f"\n[SCAN] hub=({p[0]:7.0f},{p[1]:7.0f})  未发现频道={len(channels):2d} "
              f"已清除={len(self.cleared):2d}  vt={self.api.virtual_time:.1f}s")
        for ch in channels:
            if not self.api.enough_real_time(20):
                break
            ans=self.api.measure(p,ch)
            mr=ans["measure_result"]
            if mr=="near":
                self.discovered.add(ch)
                if self.try_clear(self.targets[ch],p):
                    continue
            elif mr=="direction":
                self.discovered.add(ch)
                t=self.targets[ch]
                t.add_bearing(p,float(ans["svd_deg"]))
                newly.append(ch)

        nnew=len(self.discovered-before)
        if nnew==0:
            self.empty_hub_streak+=1
        else:
            self.empty_hub_streak=0
        print(f"[SCAN RESULT] 新发现={nnew}, 累计发现={len(self.discovered)}, 连续空hub={self.empty_hub_streak}")

        self.batch_second_bearings(newly)
        self.process_channels(newly)

        old_pending=[ch for ch in self.discovered if ch not in self.cleared and ch not in newly]
        if old_pending:
            self.process_channels(old_pending)

    def rescue_discovered(self):
        pending=[ch for ch in self.discovered if ch not in self.cleared]
        if not pending:
            return
        print(f"[RESCUE] 已发现未清除频道: {pending}")
        for ch in pending:
            t=self.targets[ch]
            if (not t.cleared and t.last_signal_pos is not None
                    and dist(self.api.pos,t.last_signal_pos)>=180
                    and not t.has_observation_near(self.api.pos,30)):
                self.measure_target_at(t,self.api.pos)
        for ch in self.pending_order_dp(pending):
            self.clear_ready_target(ch)

    def run_search_layer(self, points: List[Point], max_points: Optional[int]=None):
        unvisited=points[:]
        used=0
        while unvisited and len(self.cleared)<16 and self.api.enough_real_time(35):
            if max_points is not None and used>=max_points:
                break
            route=self.route_2opt(self.api.pos,unvisited)
            p=route[0]
            j=min(range(len(unvisited)),key=lambda i:dist(unvisited[i],p))
            p=unvisited.pop(j)
            self.scan_at(p)
            self.rescue_discovered()
            used+=1

    def extra_budget(self) -> int:
        if SEARCH_MODE=="safe":
            return 12
        if SEARCH_MODE=="fast":
            if len(self.cleared)<10: return 6
            if len(self.cleared)<=12: return 2
            return 0
        if len(self.cleared)<10: return 12
        if len(self.cleared)<=11: return 8
        if len(self.cleared)<=13: return 4
        if len(self.cleared)<=15: return 2
        return 0

    def solve(self):
        primary,extra=self.search_points()
        self.run_search_layer(primary)
        self.rescue_discovered()

        budget=self.extra_budget()
        if budget>0 and len(self.cleared)<16 and self.api.enough_real_time(60):
            print(f"\n[VERIFY] 主层结束：cleared={len(self.cleared)}，补充层最多再扫 {budget} 个点")
            ordered=self.route_2opt(self.api.pos,extra)
            self.run_search_layer(ordered,max_points=budget)
            self.rescue_discovered()

        avg=self.api.virtual_time/max(1,len(self.cleared))
        print("\n" + "="*72)
        print(f"[DONE] 清除数量         : {len(self.cleared)}")
        print(f"[DONE] 定位清除总时间   : {self.api.virtual_time:.3f} s")
        print(f"[DONE] 平均定位清除时间 : {avg:.3f} s/源")
        print(f"[DONE] 移动总距离       : {self.api.move_distance:.1f} m")
        print(f"[DONE] 检测次数/清除尝试: {self.api.measure_count}/{self.api.clear_count}")
        print("="*72)


def main():
    if ROBOT_ID.startswith("<"):
        print("请先修改脚本顶部 ROBOT_ID 为当前登录模拟器的参赛队号。")
        return

    api=RobotAPI()
    solver=None
    entered=False
    try:
        api.enter()
        entered=True
        solver=Q4Solver(api)
        solver.solve()
    except Exception as e:
        print("[FATAL]",repr(e))
    finally:
        if entered:
            try:
                ans=api.exit()
                print("[EXIT]",ans)
            except Exception as e:
                print("[EXIT failed]",repr(e))
        cleared=len(solver.cleared) if solver is not None else api.clear_success
        summary=api.summary(cleared)
        print("\n*** 最终统计（这个就是论文表格里的核心时间指标）***")
        print(f"清除干扰源个数 = {summary['cleared']}")
        print(f"定位清除总时间 = {summary['virtual_time_s']:.3f} s")
        print(f"平均定位清除时间 = {summary['average_time_s_per_source']:.3f} s/源")
        print(f"移动时间估计 = {summary['move_time_s_est']:.1f} s "
              f"({summary['move_distance_m']:.0f} m / 5m·s^-1)")
        print(f"结果同时写入 {SUMMARY_FILE}；本局日志为 {LOG_FILE}")




PRIMARY_RING_N = 7
PRIMARY_RING_R = 1000.0
V3_SECOND_FORWARD = 600.0
V3_SECOND_LATERAL = 250.0
REOBSERVE_MAX_PER_HUB = 7
REOBSERVE_PREDICT_R = 1400.0
REOBSERVE_MIN_BASELINE = 180.0
OPPORTUNISTIC_CLEAR_DETOUR = 230.0
OPPORTUNISTIC_CLEAR_MAX = 1
SERVICE_SCAN_MIN_SEP = 620.0
SERVICE_SCAN_UNKNOWN_MAX = 13
SERVICE_SCAN_MIN_CLEARED = 2
SERVICE_SCAN_MAX_COUNT = 6
RESCUE_RING_R = 1900.0
RESCUE_CAND_N = 14
COVERAGE_SAMPLE_STEP = 300.0


class Q4SolverV3(Q4Solver):
    def __init__(self, api: RobotAPI):
        super().__init__(api)
        self.measure_history: Dict[int, List[Point]] = {ch: [] for ch in range(1, 21)}
        self.full_scan_points: List[Point] = []
        self._in_service_scan = False
        self.service_scan_count = 0
        self.rescue_points_used: List[Point] = []
        self._coverage_grid = self._make_coverage_grid()

    def _record_measure(self, ch: int, p: Point):
        self.measure_history[ch].append(p)

    def _measured_near(self, ch: int, p: Point, tol: float = 1.0) -> bool:
        return any(dist(p, q) <= tol for q in self.measure_history[ch])

    def measure_target_at(self, t: TargetState, p: Point) -> str:
        ans = self.api.measure(p, t.ch)
        self._record_measure(t.ch, p)
        mr = ans["measure_result"]
        if mr == "near":
            if self.try_clear(t, p):
                return "cleared"
            return "near"
        if mr == "direction":
            t.add_bearing(p, float(ans["svd_deg"]))
        return mr

    def second_pair(self, t: TargetState) -> Tuple[Point, Point]:
        """
        复用问题三稳定版的长基线思想：先沿首测方位前进，再做较小横移。
        对定向源，较小横移比 v2 的 420m 更不容易一下跨出 180° 覆盖半平面；
        若一侧 no_signal，另一侧仍作为对称兜底。
        """
        o = t.obs[0]
        u = unit_from_deg(o.deg)
        n = perp(u)
        base = add(o.p, mul(u, V3_SECOND_FORWARD))
        return add(base, mul(n, V3_SECOND_LATERAL)), add(base, mul(n, -V3_SECOND_LATERAL))

    def _unknown_channels(self) -> List[int]:
        return [ch for ch in range(1, 21) if ch not in self.discovered and ch not in self.cleared]

    def _append_full_scan_point(self, p: Point):
        if all(dist(p, q) > 1.0 for q in self.full_scan_points):
            self.full_scan_points.append(p)

    def scan_unknown_at(self, p: Point, label: str = "HUB") -> List[int]:
        channels = self._unknown_channels()
        if not channels:
            return []
        cc = self.api.current_channel
        channels.sort(key=lambda ch: (abs(ch - cc), ch))
        newly: List[int] = []
        before = set(self.discovered)
        print(f"\n[{label}] p=({p[0]:7.0f},{p[1]:7.0f}) unknown={len(channels):2d} "
              f"found={len(self.discovered):2d} cleared={len(self.cleared):2d} vt={self.api.virtual_time:.1f}s")
        for ch in channels:
            if not self.api.enough_real_time(20):
                break
            if self._measured_near(ch, p):
                continue
            ans = self.api.measure(p, ch)
            self._record_measure(ch, p)
            mr = ans["measure_result"]
            if mr == "near":
                self.discovered.add(ch)
                newly.append(ch)
                self.try_clear(self.targets[ch], p)
            elif mr == "direction":
                self.discovered.add(ch)
                self.targets[ch].add_bearing(p, float(ans["svd_deg"]))
                newly.append(ch)
        self._append_full_scan_point(p)
        nnew = len(self.discovered - before)
        print(f"[{label} RESULT] 新发现={nnew}, 累计发现={len(self.discovered)}, cleared={len(self.cleared)}")
        return [ch for ch in newly if ch in self.discovered]

    def _rough_point(self, t: TargetState) -> Point:
        if len(t.obs) >= 2:
            return t.line_ls_estimate()
        if t.obs:
            o = t.obs[0]
            return add(o.p, mul(unit_from_deg(o.deg), 850.0))
        return (0.0, 0.0)

    def reobserve_at_hub(self, p: Point, max_count: int = REOBSERVE_MAX_PER_HUB):
        cand = []
        for ch in sorted(self.discovered):
            if ch in self.cleared:
                continue
            t = self.targets[ch]
            if not t.obs or self._measured_near(ch, p, 2.0):
                continue
            if len(t.obs) >= 2:
                _, rr = t.center_radius()
                if rr <= 45.0:
                    continue
                priority = 1
            else:
                priority = 0
            baseline = min(dist(p, o.p) for o in t.obs)
            if baseline < REOBSERVE_MIN_BASELINE:
                continue
            rough = self._rough_point(t)
            pred_d = dist(p, rough)
            if pred_d > REOBSERVE_PREDICT_R:
                continue
            score = (priority, pred_d - 0.22 * baseline, ch)
            cand.append((score, ch))
        cand.sort()
        if cand:
            print(f"[REOBSERVE] hub原地补测 {min(max_count, len(cand))} 个已发现目标")
        for _, ch in cand[:max_count]:
            if ch in self.cleared:
                continue
            self.measure_target_at(self.targets[ch], p)

    def try_clear(self, t: TargetState, p: Point) -> bool:
        if any(dist(p, q) < 1.0 for q in t.clear_attempted_points):
            return False
        t.clear_attempted_points.append(p)
        ans = self.api.clear(p, t.ch)
        if ans.get("clear_result") == "success":
            t.cleared = True
            self.cleared.add(t.ch)
            avg = self.api.virtual_time / max(1, len(self.cleared))
            print(f"  [CLEAR OK] ch={t.ch:2d} total={len(self.cleared):2d} "
                  f"vt={self.api.virtual_time:8.1f}s  当前平均={avg:6.1f}s/源")
            if not self._in_service_scan:
                self.maybe_service_scan_after_clear(p)
            return True
        return False

    def maybe_service_scan_after_clear(self, p: Point):
        unknown = self._unknown_channels()
        if not unknown:
            return
        if len(self.cleared) < SERVICE_SCAN_MIN_CLEARED:
            return
        if len(unknown) > SERVICE_SCAN_UNKNOWN_MAX:
            return
        if self.service_scan_count >= SERVICE_SCAN_MAX_COUNT:
            return
        if self.full_scan_points:
            novelty = min(dist(p, q) for q in self.full_scan_points)
            if novelty < SERVICE_SCAN_MIN_SEP:
                return
        self.service_scan_count += 1
        print(f"  [SERVICE-SCAN] 清除点本身作为新搜索点；原地扫 {len(unknown)} 个未知频道")
        self._in_service_scan = True
        try:
            self.scan_unknown_at(p, label="SERVICE")
            self.reobserve_at_hub(p, max_count=4)
        finally:
            self._in_service_scan = False

    def primary_ring_points(self) -> List[Point]:
        return [
            (PRIMARY_RING_R * math.cos(2 * math.pi * k / PRIMARY_RING_N),
             PRIMARY_RING_R * math.sin(2 * math.pi * k / PRIMARY_RING_N))
            for k in range(PRIMARY_RING_N)
        ]

    def _open_service_cost(self, start: Point, channels: List[int]) -> float:
        channels = [ch for ch in channels if ch not in self.cleared]
        if not channels:
            return 0.0
        pts = [self._rough_point(self.targets[ch]) for ch in channels]
        n = len(pts)
        if n > 11:
            rem = list(range(n)); cur = start; total = 0.0
            while rem:
                j = min(rem, key=lambda k: dist(cur, pts[k]))
                total += dist(cur, pts[j]); cur = pts[j]; rem.remove(j)
            return total
        dp = {}
        for j in range(n):
            dp[(1 << j, j)] = dist(start, pts[j])
        for mask in range(1, 1 << n):
            for j in range(n):
                if (mask, j) not in dp:
                    continue
                c = dp[(mask, j)]
                for k in range(n):
                    if mask & (1 << k):
                        continue
                    key = (mask | (1 << k), k)
                    nc = c + dist(pts[j], pts[k])
                    if key not in dp or nc < dp[key]:
                        dp[key] = nc
        full = (1 << n) - 1
        return min(dp[(full, j)] for j in range(n))

    def choose_primary_tour(self, ring_pts: List[Point]) -> List[Point]:
        if not self.discovered:
            return ring_pts[:]
        best = None
        best_route = ring_pts[:]
        pending = [ch for ch in self.discovered if ch not in self.cleared]
        for st in range(PRIMARY_RING_N):
            for direction in (1, -1):
                ids = [(st + direction * k) % PRIMARY_RING_N for k in range(PRIMARY_RING_N)]
                route = [ring_pts[i] for i in ids]
                endpoint = route[-1]
                service = self._open_service_cost(endpoint, pending)
                rank = 0
                for ch in pending:
                    t = self.targets[ch]
                    if len(t.obs) != 1:
                        continue
                    rough = self._rough_point(t)
                    first = PRIMARY_RING_N + 1
                    for k, p in enumerate(route):
                        if dist(p, t.obs[0].p) < REOBSERVE_MIN_BASELINE:
                            continue
                        if dist(p, rough) <= REOBSERVE_PREDICT_R:
                            first = k
                            break
                    rank += first
                key = (service + 22.0 * rank, rank)
                if best is None or key < best:
                    best = key
                    best_route = route
        return best_route

    def opportunistic_clear_on_leg(self, next_p: Point):
        """只做低绕路清除，避免再次出现 v2 的“发现一个就横穿全场”。"""
        done = 0
        while done < OPPORTUNISTIC_CLEAR_MAX:
            best = None
            cur = self.api.pos
            direct = dist(cur, next_p)
            for ch in sorted(self.discovered):
                if ch in self.cleared:
                    continue
                t = self.targets[ch]
                if len(t.obs) < 2:
                    continue
                center, rr = t.center_radius()
                if rr <= GUARANTEE_CLEAR_R:
                    q = center
                elif t.crossing_angle() >= MIN_CROSS_ANGLE:
                    q = t.line_ls_estimate()
                else:
                    continue
                detour = dist(cur, q) + dist(q, next_p) - direct
                if detour > OPPORTUNISTIC_CLEAR_DETOUR and dist(cur, q) > 260.0:
                    continue
                cand = (detour, dist(cur, q), ch, q)
                if best is None or cand < best:
                    best = cand
            if best is None:
                break
            _, _, ch, q = best
            t = self.targets[ch]
            print(f"[ON-LEG CLEAR] ch={ch} 顺路尝试")
            if not self.try_clear(t, q):
                mr = self.measure_target_at(t, self.api.pos)
                if mr == "direction":
                    q2 = t.line_ls_estimate()
                    if dist(self.api.pos, q2) <= 90.0:
                        self.try_clear(t, q2)
            done += 1

    def run_primary_search(self):
        self.scan_unknown_at((0.0, 0.0), label="CENTER")
        ring_pts = self.primary_ring_points()
        route = self.choose_primary_tour(ring_pts)
        print("[PRIMARY ROUTE] " + " -> ".join(f"({p[0]:.0f},{p[1]:.0f})" for p in route))
        for i, p in enumerate(route):
            if len(self.cleared) >= 16 or not self.api.enough_real_time(35):
                break
            self.scan_unknown_at(p, label="RING")
            self.reobserve_at_hub(p)
            if i + 1 < len(route):
                self.opportunistic_clear_on_leg(route[i + 1])

    def process_all_discovered(self):
        for _ in range(5):
            pending = [ch for ch in self.discovered if ch not in self.cleared]
            if not pending:
                return

            for ch in list(pending):
                if ch in self.cleared:
                    continue
                t = self.targets[ch]
                if len(t.obs) == 1 and not self._measured_near(ch, self.api.pos, 2.0):
                    rough = self._rough_point(t)
                    if (dist(self.api.pos, t.obs[0].p) >= REOBSERVE_MIN_BASELINE
                            and dist(self.api.pos, rough) <= REOBSERVE_PREDICT_R):
                        self.measure_target_at(t, self.api.pos)

            ready = [ch for ch in pending if ch not in self.cleared and len(self.targets[ch].obs) >= 2]
            for ch in self.pending_order_dp(ready):
                if ch not in self.cleared:
                    self.clear_ready_target(ch)

            ones = [ch for ch in self.discovered if ch not in self.cleared and len(self.targets[ch].obs) == 1]
            if ones:
                self.batch_second_bearings(ones)
                ready2 = [ch for ch in ones if ch not in self.cleared and len(self.targets[ch].obs) >= 2]
                for ch in self.pending_order_dp(ready2):
                    if ch not in self.cleared:
                        self.clear_ready_target(ch)

            after = [ch for ch in self.discovered if ch not in self.cleared]
            if not after:
                return
            before_n = len(after)
            self.rescue_discovered()
            after2 = [ch for ch in self.discovered if ch not in self.cleared]
            if len(after2) >= before_n:
                return

    def _make_coverage_grid(self) -> List[Point]:
        pts = []
        x = -ARENA_R
        while x <= ARENA_R + 1e-9:
            y = -ARENA_R
            while y <= ARENA_R + 1e-9:
                if x*x + y*y <= ARENA_R*ARENA_R + 1e-9:
                    pts.append((x, y))
                y += COVERAGE_SAMPLE_STEP
            x += COVERAGE_SAMPLE_STEP
        return pts

    @staticmethod
    def _orientation_fraction(scan_pts: List[Point], g: Point, recv_r: float = 1000.0) -> float:
        angs = []
        for p in scan_pts:
            d = dist(p, g)
            if d <= recv_r + 1e-9:
                if d < 1e-9:
                    return 1.0
                angs.append(math.atan2(p[1]-g[1], p[0]-g[0]) % (2*math.pi))
        if not angs:
            return 0.0
        intervals = []
        for a in angs:
            lo, hi = a - math.pi/2, a + math.pi/2
            while lo < 0:
                lo += 2*math.pi; hi += 2*math.pi
            while lo >= 2*math.pi:
                lo -= 2*math.pi; hi -= 2*math.pi
            if hi <= 2*math.pi:
                intervals.append((lo, hi))
            else:
                intervals.append((lo, 2*math.pi)); intervals.append((0.0, hi-2*math.pi))
        intervals.sort()
        total = 0.0
        lo, hi = intervals[0]
        for a, b in intervals[1:]:
            if a <= hi:
                hi = max(hi, b)
            else:
                total += hi-lo; lo, hi = a, b
        total += hi-lo
        return total/(2*math.pi)

    def directional_coverage_stats(self, extra: Optional[Point] = None) -> Tuple[float, float, float]:
        pts = list(self.full_scan_points)
        if extra is not None and all(dist(extra, q) > 1 for q in pts):
            pts.append(extra)
        vals = [self._orientation_fraction(pts, g) for g in self._coverage_grid]
        if not vals:
            return 0.0, 0.0, 0.0
        vals.sort()
        n = len(vals)
        mean = sum(vals)/n
        q10 = vals[min(n-1, max(0, int(0.10*(n-1))))]
        return mean, q10, vals[0]

    def rescue_candidates(self) -> List[Point]:
        off = 180.0 / RESCUE_CAND_N
        return [
            (RESCUE_RING_R * math.cos(math.radians(off + 360.0*k/RESCUE_CAND_N)),
             RESCUE_RING_R * math.sin(math.radians(off + 360.0*k/RESCUE_CAND_N)))
            for k in range(RESCUE_CAND_N)
        ]

    def choose_rescue_point(self, candidates: List[Point]) -> Point:
        base_mean, base_q10, _ = self.directional_coverage_stats()
        best = None
        for p in candidates:
            mean, q10, mn = self.directional_coverage_stats(extra=p)
            gain = 6500.0*(mean-base_mean) + 2200.0*(q10-base_q10)
            travel_pen = 0.18 * dist(self.api.pos, p)
            key = (gain - travel_pen, mean, q10, mn)
            if best is None or key > best[0]:
                best = (key, p)
        return best[1]

    def rescue_budget(self) -> int:
        c = len(self.cleared)
        mean, q10, _ = self.directional_coverage_stats()
        if c >= 10 and len(self.full_scan_points) >= 10 and mean >= 0.88 and q10 >= 0.62:
            return 1
        if c < 10:
            return 6
        if c <= 11:
            return 3
        if c <= 13:
            return 2
        if c <= 15:
            return 1
        return 0

    def run_directional_rescue(self):
        if len(self.cleared) >= 16:
            return
        candidates = self.rescue_candidates()
        budget = self.rescue_budget()
        if budget <= 0:
            return
        mean, q10, mn = self.directional_coverage_stats()
        print(f"\n[DIR-RESCUE] budget={budget}, coverage(mean/q10/min)="
              f"{mean:.3f}/{q10:.3f}/{mn:.3f}")
        empty = 0
        for _ in range(budget):
            if not candidates or not self.api.enough_real_time(45) or len(self.cleared) >= 16:
                break
            p = self.choose_rescue_point(candidates)
            candidates.remove(p)
            before = len(self.discovered)
            self.scan_unknown_at(p, label="DIR-PROBE")
            self.reobserve_at_hub(p, max_count=5)
            new_n = len(self.discovered) - before
            if new_n > 0:
                empty = 0
                self.process_all_discovered()
            else:
                empty += 1
            mean, q10, mn = self.directional_coverage_stats()
            print(f"[DIR-RESCUE SCORE] mean={mean:.3f} q10={q10:.3f} min={mn:.3f} empty={empty}")
            if len(self.cleared) >= 10:
                stop_empty = 1 if (mean >= 0.90 and q10 >= 0.68) else (2 if len(self.cleared) >= 12 else 3)
                if empty >= stop_empty:
                    print("[DIR-RESCUE] 连续空探针 + 覆盖评分达到停止条件，结束补盲")
                    break

    def solve(self):
        print("\n[V3] 采用问题三稳定版的核心思想：固定主搜索骨架 + 原地补测 + 统一DP服务 + 清除点复用")
        self.run_primary_search()
        self.process_all_discovered()
        self.run_directional_rescue()
        self.process_all_discovered()

        avg = self.api.virtual_time / max(1, len(self.cleared))
        mean, q10, mn = self.directional_coverage_stats()
        print("\n" + "="*72)
        print(f"[DONE] 清除数量         : {len(self.cleared)}")
        print(f"[DONE] 定位清除总时间   : {self.api.virtual_time:.3f} s")
        print(f"[DONE] 平均定位清除时间 : {avg:.3f} s/源")
        print(f"[DONE] 移动总距离       : {self.api.move_distance:.1f} m")
        print(f"[DONE] 检测次数/清除尝试: {self.api.measure_count}/{self.api.clear_count}")
        print(f"[DONE] 扫描点方向覆盖启发式(mean/q10/min): {mean:.3f}/{q10:.3f}/{mn:.3f}")
        print("="*72)



V4_RESCUE_MAX_EXTRA_S = 2000.0
V4_RESCUE_MAX_PROBES = 6
V4_TOTAL_VIRTUAL_CAP = 7200.0
V4_MESH_STEP = 990.0
V4_HALO_R = 1850.0
V4_CAND_DEDUP_M = 70.0


class Q4SolverV4(Q4SolverV3):
    def __init__(self, api: RobotAPI):
        super().__init__(api)
        self.v4_rescue_start_vt: Optional[float] = None
        self.v4_probe_count = 0

    @staticmethod
    def _dedup_points(points: List[Point], tol: float = V4_CAND_DEDUP_M) -> List[Point]:
        out: List[Point] = []
        for p in points:
            if all(dist(p, q) > tol for q in out):
                out.append(p)
        return out

    def v4_candidate_pool(self) -> List[Point]:
        """
        候选点由两部分组成：
        1) 边长 990m 的三角格点（含两层六边形格）；
        2) 半径 1850m 的 6 个边界外侧补点；
        3) 半径 1800m 的 12 个圆周点，用来填补某些方位盲区。

        990m 小于题目最小有效接收半径1000m；候选池本身不要求全部访问，
        而是每一步按“方向覆盖增益 / 预计耗时”在线选点。
        """
        s = V4_MESH_STEP
        pts: List[Point] = []
        for i in range(-4, 5):
            for j in range(-4, 5):
                x = s * (i + 0.5 * j)
                y = s * (math.sqrt(3.0) / 2.0 * j)
                if x*x + y*y <= (2.0*s + 1e-6)**2:
                    pts.append((x, y))
        for k in range(6):
            a = math.radians(30.0 + 60.0*k)
            pts.append((V4_HALO_R*math.cos(a), V4_HALO_R*math.sin(a)))
        for k in range(12):
            a = math.radians(30.0*k)
            pts.append((1800.0*math.cos(a), 1800.0*math.sin(a)))
        pts = self._dedup_points(pts)
        return [p for p in pts if all(dist(p, q) > V4_CAND_DEDUP_M for q in self.full_scan_points)]

    def _coverage_gain_score(self, p: Point, unknown_n: int) -> Tuple[float, Tuple[float,float,float], float]:
        bm, bq, bmin = self.directional_coverage_stats()
        m, q, mn = self.directional_coverage_stats(extra=p)
        gain = (m-bm) + 1.60*(q-bq) + 1.20*(mn-bmin)
        est_cost = dist(self.api.pos, p)/5.0 + 6.0*unknown_n
        score = gain / max(est_cost, 1.0)
        return score, (m, q, mn), est_cost

    def choose_v4_probe(self, candidates: List[Point]) -> Tuple[Optional[Point], float, Tuple[float,float,float]]:
        unknown_n = max(1, len(self._unknown_channels()))
        best = None
        for p in candidates:
            score, stats, cost = self._coverage_gain_score(p, unknown_n)
            key = (score, stats[2], stats[1], stats[0], -cost)
            if best is None or key > best[0]:
                best = (key, p, cost, stats)
        if best is None:
            return None, 0.0, self.directional_coverage_stats()
        return best[1], best[2], best[3]

    def run_v4_budget_rescue(self):
        """
        在主流程之后最多追加约2000s，目标是把“省下来的时间”换成定向源全清率。
        不再像V2那样无上限扫外环；达到预算或覆盖/空探针停止条件立即退出。
        """
        self.v4_rescue_start_vt = self.api.virtual_time
        candidates = self.v4_candidate_pool()
        empty = 0
        print(f"\n[V4-RESCUE] start vt={self.api.virtual_time:.1f}s, cleared={len(self.cleared)}, "
              f"unknown_ch={len(self._unknown_channels())}, candidates={len(candidates)}")

        for _ in range(V4_RESCUE_MAX_PROBES):
            if not candidates or not self.api.enough_real_time(55):
                break
            spent = self.api.virtual_time - self.v4_rescue_start_vt
            if spent >= V4_RESCUE_MAX_EXTRA_S or self.api.virtual_time >= V4_TOTAL_VIRTUAL_CAP:
                print("[V4-RESCUE] 到达额外虚拟时间预算，停止补盲")
                break

            p, est_cost, predicted_stats = self.choose_v4_probe(candidates)
            if p is None:
                break
            remaining = min(V4_RESCUE_MAX_EXTRA_S-spent, V4_TOTAL_VIRTUAL_CAP-self.api.virtual_time)
            if est_cost > remaining + 80.0:
                print(f"[V4-RESCUE] 下一点预计{est_cost:.0f}s，剩余预算{remaining:.0f}s，停止")
                break
            candidates.remove(p)
            before = len(self.discovered)
            print(f"[V4-PROBE] #{self.v4_probe_count+1} p=({p[0]:.0f},{p[1]:.0f}) "
                  f"est={est_cost:.0f}s predicted_cov={predicted_stats[0]:.3f}/"
                  f"{predicted_stats[1]:.3f}/{predicted_stats[2]:.3f}")
            self.v4_probe_count += 1
            self.scan_unknown_at(p, label="V4-PROBE")
            self.reobserve_at_hub(p, max_count=6)
            new_n = len(self.discovered) - before

            if new_n > 0:
                empty = 0
                print(f"[V4-PROBE] 新发现 {new_n} 个频道，立即统一定位/清除")
                self.process_all_discovered()
            else:
                empty += 1

            m, q, mn = self.directional_coverage_stats()
            spent = self.api.virtual_time - self.v4_rescue_start_vt
            print(f"[V4-RESCUE SCORE] spent={spent:.0f}s cleared={len(self.cleared)} "
                  f"empty={empty} coverage={m:.3f}/{q:.3f}/{mn:.3f}")

            if len(self.cleared) >= 10:
                if empty >= 2 and q >= 0.90:
                    print("[V4-RESCUE] 连续2个空探针且q10>=0.90，停止")
                    break
                if empty >= 3 and q >= 0.82:
                    print("[V4-RESCUE] 连续3个空探针且q10>=0.82，停止")
                    break

    def solve(self):
        print("\n[V4] 目标改为：优先全清，平均时间稳定在400~500s/源；不再硬追300s")
        print("[V4] 主流程沿用V3；最后仅用受控约2000s预算补定向源盲区")
        self.run_primary_search()
        self.process_all_discovered()
        self.run_v4_budget_rescue()
        self.process_all_discovered()

        avg = self.api.virtual_time / max(1, len(self.cleared))
        mean, q10, mn = self.directional_coverage_stats()
        print("\n" + "="*72)
        print(f"[DONE] 清除数量         : {len(self.cleared)}")
        print(f"[DONE] 定位清除总时间   : {self.api.virtual_time:.3f} s")
        print(f"[DONE] 平均定位清除时间 : {avg:.3f} s/源")
        print(f"[DONE] 移动总距离       : {self.api.move_distance:.1f} m")
        print(f"[DONE] V4补盲探针数     : {self.v4_probe_count}")
        print(f"[DONE] 扫描点方向覆盖启发式(mean/q10/min): {mean:.3f}/{q10:.3f}/{mn:.3f}")
        print("="*72)


def main_v4():
    if ROBOT_ID.startswith("<"):
        print("请先修改脚本顶部 ROBOT_ID 为当前登录模拟器的参赛队号。")
        return
    api = RobotAPI()
    solver = None
    entered = False
    try:
        api.enter()
        entered = True
        solver = Q4SolverV4(api)
        solver.solve()
    except Exception as e:
        print("[FATAL]", repr(e))
    finally:
        if entered:
            try:
                ans = api.exit()
                print("[EXIT]", ans)
            except Exception as e:
                print("[EXIT failed]", repr(e))
        cleared = len(solver.cleared) if solver is not None else api.clear_success
        summary = api.summary(cleared)
        if solver is not None:
            try:
                mean, q10, mn = solver.directional_coverage_stats()
                summary.update({
                    "strategy_version": "q4_v4_budget_rescue",
                    "full_scan_points": len(solver.full_scan_points),
                    "service_scan_count": solver.service_scan_count,
                    "v4_rescue_probe_count": solver.v4_probe_count,
                    "v4_rescue_extra_virtual_s": round(
                        0.0 if solver.v4_rescue_start_vt is None else
                        max(0.0, api.virtual_time-solver.v4_rescue_start_vt), 6),
                    "directional_coverage_mean": round(mean, 6),
                    "directional_coverage_q10": round(q10, 6),
                    "directional_coverage_min": round(mn, 6),
                })
                with open(SUMMARY_FILE, "w", encoding="utf-8") as f:
                    json.dump(summary, f, ensure_ascii=False, indent=2)
            except Exception:
                pass
        print("\n*** V4 最终统计 ***")
        print(f"清除干扰源个数 = {summary['cleared']}")
        print(f"定位清除总时间 = {summary['virtual_time_s']:.3f} s")
        print(f"平均定位清除时间 = {summary['average_time_s_per_source']:.3f} s/源")
        print(f"移动时间估计 = {summary['move_time_s_est']:.1f} s "
              f"({summary['move_distance_m']:.0f} m / 5m·s^-1)")
        print(f"结果写入 {SUMMARY_FILE}；日志为 {LOG_FILE}")




V5_TARGET_AVG_CAP = 500.0
V5_SEARCH_CREDIT_S = 350.0
V5_RESCUE_HARD_EXTRA_S = 2100.0
V5_RESCUE_MAX_PROBES = 7
V5_ABSOLUTE_VIRTUAL_CAP = 8000.0


class Q4SolverV5(Q4SolverV4):
    def __init__(self, api: RobotAPI):
        super().__init__(api)
        self.v5_rescue_start_vt: Optional[float] = None
        self.v5_probe_count = 0
        self.v5_new_channels = 0
        self.v5_new_clears = 0

    def _v5_base_probe_limit(self) -> int:
        c = len(self.cleared)
        if c >= 14:
            return 3
        if c == 13:
            return 4
        if c >= 11:
            return 5
        return 6

    def _v5_dynamic_total_cap(self) -> float:
        """
        若已经清了 14 个而实际上总数就是 14，则不允许为了“可能存在”的目标
        把平均时间长期拖过 500s/源；当清除数较低时仅额外借 350s 搜索信用。
        一旦找到并清除新源，cleared 增加，时间额度自然增加约 500s。
        """
        c = max(1, len(self.cleared))
        credit = V5_SEARCH_CREDIT_S if c <= 13 else 0.0
        return min(V5_ABSOLUTE_VIRTUAL_CAP, V5_TARGET_AVG_CAP * c + credit)

    def run_v5_evidence_rescue(self):
        self.v5_rescue_start_vt = self.api.virtual_time
        self.v4_rescue_start_vt = self.api.virtual_time
        candidates = self.v4_candidate_pool()
        empty = 0
        base_limit = self._v5_base_probe_limit()
        probe_limit = base_limit

        print(f"\n[V5-RESCUE] start vt={self.api.virtual_time:.1f}s, cleared={len(self.cleared)}, "
              f"unknown_ch={len(self._unknown_channels())}, base_probe_limit={base_limit}")

        while (self.v5_probe_count < probe_limit
               and self.v5_probe_count < V5_RESCUE_MAX_PROBES
               and candidates
               and self.api.enough_real_time(55)):
            spent = self.api.virtual_time - self.v5_rescue_start_vt
            if spent >= V5_RESCUE_HARD_EXTRA_S:
                print("[V5-RESCUE] 到达硬补盲预算，停止")
                break

            dyn_cap = self._v5_dynamic_total_cap()
            remaining = min(V5_RESCUE_HARD_EXTRA_S - spent,
                            dyn_cap - self.api.virtual_time,
                            V5_ABSOLUTE_VIRTUAL_CAP - self.api.virtual_time)
            if remaining <= 45.0:
                print(f"[V5-RESCUE] 动态平均时间预算不足（remaining={remaining:.0f}s），停止")
                break

            p, est_cost, predicted_stats = self.choose_v4_probe(candidates)
            if p is None:
                break
            if est_cost > remaining + 80.0:
                print(f"[V5-RESCUE] 下一点预计{est_cost:.0f}s，当前剩余预算{remaining:.0f}s，停止")
                break

            candidates.remove(p)
            before_disc = len(self.discovered)
            before_clear = len(self.cleared)
            before_stats = self.directional_coverage_stats()

            self.v5_probe_count += 1
            self.v4_probe_count += 1
            print(f"[V5-PROBE] #{self.v5_probe_count}/{probe_limit} p=({p[0]:.0f},{p[1]:.0f}) "
                  f"est={est_cost:.0f}s predicted_cov={predicted_stats[0]:.3f}/"
                  f"{predicted_stats[1]:.3f}/{predicted_stats[2]:.3f}")

            self.scan_unknown_at(p, label="V5-PROBE")
            self.reobserve_at_hub(p, max_count=6)
            new_n = len(self.discovered) - before_disc

            if new_n > 0:
                self.v5_new_channels += new_n
                empty = 0
                print(f"[V5-PROBE] 新发现 {new_n} 个频道，立即定位/清除，并增加后续验证额度")
                self.process_all_discovered()
                gained = max(0, len(self.cleared) - before_clear)
                self.v5_new_clears += gained
                probe_limit = min(V5_RESCUE_MAX_PROBES, max(probe_limit, self.v5_probe_count + 1))
            else:
                empty += 1

            m, q, mn = self.directional_coverage_stats()
            spent = self.api.virtual_time - self.v5_rescue_start_vt
            dq = q - before_stats[1]
            dm = m - before_stats[0]
            avg_now = self.api.virtual_time / max(1, len(self.cleared))
            print(f"[V5-RESCUE SCORE] spent={spent:.0f}s cleared={len(self.cleared)} "
                  f"avg={avg_now:.1f}s empty={empty} gain(mean/q10)={dm:+.3f}/{dq:+.3f} "
                  f"coverage={m:.3f}/{q:.3f}/{mn:.3f}")

            c = len(self.cleared)
            if c >= 14 and empty >= 3:
                print("[V5-RESCUE] 已清>=14且连续3个空探针，停止验证")
                break
            if c == 13 and empty >= 4:
                print("[V5-RESCUE] 已清13且连续4个空探针，停止验证")
                break
            if c <= 12 and empty >= 5:
                print("[V5-RESCUE] 连续5个空探针，停止验证")
                break

            if c >= 14 and empty >= 2 and dm < 0.012 and dq < 0.025:
                print("[V5-RESCUE] 连续空探针且边际覆盖增益很小，停止")
                break

    def solve(self):
        print("\n[V5] 目标：优先全清，同时把平均定位清除时间稳定压在约400~500s/源")
        print("[V5] 主搜索/定位沿用V4；补盲改为证据驱动的自适应预算")
        self.run_primary_search()
        self.process_all_discovered()
        self.run_v5_evidence_rescue()
        self.process_all_discovered()

        avg = self.api.virtual_time / max(1, len(self.cleared))
        mean, q10, mn = self.directional_coverage_stats()
        print("\n" + "="*72)
        print(f"[DONE] 清除数量         : {len(self.cleared)}")
        print(f"[DONE] 定位清除总时间   : {self.api.virtual_time:.3f} s")
        print(f"[DONE] 平均定位清除时间 : {avg:.3f} s/源")
        print(f"[DONE] 移动总距离       : {self.api.move_distance:.1f} m")
        print(f"[DONE] V5补盲探针数     : {self.v5_probe_count}")
        print(f"[DONE] V5补盲新发现/新清除: {self.v5_new_channels}/{self.v5_new_clears}")
        print(f"[DONE] 扫描点方向覆盖启发式(mean/q10/min): {mean:.3f}/{q10:.3f}/{mn:.3f}")
        print("="*72)


def main_v5():
    if ROBOT_ID.startswith("<"):
        print("请先修改脚本顶部 ROBOT_ID 为当前登录模拟器的参赛队号。")
        return
    api = RobotAPI()
    solver = None
    entered = False
    try:
        api.enter()
        entered = True
        solver = Q4SolverV5(api)
        solver.solve()
    except Exception as e:
        print("[FATAL]", repr(e))
    finally:
        if entered:
            try:
                ans = api.exit()
                print("[EXIT]", ans)
            except Exception as e:
                print("[EXIT failed]", repr(e))
        cleared = len(solver.cleared) if solver is not None else api.clear_success
        summary = api.summary(cleared)
        if solver is not None:
            try:
                mean, q10, mn = solver.directional_coverage_stats()
                summary.update({
                    "strategy_version": "q4_v5_evidence_budget",
                    "full_scan_points": len(solver.full_scan_points),
                    "service_scan_count": solver.service_scan_count,
                    "v5_rescue_probe_count": solver.v5_probe_count,
                    "v5_rescue_new_channels": solver.v5_new_channels,
                    "v5_rescue_new_clears": solver.v5_new_clears,
                    "v5_rescue_extra_virtual_s": round(
                        0.0 if solver.v5_rescue_start_vt is None else
                        max(0.0, api.virtual_time-solver.v5_rescue_start_vt), 6),
                    "directional_coverage_mean": round(mean, 6),
                    "directional_coverage_q10": round(q10, 6),
                    "directional_coverage_min": round(mn, 6),
                })
                with open(SUMMARY_FILE, "w", encoding="utf-8") as f:
                    json.dump(summary, f, ensure_ascii=False, indent=2)
            except Exception:
                pass
        print("\n最终统计：")
        print(f"清除干扰源个数 = {summary['cleared']}")
        print(f"定位清除总时间 = {summary['virtual_time_s']:.3f} s")
        print(f"平均定位清除时间 = {summary['average_time_s_per_source']:.3f} s/源")
        print(f"移动时间估计 = {summary['move_time_s_est']:.1f} s "
              f"({summary['move_distance_m']:.0f} m / 5m·s^-1)")
        print(f"结果写入 {SUMMARY_FILE}；日志为 {LOG_FILE}")





V6_SECOND_FORWARD = 120.0
V6_SECOND_LATERAL = 450.0
V6_SHORT_SWEEP_DISTS = (15.0, 50.0, 85.0, 120.0)
V6_RESCUE_MAX_PROBES = 6
V6_RESCUE_HARD_EXTRA_S = 2300.0
V6_ABSOLUTE_VIRTUAL_CAP = 8500.0
V6_TARGET_AVG_SOFT = 500.0
V6_SEARCH_CREDIT_S = 550.0


class Q4SolverV6(Q4SolverV5):
    def __init__(self, api: RobotAPI):
        super().__init__(api)
        self.v6_rescue_start_vt: Optional[float] = None
        self.v6_probe_count = 0
        self.v6_new_channels = 0
        self.v6_new_clears = 0
        self.v6_short_sweep_count = 0

    def second_pair(self, t: TargetState) -> Tuple[Point, Point]:
        o = t.obs[0]
        u = unit_from_deg(o.deg)
        n = perp(u)
        base = add(o.p, mul(u, V6_SECOND_FORWARD))
        return (add(base, mul(n, V6_SECOND_LATERAL)),
                add(base, mul(n, -V6_SECOND_LATERAL)))

    def _short_ray_clear_sweep(self, t: TargetState) -> bool:
        """仅在一条示向的两个鲁棒二测点都 no_signal 时启用。

        这通常意味着目标非常靠近首次有信号点（约 120m 内）。
        沿首条示向中心线做四个 35m 间隔的 /clear，避免再跨场寻找第二基线。
        """
        if t.cleared or not t.obs:
            return t.cleared
        o = t.obs[0]
        u = unit_from_deg(o.deg)
        pts = [add(o.p, mul(u, d)) for d in V6_SHORT_SWEEP_DISTS]
        if dist(self.api.pos, pts[-1]) < dist(self.api.pos, pts[0]):
            pts = list(reversed(pts))
        self.v6_short_sweep_count += 1
        print(f"[V6-SHORT-SWEEP] ch={t.ch} 两侧二测均无信号，执行120m内短程清除扫线")
        for p in pts:
            if not self.api.enough_real_time(18):
                break
            if self.try_clear(t, p):
                return True
        return False

    def batch_second_bearings(self, channels: List[int]):
        todo = [ch for ch in channels
                if ch not in self.cleared and len(self.targets[ch].obs) == 1]
        if not todo:
            return
        print(f"[V6-BATCH-2] 对 {len(todo)} 个单示向目标执行 120m前进 + ±450m 对称二测")

        remaining = set(todo)
        fallback: List[Tuple[int, Point]] = []
        first_result: Dict[int, str] = {}

        while remaining and self.api.enough_real_time(25):
            best = None
            for ch in remaining:
                t = self.targets[ch]
                a, b = self.second_pair(t)
                da, db = dist(self.api.pos, a), dist(self.api.pos, b)
                p, opp = (a, b) if da <= db else (b, a)
                cand = (min(da, db), ch, p, opp)
                if best is None or cand < best:
                    best = cand
            _, ch, p, opp = best
            remaining.remove(ch)
            t = self.targets[ch]
            mr = self.measure_target_at(t, p)
            first_result[ch] = mr
            if mr == "cleared":
                continue
            if mr == "no_signal" or (mr == "direction" and t.crossing_angle() < MIN_CROSS_ANGLE):
                fallback.append((ch, opp))

        while fallback and self.api.enough_real_time(25):
            j = min(range(len(fallback)), key=lambda i: dist(self.api.pos, fallback[i][1]))
            ch, p = fallback.pop(j)
            if ch in self.cleared:
                continue
            t = self.targets[ch]
            if len(t.obs) >= 2 and t.crossing_angle() >= MIN_CROSS_ANGLE:
                continue
            self.measure_target_at(t, p)

        stuck = [ch for ch in todo
                 if ch not in self.cleared and len(self.targets[ch].obs) == 1]
        for ch in self.pending_order_dp(stuck):
            if ch in self.cleared:
                continue
            self._short_ray_clear_sweep(self.targets[ch])

    def _force_pending_reobserve_at_hub(self, p: Point, max_count: int = 6):
        pending = [ch for ch in self.discovered
                   if ch not in self.cleared and self.targets[ch].obs]
        def key(ch: int):
            t = self.targets[ch]
            _, rr = t.center_radius()
            return (len(t.obs), -rr, ch)
        pending.sort(key=key)
        n = 0
        for ch in pending:
            if n >= max_count or not self.api.enough_real_time(18):
                break
            if self._measured_near(ch, p, 2.0):
                continue
            self.measure_target_at(self.targets[ch], p)
            n += 1

    def _v6_choose_probe(self, candidates: List[Point]) -> Tuple[Optional[Point], float, Tuple[float,float,float]]:
        if not candidates:
            return None, 0.0, self.directional_coverage_stats()
        base_pts = list(self.full_scan_points)
        base_vals = [self._orientation_fraction(base_pts, g) for g in self._coverage_grid]
        unknown_n = max(1, len(self._unknown_channels()))
        best = None
        for p in candidates:
            vals = [self._orientation_fraction(base_pts + [p], g) for g in self._coverage_grid]
            weighted_gain = 0.0
            rescued_bad = 0
            for b, v in zip(base_vals, vals):
                dg = max(0.0, v - b)
                w = 1.0 + 7.0 * (1.0 - b) ** 2
                weighted_gain += w * dg
                if b < 0.70 and v >= 0.70:
                    rescued_bad += 1
            sv = sorted(vals)
            m = sum(sv) / len(sv)
            q10 = sv[min(len(sv)-1, max(0, int(0.10*(len(sv)-1))))]
            mn = sv[0]
            est_cost = dist(self.api.pos, p) / 5.0 + 6.0 * unknown_n
            score = (weighted_gain + 0.35 * rescued_bad) / max(1.0, est_cost ** 0.72)
            key = (score, rescued_bad, q10, mn, m, -est_cost)
            if best is None or key > best[0]:
                best = (key, p, est_cost, (m, q10, mn))
        return best[1], best[2], best[3]

    def _v6_probe_limits(self) -> Tuple[int, int]:
        """返回 (至少执行的探针数, 连续空探针停止阈值)。"""
        c = len(self.cleared)
        if c >= 15:
            return 1, 2
        if c >= 14:
            return 2, 3
        if c == 13:
            return 3, 4
        if c >= 11:
            return 4, 5
        return 4, 6

    def _v6_soft_total_cap(self) -> float:
        c = max(1, len(self.cleared))
        return min(V6_ABSOLUTE_VIRTUAL_CAP,
                   V6_TARGET_AVG_SOFT * c + V6_SEARCH_CREDIT_S)

    def run_v6_directional_rescue(self):
        self.v6_rescue_start_vt = self.api.virtual_time
        candidates = self.v4_candidate_pool()
        min_probes, empty_stop = self._v6_probe_limits()
        empty = 0
        print(f"\n[V6-RESCUE] start vt={self.api.virtual_time:.1f}s cleared={len(self.cleared)} "
              f"unknown={len(self._unknown_channels())} min_probe={min_probes} empty_stop={empty_stop}")

        while (self.v6_probe_count < V6_RESCUE_MAX_PROBES
               and candidates and self.api.enough_real_time(55)):
            spent = self.api.virtual_time - self.v6_rescue_start_vt
            if spent >= V6_RESCUE_HARD_EXTRA_S:
                print("[V6-RESCUE] 达到硬补盲预算，停止")
                break
            if self.api.virtual_time >= V6_ABSOLUTE_VIRTUAL_CAP:
                break

            p, est_cost, pred = self._v6_choose_probe(candidates)
            if p is None:
                break
            soft_cap = self._v6_soft_total_cap()
            if self.v6_probe_count >= min_probes and self.api.virtual_time + est_cost > soft_cap + 100.0:
                print(f"[V6-RESCUE] 已完成最低验证，下一探针预计超软预算({soft_cap:.0f}s)，停止")
                break

            candidates.remove(p)
            before_disc = len(self.discovered)
            before_clear = len(self.cleared)
            self.v6_probe_count += 1
            print(f"[PROBE] #{self.v6_probe_count} p=({p[0]:.0f},{p[1]:.0f}) "
                  f"est={est_cost:.0f}s pred_cov={pred[0]:.3f}/{pred[1]:.3f}/{pred[2]:.3f}")

            self.scan_unknown_at(p, label="V6-PROBE")
            self._force_pending_reobserve_at_hub(p, max_count=6)
            new_n = len(self.discovered) - before_disc
            if new_n > 0:
                self.v6_new_channels += new_n
                empty = 0
                self.process_all_discovered()
                self.v6_new_clears += max(0, len(self.cleared) - before_clear)
            else:
                self.process_all_discovered()
                if len(self.cleared) > before_clear:
                    empty = 0
                    self.v6_new_clears += len(self.cleared) - before_clear
                else:
                    empty += 1

            m, q, mn = self.directional_coverage_stats()
            avg_now = self.api.virtual_time / max(1, len(self.cleared))
            print(f"[RESCUE SCORE] cleared={len(self.cleared)} avg={avg_now:.1f}s "
                  f"empty={empty} cov={m:.3f}/{q:.3f}/{mn:.3f}")

            min_probes, empty_stop = self._v6_probe_limits()
            if self.v6_probe_count >= min_probes and empty >= empty_stop:
                print("[RESCUE] 达到连续空探针停止阈值，结束补盲")
                break

    def solve(self):
        self.run_primary_search()
        self.process_all_discovered()
        self.run_v6_directional_rescue()
        self.process_all_discovered()

        avg = self.api.virtual_time / max(1, len(self.cleared))
        mean, q10, mn = self.directional_coverage_stats()
        print("\n" + "="*72)
        print(f"[DONE] 清除数量         : {len(self.cleared)}")
        print(f"[DONE] 定位清除总时间   : {self.api.virtual_time:.3f} s")
        print(f"[DONE] 平均定位清除时间 : {avg:.3f} s/源")
        print(f"[DONE] 移动总距离       : {self.api.move_distance:.1f} m")
        print(f"[DONE] 短程扫线次数   : {self.v6_short_sweep_count}")
        print(f"[DONE] 补盲探针数     : {self.v6_probe_count}")
        print(f"[DONE] 补盲新发现/新清除: {self.v6_new_channels}/{self.v6_new_clears}")
        print(f"[DONE] 扫描点方向覆盖启发式(mean/q10/min): {mean:.3f}/{q10:.3f}/{mn:.3f}")
        print("="*72)


def main_v6():
    if ROBOT_ID.startswith("<"):
        print("请先修改脚本顶部 ROBOT_ID 为当前登录模拟器的参赛队号。")
        return
    api = RobotAPI()
    solver = None
    entered = False
    try:
        api.enter()
        entered = True
        solver = Q4SolverV6(api)
        solver.solve()
    except Exception as e:
        print("[FATAL]", repr(e))
    finally:
        if entered:
            try:
                ans = api.exit()
                print("[EXIT]", ans)
            except Exception as e:
                print("[EXIT failed]", repr(e))
        cleared = len(solver.cleared) if solver is not None else api.clear_success
        summary = api.summary(cleared)
        if solver is not None:
            try:
                mean, q10, mn = solver.directional_coverage_stats()
                summary.update({
                    "strategy_version": "q4_v6_directional_robust",
                    "full_scan_points": len(solver.full_scan_points),
                    "service_scan_count": solver.service_scan_count,
                    "v6_short_sweep_count": solver.v6_short_sweep_count,
                    "v6_rescue_probe_count": solver.v6_probe_count,
                    "v6_rescue_new_channels": solver.v6_new_channels,
                    "v6_rescue_new_clears": solver.v6_new_clears,
                    "v6_rescue_extra_virtual_s": round(
                        0.0 if solver.v6_rescue_start_vt is None else
                        max(0.0, api.virtual_time - solver.v6_rescue_start_vt), 6),
                    "directional_coverage_mean": round(mean, 6),
                    "directional_coverage_q10": round(q10, 6),
                    "directional_coverage_min": round(mn, 6),
                })
                with open(SUMMARY_FILE, "w", encoding="utf-8") as f:
                    json.dump(summary, f, ensure_ascii=False, indent=2)
            except Exception:
                pass
        print("\n最终统计:")
        print(f"清除干扰源个数 = {summary['cleared']}")
        print(f"定位清除总时间 = {summary['virtual_time_s']:.3f} s")
        print(f"平均定位清除时间 = {summary['average_time_s_per_source']:.3f} s/源")
        print(f"移动时间估计 = {summary['move_time_s_est']:.1f} s "
              f"({summary['move_distance_m']:.0f} m / 5m·s^-1)")
        print(f"结果写入 {SUMMARY_FILE}；日志为 {LOG_FILE}")




V7_PRE_RESCUE_MAX_PROBES = 3
V7_POST_RESCUE_MAX_PROBES = 3
V7_PRE_HARD_EXTRA_S = 1150.0
V7_POST_HARD_EXTRA_S = 1450.0
V7_ABSOLUTE_VIRTUAL_CAP = 8200.0
V7_TARGET_AVG_SOFT = 505.0
V7_SOFT_CREDIT_S = 650.0

V7_CLEAR_HUB_REOBSERVE_MAX = 4
V7_CLEAR_HUB_PREDICT_R = 1250.0
V7_CLEAR_HUB_MIN_BASELINE = 170.0


class Q4SolverV7(Q4SolverV6):
    def __init__(self, api: RobotAPI):
        super().__init__(api)
        self.v7_pre_probe_count = 0
        self.v7_post_probe_count = 0
        self.v7_pre_new_channels = 0
        self.v7_post_new_channels = 0
        self.v7_post_new_clears = 0
        self.v7_pre_start_vt: Optional[float] = None
        self.v7_post_start_vt: Optional[float] = None
        self._v7_in_clear_hub = False

    def _v7_reobserve_after_clear(self, p: Point):
        if self._v7_in_clear_hub:
            return
        cand = []
        for ch in sorted(self.discovered):
            if ch in self.cleared:
                continue
            t = self.targets[ch]
            if len(t.obs) != 1:
                continue
            if self._measured_near(ch, p, 2.0):
                continue
            baseline = dist(p, t.obs[0].p)
            if baseline < V7_CLEAR_HUB_MIN_BASELINE:
                continue
            rough = self._rough_point(t)
            pred = dist(p, rough)
            if pred > V7_CLEAR_HUB_PREDICT_R:
                continue
            score = pred - 0.20 * baseline
            cand.append((score, ch))
        cand.sort()
        if not cand:
            return
        self._v7_in_clear_hub = True
        try:
            take = cand[:V7_CLEAR_HUB_REOBSERVE_MAX]
            print(f"  [V7-CLEAR-HUB] 原地补测 {len(take)} 个单示向目标")
            for _, ch in take:
                if ch in self.cleared:
                    continue
                self.measure_target_at(self.targets[ch], p)
        finally:
            self._v7_in_clear_hub = False

    def try_clear(self, t: TargetState, p: Point) -> bool:
        if any(dist(p, q) < 1.0 for q in t.clear_attempted_points):
            return False
        t.clear_attempted_points.append(p)
        ans = self.api.clear(p, t.ch)
        if ans.get("clear_result") == "success":
            t.cleared = True
            self.cleared.add(t.ch)
            avg = self.api.virtual_time / max(1, len(self.cleared))
            print(f"  [CLEAR OK] ch={t.ch:2d} total={len(self.cleared):2d} "
                  f"vt={self.api.virtual_time:8.1f}s  当前平均={avg:6.1f}s/源")
            if not self._in_service_scan:
                self.maybe_service_scan_after_clear(p)
            self._v7_reobserve_after_clear(p)
            return True
        return False

    def _v7_pre_probe_limit(self) -> int:
        d = len(self.discovered)
        if d >= 12:
            return 1
        if d >= 9:
            return 2
        return 3

    def run_v7_pre_rescue(self):
        self.v7_pre_start_vt = self.api.virtual_time
        limit = min(V7_PRE_RESCUE_MAX_PROBES, self._v7_pre_probe_limit())
        candidates = self.v4_candidate_pool()
        print(f"\n[V7-PRE] 主环结束先补盲：discovered={len(self.discovered)} "
              f"unknown={len(self._unknown_channels())} limit={limit}")

        for _ in range(limit):
            if not candidates or not self.api.enough_real_time(45):
                break
            if self.api.virtual_time - self.v7_pre_start_vt >= V7_PRE_HARD_EXTRA_S:
                break
            p, est_cost, pred = self._v6_choose_probe(candidates)
            if p is None:
                break
            candidates.remove(p)
            before = len(self.discovered)
            self.v7_pre_probe_count += 1
            print(f"[V7-PRE-PROBE] #{self.v7_pre_probe_count} "
                  f"p=({p[0]:.0f},{p[1]:.0f}) est={est_cost:.0f}s "
                  f"pred_cov={pred[0]:.3f}/{pred[1]:.3f}/{pred[2]:.3f}")

            self.scan_unknown_at(p, label="V7-PRE")
            self._force_pending_reobserve_at_hub(p, max_count=7)

            new_n = len(self.discovered) - before
            self.v7_pre_new_channels += max(0, new_n)
            if new_n > 0:
                candidates = self.v4_candidate_pool()

    def process_all_discovered(self):
        for _ in range(7):
            pending = [ch for ch in self.discovered if ch not in self.cleared]
            if not pending:
                return

            self.reobserve_at_hub(self.api.pos, max_count=8)

            ones = [ch for ch in self.discovered
                    if ch not in self.cleared and len(self.targets[ch].obs) == 1]
            if ones:
                self.batch_second_bearings(ones)

            any_progress = False
            failed_ready = set()

            while self.api.enough_real_time(22):
                ready = [ch for ch in self.discovered
                         if ch not in self.cleared
                         and len(self.targets[ch].obs) >= 2
                         and ch not in failed_ready]
                if not ready:
                    break
                order = self.pending_order_dp(ready)
                if not order:
                    break
                ch = order[0]
                before = len(self.cleared)
                ok = self.clear_ready_target(ch)
                if ok or len(self.cleared) > before:
                    any_progress = True
                else:
                    failed_ready.add(ch)

            ones2 = [ch for ch in self.discovered
                     if ch not in self.cleared and len(self.targets[ch].obs) == 1]
            if ones2:
                before_obs = sum(len(self.targets[ch].obs) for ch in ones2)
                self.batch_second_bearings(ones2)
                after_obs = sum(len(self.targets[ch].obs) for ch in ones2)
                if after_obs > before_obs:
                    any_progress = True

            left = [ch for ch in self.discovered if ch not in self.cleared]
            if not left:
                return

            before_clear = len(self.cleared)
            before_obs_all = sum(len(self.targets[ch].obs) for ch in left)
            self.rescue_discovered()
            after_left = [ch for ch in self.discovered if ch not in self.cleared]
            after_obs_all = sum(len(self.targets[ch].obs) for ch in after_left)
            if len(self.cleared) > before_clear or after_obs_all > before_obs_all:
                any_progress = True

            if not any_progress:
                return

    def _v7_post_limit(self) -> int:
        c = len(self.cleared)
        if c >= 14:
            return 1
        if c >= 12:
            return 2
        return 3

    def _v7_soft_cap(self) -> float:
        c = max(1, len(self.cleared))
        return min(V7_ABSOLUTE_VIRTUAL_CAP,
                   V7_TARGET_AVG_SOFT * c + V7_SOFT_CREDIT_S)

    def run_v7_post_rescue(self):
        self.v7_post_start_vt = self.api.virtual_time
        limit = min(V7_POST_RESCUE_MAX_PROBES, self._v7_post_limit())
        candidates = self.v4_candidate_pool()
        empty = 0

        print(f"\n[V7-POST] 最后保险补盲：cleared={len(self.cleared)} "
              f"unknown={len(self._unknown_channels())} limit={limit}")

        while (self.v7_post_probe_count < limit
               and candidates and self.api.enough_real_time(50)):

            spent = self.api.virtual_time - self.v7_post_start_vt
            if spent >= V7_POST_HARD_EXTRA_S:
                break
            if self.api.virtual_time >= V7_ABSOLUTE_VIRTUAL_CAP:
                break

            p, est_cost, pred = self._v6_choose_probe(candidates)
            if p is None:
                break

            if self.v7_post_probe_count >= 1:
                soft = self._v7_soft_cap()
                if self.api.virtual_time + est_cost > soft + 80.0:
                    print(f"[V7-POST] 下一点预计超软预算({soft:.0f}s)，停止")
                    break

            candidates.remove(p)
            before_disc = len(self.discovered)
            before_clear = len(self.cleared)

            self.v7_post_probe_count += 1
            print(f"[V7-POST-PROBE] #{self.v7_post_probe_count}/{limit} "
                  f"p=({p[0]:.0f},{p[1]:.0f}) est={est_cost:.0f}s "
                  f"pred_cov={pred[0]:.3f}/{pred[1]:.3f}/{pred[2]:.3f}")

            self.scan_unknown_at(p, label="V7-POST")
            self._force_pending_reobserve_at_hub(p, max_count=7)

            new_n = max(0, len(self.discovered) - before_disc)
            if new_n:
                self.v7_post_new_channels += new_n
                empty = 0
            else:
                empty += 1

            self.process_all_discovered()
            self.v7_post_new_clears += max(0, len(self.cleared) - before_clear)

            if new_n > 0 or len(self.cleared) > before_clear:
                candidates = self.v4_candidate_pool()

            mean, q10, mn = self.directional_coverage_stats()
            avg = self.api.virtual_time / max(1, len(self.cleared))
            print(f"[V7-POST SCORE] cleared={len(self.cleared)} avg={avg:.1f}s "
                  f"empty={empty} cov={mean:.3f}/{q10:.3f}/{mn:.3f}")

            if self.v7_post_probe_count >= 2 and empty >= 2:
                break

    def solve(self):
        print("\n[V7] 路线整合版：主环 -> 前置补盲 -> 批量定位 -> 一次性DP清除 -> 少量后置保险")
        self.run_primary_search()
        self.run_v7_pre_rescue()
        self.process_all_discovered()
        self.run_v7_post_rescue()
        self.process_all_discovered()

        avg = self.api.virtual_time / max(1, len(self.cleared))
        mean, q10, mn = self.directional_coverage_stats()
        print("\n" + "="*72)
        print(f"[DONE] 清除数量         : {len(self.cleared)}")
        print(f"[DONE] 定位清除总时间   : {self.api.virtual_time:.3f} s")
        print(f"[DONE] 平均定位清除时间 : {avg:.3f} s/源")
        print(f"[DONE] 移动总距离       : {self.api.move_distance:.1f} m")
        print(f"[DONE] V7前置/后置探针   : {self.v7_pre_probe_count}/{self.v7_post_probe_count}")
        print(f"[DONE] V7前置/后置新发现 : {self.v7_pre_new_channels}/{self.v7_post_new_channels}")
        print(f"[DONE] V7后置新清除      : {self.v7_post_new_clears}")
        print(f"[DONE] 覆盖启发式(mean/q10/min): {mean:.3f}/{q10:.3f}/{mn:.3f}")
        print("="*72)


def main_v7():
    if ROBOT_ID.startswith("<"):
        print("请先修改脚本顶部 ROBOT_ID 为当前登录模拟器的参赛队号。")
        return

    api = RobotAPI()
    solver = None
    entered = False
    try:
        api.enter()
        entered = True
        solver = Q4SolverV7(api)
        solver.solve()
    except Exception as e:
        print("[FATAL]", repr(e))
    finally:
        if entered:
            try:
                ans = api.exit()
                print("[EXIT]", ans)
            except Exception as e:
                print("[EXIT failed]", repr(e))

        cleared = len(solver.cleared) if solver is not None else api.clear_success
        summary = api.summary(cleared)

        if solver is not None:
            try:
                mean, q10, mn = solver.directional_coverage_stats()
                pre_to_post = (0.0 if solver.v7_pre_start_vt is None
                               else max(0.0, (solver.v7_post_start_vt or api.virtual_time)
                                        - solver.v7_pre_start_vt))
                post_extra = (0.0 if solver.v7_post_start_vt is None
                              else max(0.0, api.virtual_time - solver.v7_post_start_vt))
                summary.update({
                    "strategy_version": "q4_v7_route_integrated",
                    "full_scan_points": len(solver.full_scan_points),
                    "service_scan_count": solver.service_scan_count,
                    "v7_pre_probe_count": solver.v7_pre_probe_count,
                    "v7_post_probe_count": solver.v7_post_probe_count,
                    "v7_pre_new_channels": solver.v7_pre_new_channels,
                    "v7_post_new_channels": solver.v7_post_new_channels,
                    "v7_post_new_clears": solver.v7_post_new_clears,
                    "v7_pre_to_post_virtual_s": round(pre_to_post, 6),
                    "v7_post_extra_virtual_s": round(post_extra, 6),
                    "v6_short_sweep_count": solver.v6_short_sweep_count,
                    "directional_coverage_mean": round(mean, 6),
                    "directional_coverage_q10": round(q10, 6),
                    "directional_coverage_min": round(mn, 6),
                })
                with open(SUMMARY_FILE, "w", encoding="utf-8") as f:
                    json.dump(summary, f, ensure_ascii=False, indent=2)
            except Exception as e:
                print("[SUMMARY-WARN]", repr(e))

        print("\n*** V7 最终统计 ***")
        print(f"清除干扰源个数 = {summary['cleared']}")
        print(f"定位清除总时间 = {summary['virtual_time_s']:.3f} s")
        print(f"平均定位清除时间 = {summary['average_time_s_per_source']:.3f} s/源")
        print(f"移动时间估计 = {summary['move_time_s_est']:.1f} s "
              f"({summary['move_distance_m']:.0f} m / 5m·s^-1)")
        print(f"结果写入 {SUMMARY_FILE}；日志为 {LOG_FILE}")


if __name__ == "__main__":
    main_v7()
