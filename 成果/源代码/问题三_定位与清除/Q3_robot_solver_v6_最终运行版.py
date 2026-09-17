# Reviewable runtime-optimized version derived from B3_robot_solver_v6_stable.py.
# Search layout selection and conservative action gates are documented below.
# Run with: python B3_robot_solver_v6_reviewable.py --self-test

"""
2026 数学建模 B 题 - 问题 3
机器狗全向干扰源自动搜索、定位与清除程序

核心策略（V6 STABLE）
------------------
1. 全局搜索：原点 + 经过认证的环点布局。
   默认采用正七边形、半径 1000 m；只有在覆盖认证和成本门控同时通过时，
   才候选采用正六边形、半径 1135 m。两种布局都必须满足目标圆域的覆盖认证。

2. 频道剪枝：只在全局搜索点扫描“尚未发现”的频道。
   一旦某频道测得 direction/near，就不再参加后续全局盲扫，减少 5 s 检测与 1 s 切频开销。

3. 有界误差集合定位：
   每个 direction 测量对应 ±1°（程序取 1.02° 留舍入裕量）的楔形可行域；
   同时目标一定在半径 1800 m 目标圆内，且每次成功测向点到目标距离 <= 1500 m。
   将这些约束求交，得到凸多边形可行域 P。

4. 谨慎贪心二次测量：
   只有一个测向时，从首测点沿“测向方向 ±30°”移动 300 m，
   选择离机器狗当前位置更近的一侧。该点仍能保证处于该全向源接收范围内，
   同时形成非共线基线，提高交会定位精度。

5. 在线收缩：
   若可行域包络半径 > 18 m，则优先前往可行域中心再测一次；
   若包络过大，则再做一次侧向测量。每次新测量后重新求交，直到可安全清除。

6. 路线规划：
   对当前已发现、未清除目标的“估计位置”做 Held-Karp 动态规划，
   并把下一个全局搜索点作为终点代价加入，选择总代价最小的第一个目标。
   目标过多时自动降级为最近邻 + 终点方向惩罚，避免指数爆炸。

说明
----
- 仅使用 Python 标准库，无需 pip 安装第三方包。
- 必须先在模拟器中登录并启动“问题 3”测试，等接口 ready 后再运行本程序。
- 修改 ROBOT_ID，或命令行传入 --robot-id。
- 正式测试前务必先大量演练，并根据日志微调参数。
"""
from __future__ import annotations
import argparse
import json
import math
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
BASE_URL = 'http://127.0.0.1:2026'
ROBOT_ID = '<参赛队号>'
AREA_R = 1800.0
MIN_RECV_R = 1000.0
MAX_RECV_R = 1500.0
CLEAR_R = 20.0
NEAR_R = 5.0
BEARING_ERR_DEG = 1.02
SURVEY_N = 7
SURVEY_COVER_R = 999.0
COMPACT_SURVEY_N = 6
COMPACT_SURVEY_R = 1135.0
MOVE_SPEED_MPS = 5.0
MEASURE_SCAN_COST_S = 6.0
LAYOUT_SWITCH_MARGIN_S = 5.0
SAFE_CENTER_PROBE_DIST = 1200.0

def _survey_ring_radius(n: int, cover_r: float) -> float:
    a = math.pi / n
    c = math.cos(a)
    disc = (AREA_R * c) ** 2 - (AREA_R ** 2 - cover_r ** 2)
    if disc <= 0:
        raise ValueError('搜索环参数无法覆盖目标圆域')
    return AREA_R * c - math.sqrt(disc)
SURVEY_R = 1000.0
SECOND_BASELINE = 300.0
SECOND_OFFSET_DEG = 30.0
SAFE_SECOND_FORWARD = 600.0
SAFE_SECOND_LATERAL = 250.0
OPPORTUNISTIC_PREDICT_DIST = 1350.0
OPPORTUNISTIC_MIN_BASELINE = 180.0
OPPORTUNISTIC_MAX_PER_POINT = 20
OPPORTUNISTIC_HARD_CAP = 8
OPPORTUNISTIC_MAX_FEASIBLE_DIST = 1400.0
SEARCH_CLEAR_DETOUR_MAX = 100.0
SEARCH_CLEAR_MAX_PER_LEG = 2
COVER_CERT_R = 999.0
SCAN_EQUIV_M_PER_UNKNOWN = 30.0
SERVICE_COVER_EXTRA_MAX = 450.0
LONG_EDGE_SOFT = 1250.0
LONG_EDGE_PENALTY = 0.2
DIRECT_CLEAR_BOUND = 19.0
PROBE_CLEAR_BOUND = 950.0
SAFE_CENTER_MEASURE_BOUND = 950.0
MAX_LOCALIZE_OBS = 7
DP_MAX_TARGETS = 11
HTTP_TIMEOUT = 5.0
HTTP_RETRIES = 4
RETRY_SLEEP = 0.2
LOCAL_LOG = 'B3_strategy_log_v6_stable.jsonl'
Point = Tuple[float, float]

def distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])

def unit_from_deg(deg: float) -> Point:
    a = math.radians(deg)
    return (math.cos(a), math.sin(a))

def normalize_deg(deg: float) -> float:
    return deg % 360.0

def angle_diff_deg(a: float, b: float) -> float:
    """返回 a-b 归一化到 [-180,180)。"""
    return (a - b + 180.0) % 360.0 - 180.0

def add_point(a: Point, b: Point) -> Point:
    return (a[0] + b[0], a[1] + b[1])

def scale_point(v: Point, s: float) -> Point:
    return (v[0] * s, v[1] * s)

def point_along(p: Point, bearing_deg: float, length: float) -> Point:
    u = unit_from_deg(bearing_deg)
    return (p[0] + length * u[0], p[1] + length * u[1])

def clip_halfplane(poly: List[Point], a: float, b: float, c: float, eps: float=1e-09) -> List[Point]:
    """
    Sutherland-Hodgman 裁剪。
    保留满足 a*x + b*y + c >= 0 的半平面。
    """
    if not poly:
        return []

    def f(p: Point) -> float:
        return a * p[0] + b * p[1] + c
    out: List[Point] = []
    prev = poly[-1]
    f_prev = f(prev)
    prev_in = f_prev >= -eps
    for cur in poly:
        f_cur = f(cur)
        cur_in = f_cur >= -eps
        if cur_in != prev_in:
            den = f_prev - f_cur
            if abs(den) > 1e-15:
                t = f_prev / den
                inter = (prev[0] + t * (cur[0] - prev[0]), prev[1] + t * (cur[1] - prev[1]))
                out.append(inter)
        if cur_in:
            out.append(cur)
        prev = cur
        f_prev = f_cur
        prev_in = cur_in
    return out

def circumscribed_circle_polygon(center: Point, radius: float, n: int=96) -> List[Point]:
    """
    返回包含真实圆的正 n 边形（外接多边形），保证不会把真实目标误裁掉。
    """
    rv = radius / math.cos(math.pi / n)
    cx, cy = center
    return [(cx + rv * math.cos((2 * k + 1) * math.pi / n), cy + rv * math.sin((2 * k + 1) * math.pi / n)) for k in range(n)]

def clip_by_outer_circle(poly: List[Point], center: Point, radius: float, n: int=48) -> List[Point]:
    """
    用圆的 n 个切线半平面构成外接多边形，保守地施加 |x-center| <= radius 约束。
    """
    cx, cy = center
    for k in range(n):
        ang = 2.0 * math.pi * k / n
        nx, ny = (math.cos(ang), math.sin(ang))
        poly = clip_halfplane(poly, -nx, -ny, radius + nx * cx + ny * cy)
        if not poly:
            break
    return poly

def clip_by_bearing_wedge(poly: List[Point], sensor: Point, measured_deg: float, half_width_deg: float=BEARING_ERR_DEG) -> List[Point]:
    """
    真方位位于 [measured-half_width, measured+half_width]。
    对小于 180° 的楔形，等价为两半平面求交。
    """
    px, py = sensor
    lo = math.radians(measured_deg - half_width_deg)
    dx, dy = (math.cos(lo), math.sin(lo))
    poly = clip_halfplane(poly, -dy, dx, dy * px - dx * py)
    if not poly:
        return []
    hi = math.radians(measured_deg + half_width_deg)
    dx, dy = (math.cos(hi), math.sin(hi))
    poly = clip_halfplane(poly, dy, -dx, -dy * px + dx * py)
    return poly

def polygon_centroid(poly: Sequence[Point]) -> Point:
    if not poly:
        raise ValueError('empty polygon')
    area2 = 0.0
    cx_num = 0.0
    cy_num = 0.0
    for i, p in enumerate(poly):
        q = poly[(i + 1) % len(poly)]
        cr = p[0] * q[1] - q[0] * p[1]
        area2 += cr
        cx_num += (p[0] + q[0]) * cr
        cy_num += (p[1] + q[1]) * cr
    if abs(area2) < 1e-10:
        return (sum((p[0] for p in poly)) / len(poly), sum((p[1] for p in poly)) / len(poly))
    return (cx_num / (3.0 * area2), cy_num / (3.0 * area2))

def polygon_bound(poly: Sequence[Point]) -> Tuple[Point, float]:
    """
    用多边形面积重心作为代表点，最大顶点距作为保守包络半径。
    对凸多边形，点到固定中心的最大值出现在顶点。
    """
    c = polygon_centroid(poly)
    r = max((distance(c, p) for p in poly))
    try:
        mec_center, mec_radius = minimum_enclosing_circle(poly)
        if _point_in_convex_polygon(mec_center, poly):
            return (mec_center, mec_radius)
    except (ArithmeticError, ValueError):
        pass
    return (c, r)

def _circumcenter(a: Point, b: Point, c: Point) -> Optional[Point]:
    ax, ay = a
    bx, by = b
    cx, cy = c
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-10:
        return None
    aa = ax * ax + ay * ay
    bb = bx * bx + by * by
    cc = cx * cx + cy * cy
    ux = (aa * (by - cy) + bb * (cy - ay) + cc * (ay - by)) / d
    uy = (aa * (cx - bx) + bb * (ax - cx) + cc * (bx - ax)) / d
    return (ux, uy)

def minimum_enclosing_circle(points: Sequence[Point]) -> Tuple[Point, float]:
    """用确定性增量算法求点集的最小包围圆。"""
    pts = list(points)
    if not pts:
        raise ValueError('empty point set')
    center: Optional[Point] = None
    radius = -1.0

    def contains(p: Point) -> bool:
        return center is not None and distance(center, p) <= radius + 1e-7

    for i, p in enumerate(pts):
        if contains(p):
            continue
        center, radius = p, 0.0
        for j in range(i):
            q = pts[j]
            if contains(q):
                continue
            center = ((p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0)
            radius = distance(p, q) / 2.0
            for k in range(j):
                r = pts[k]
                if contains(r):
                    continue
                circum = _circumcenter(p, q, r)
                if circum is None:
                    continue
                center = circum
                radius = distance(center, p)
    if center is None or any(distance(center, p) > radius + 1e-6 for p in pts):
        raise ArithmeticError('无法构造包含全部顶点的包围圆')
    return (center, radius)

def _point_in_convex_polygon(point: Point, poly: Sequence[Point]) -> bool:
    """判断点是否在凸多边形内（允许落在边界上）。"""
    if not poly:
        return False
    signs = []
    for i, a in enumerate(poly):
        b = poly[(i + 1) % len(poly)]
        cross = (b[0] - a[0]) * (point[1] - a[1]) - (b[1] - a[1]) * (point[0] - a[0])
        if abs(cross) > 1e-7:
            signs.append(cross > 0.0)
    return not signs or all(s == signs[0] for s in signs)

def _boundary_bisector_points(a: Point, b: Point, radius: float=AREA_R) -> List[Point]:
    """
    大圆 |x|=radius 与两点等距线 |x-a|=|x-b| 的交点。
    """
    vx = b[0] - a[0]
    vy = b[1] - a[1]
    vn = math.hypot(vx, vy)
    if vn < 1e-12:
        return []
    k = (b[0] * b[0] + b[1] * b[1] - a[0] * a[0] - a[1] * a[1]) / 2.0
    nx, ny = (vx / vn, vy / vn)
    s = k / vn
    if abs(s) > radius + 1e-09:
        return []
    h = math.sqrt(max(0.0, radius * radius - s * s))
    tx, ty = (-ny, nx)
    bx0, by0 = (nx * s, ny * s)
    return [(bx0 + tx * h, by0 + ty * h), (bx0 - tx * h, by0 - ty * h)]

def coverage_radius(scan_points: Sequence[Point]) -> Tuple[float, Point]:
    """
    计算目标圆域内“到最近完整扫描点的最大距离”。

    最近点距离函数的最大值只可能出现在：
    1) Voronoi 顶点（任意三扫描点的外心，且落在目标圆内）；
    2) Voronoi 边与目标圆边界的交点；
    3) 边界上单个距离函数的驻点（径向正/反方向）。
    扫描点数量很少，因此直接枚举即可，避免网格近似破坏“必找全”保证。
    """
    pts = list(scan_points)
    if not pts:
        return (AREA_R, (0.0, 0.0))
    candidates: List[Point] = [(0.0, 0.0)]
    m = len(pts)
    candidates.extend([(AREA_R, 0.0), (-AREA_R, 0.0), (0.0, AREA_R), (0.0, -AREA_R)])
    for p in pts:
        r = math.hypot(p[0], p[1])
        if r > 1e-12:
            candidates.append((AREA_R * p[0] / r, AREA_R * p[1] / r))
            candidates.append((-AREA_R * p[0] / r, -AREA_R * p[1] / r))
    for i in range(m):
        for j in range(i + 1, m):
            candidates.extend(_boundary_bisector_points(pts[i], pts[j], AREA_R))
    rr = AREA_R * AREA_R
    for i in range(m):
        for j in range(i + 1, m):
            for k in range(j + 1, m):
                cc = _circumcenter(pts[i], pts[j], pts[k])
                if cc is not None and cc[0] * cc[0] + cc[1] * cc[1] <= rr + 1e-06:
                    candidates.append(cc)
    best_d = -1.0
    best_p = (0.0, 0.0)
    for q in candidates:
        if q[0] * q[0] + q[1] * q[1] > rr + 1e-05:
            continue
        d = min((distance(q, p) for p in pts))
        if d > best_d:
            best_d = d
            best_p = q
    return (best_d, best_p)

def _coverage_veto_sample(scan_points: Sequence[Point]) -> float:
    """
    独立保守复核：在目标圆的多层极坐标网格上计算最大最近扫描点距离。
    这不是覆盖证明本身，只作为“否决器”：
    只要采样发现 > 阈值，就绝不允许提前结束。
    """
    pts = list(scan_points)
    if not pts:
        return float('inf')
    worst = 0.0
    for frac in (0.0, 0.25, 0.5, 0.75, 0.9, 1.0):
        rr = AREA_R * frac
        n_ang = 1 if rr == 0.0 else 720
        for k in range(n_ang):
            a = 2.0 * math.pi * k / n_ang
            q = (rr * math.cos(a), rr * math.sin(a))
            d = min((distance(q, p) for p in pts))
            if d > worst:
                worst = d
    return worst

def coverage_complete(scan_points: Sequence[Point]) -> bool:
    unique = []
    for p in scan_points:
        if all((distance(p, q) > 1.0 for q in unique)):
            unique.append(p)
    if len(unique) < 5:
        return False
    d_exact, _ = coverage_radius(unique)
    if d_exact > COVER_CERT_R:
        return False
    d_sample = _coverage_veto_sample(unique)
    if d_sample > COVER_CERT_R:
        return False
    return True

def build_survey_points(n: int, radius: float) -> List[Point]:
    """构造“原点 + 正 n 边形环点”的全局搜索点集合。"""
    if n < 3 or radius <= 0.0:
        raise ValueError('搜索环参数必须满足 n>=3 且 radius>0')
    return [(0.0, 0.0)] + [
        (
            radius * math.cos(2.0 * math.pi * k / n),
            radius * math.sin(2.0 * math.pi * k / n),
        )
        for k in range(n)
    ]

def layout_is_certified(scan_points: Sequence[Point]) -> bool:
    """只有同时通过精确覆盖和采样否决器，布局才可用于全局搜索。"""
    return coverage_complete(scan_points)

def survey_path_length(n: int, radius: float) -> float:
    """从原点出发连续访问正 n 边形全部环点的最短固定路线长度。"""
    return radius + (n - 1) * 2.0 * radius * math.sin(math.pi / n)

def should_use_compact_layout(extra_move_s: float, unknown_n: int) -> bool:
    """成本门控：扫描节省必须严格覆盖额外移动并留出安全裕量。"""
    if unknown_n <= 0 or not math.isfinite(extra_move_s):
        return False
    scan_saving_s = MEASURE_SCAN_COST_S * float(unknown_n)
    return scan_saving_s - extra_move_s >= LAYOUT_SWITCH_MARGIN_S

def _shortest_open_path(start: Point, pts: Sequence[Point]) -> Tuple[float, List[int], float]:
    """
    Held-Karp 求从 start 出发访问全部 pts、无需返回的最短开放路径。
    同时返回最大单段长度，用于稳定性软惩罚。
    """
    n = len(pts)
    if n == 0:
        return (0.0, [], 0.0)
    if n == 1:
        d = distance(start, pts[0])
        return (d, [0], d)
    dp = {}
    for j in range(n):
        d = distance(start, pts[j])
        dp[1 << j, j] = (d, (j,), d)
    full = (1 << n) - 1
    for mask in range(1, full + 1):
        for j in range(n):
            st = dp.get((mask, j))
            if st is None:
                continue
            cost, route, mx = st
            for k in range(n):
                bit = 1 << k
                if mask & bit:
                    continue
                e = distance(pts[j], pts[k])
                nc = cost + e
                nm = mask | bit
                key = (nm, k)
                old = dp.get(key)
                if old is None or nc < old[0] - 1e-09 or (abs(nc - old[0]) <= 1e-09 and max(mx, e) < old[2]):
                    dp[key] = (nc, route + (k,), max(mx, e))
    best = None
    for j in range(n):
        st = dp[full, j]
        score = st[0] + LONG_EDGE_PENALTY * max(0.0, st[2] - LONG_EDGE_SOFT)
        if best is None or score < best[0]:
            best = (score, st)
    assert best is not None
    cost, route, mx = best[1]
    return (cost, list(route), mx)

class ApiError(RuntimeError):
    pass

class RobotAPI:

    def __init__(self, base_url: str, robot_id: str, log_path: str=LOCAL_LOG) -> None:
        self.base_url = base_url.rstrip('/')
        self.robot_id = robot_id
        self.seq = 0
        self.position: Point = (0.0, 0.0)
        self.current_channel = 1
        self.virtual_time_s = 0.0
        self.remaining_real_duration_s: Optional[float] = None
        self.log_path = Path(log_path)
        if self.log_path.exists() and self.log_path.stat().st_size > 0:
            stamp = time.strftime('%Y%m%d-%H%M%S')
            self.log_path = self.log_path.with_name(f'{self.log_path.stem}_{stamp}_{uuid.uuid4().hex[:4]}{self.log_path.suffix}')
        self.log_path.write_text('', encoding='utf-8')

    def _new_request_id(self, prefix: str) -> str:
        self.seq += 1
        return f'{prefix}-{self.seq:05d}-{uuid.uuid4().hex[:8]}'

    def _base(self, request_id: str) -> dict:
        return {'arena_id': 'default', 'robot_id': self.robot_id, 'request_id': request_id}

    def _write_log(self, record: dict) -> None:
        with self.log_path.open('a', encoding='utf-8') as f:
            f.write(json.dumps(record, ensure_ascii=False) + '\n')

    def _post_same_payload(self, path: str, payload: dict) -> dict:
        """
        网络异常重试时严格复用同一 payload / request_id，符合幂等要求。
        """
        data = json.dumps(payload, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
        last_error: Optional[BaseException] = None
        for attempt in range(HTTP_RETRIES):
            req = Request(self.base_url + path, data=data, headers={'Content-Type': 'application/json'}, method='POST')
            try:
                with urlopen(req, timeout=HTTP_TIMEOUT) as resp:
                    body = resp.read().decode('utf-8')
                    result = json.loads(body)
                self._write_log({'wall_time': time.time(), 'path': path, 'payload': payload, 'response': result})
                return result
            except HTTPError as e:
                try:
                    body = e.read().decode('utf-8', errors='replace')
                except Exception:
                    body = ''
                last_error = ApiError(f'HTTP {e.code}: {body}')
                if 400 <= e.code < 500 and e.code not in (408, 429):
                    raise last_error
            except (URLError, TimeoutError, OSError, json.JSONDecodeError) as e:
                last_error = e
            if attempt + 1 < HTTP_RETRIES:
                time.sleep(RETRY_SLEEP * (attempt + 1))
        raise ApiError(f'{path} 请求失败，已重试 {HTTP_RETRIES} 次：{last_error}')

    @staticmethod
    def _ensure_accepted(resp: dict, path: str) -> None:
        if resp.get('accepted') is not True:
            raise ApiError(f'{path} accepted != true: {resp}')

    def enter(self) -> dict:
        rid = self._new_request_id('enter')
        resp = self._post_same_payload('/enter', self._base(rid))
        self._ensure_accepted(resp, '/enter')
        self.virtual_time_s = float(resp.get('virtual_time_s', 0.0))
        self.remaining_real_duration_s = float(resp.get('remaining_real_duration_s', 0.0))
        self.position = (0.0, 0.0)
        self.current_channel = 1
        return resp

    def measure(self, p: Point, channel: int) -> dict:
        rid = self._new_request_id('measure')
        payload = self._base(rid)
        payload['position'] = {'x': float(p[0]), 'y': float(p[1])}
        payload['channel'] = int(channel)
        resp = self._post_same_payload('/measure', payload)
        self._ensure_accepted(resp, '/measure')
        self.position = (float(p[0]), float(p[1]))
        self.current_channel = int(channel)
        self.virtual_time_s = float(resp['virtual_time_s'])
        return resp

    def clear(self, p: Point, channel: int) -> dict:
        rid = self._new_request_id('clear')
        payload = self._base(rid)
        payload['position'] = {'x': float(p[0]), 'y': float(p[1])}
        payload['channel'] = int(channel)
        resp = self._post_same_payload('/clear', payload)
        self._ensure_accepted(resp, '/clear')
        self.position = (float(p[0]), float(p[1]))
        self.virtual_time_s = float(resp['virtual_time_s'])
        return resp

    def exit(self) -> dict:
        rid = self._new_request_id('exit')
        resp = self._post_same_payload('/exit', self._base(rid))
        self._ensure_accepted(resp, '/exit')
        self.virtual_time_s = float(resp.get('virtual_time_s', self.virtual_time_s))
        return resp

@dataclass
class Observation:
    point: Point
    bearing_deg: float

@dataclass
class TargetTask:
    channel: int
    observations: List[Observation] = field(default_factory=list)
    cleared: bool = False

    def add_observation(self, p: Point, bearing_deg: float) -> None:
        self.observations.append(Observation(p, normalize_deg(bearing_deg)))

    def feasible_polygon(self) -> List[Point]:
        """
        目标约束：
        - 在半径 1800 m 的全局目标圆内；
        - 对每次 direction 测量，在 ±1.02° 楔形内；
        - 成功收到信号 => 目标距该检测点 <= 1500 m。
        """
        poly = circumscribed_circle_polygon((0.0, 0.0), AREA_R, n=96)
        for ob in self.observations:
            poly = clip_by_bearing_wedge(poly, ob.point, ob.bearing_deg, BEARING_ERR_DEG)
            if not poly:
                return []
            poly = clip_by_outer_circle(poly, ob.point, MAX_RECV_R, n=48)
            if not poly:
                return []
        return poly

    def representative(self) -> Tuple[Point, float]:
        poly = self.feasible_polygon()
        if not poly:
            if not self.observations:
                return ((0.0, 0.0), float('inf'))
            ob = self.observations[-1]
            return (point_along(ob.point, ob.bearing_deg, 700.0), 800.0)
        return polygon_bound(poly)

def _route_length(start: Point, points: Sequence[Point], route: Sequence[int]) -> float:
    if not route:
        return 0.0
    total = distance(start, points[route[0]])
    for a, b in zip(route, route[1:]):
        total += distance(points[a], points[b])
    return total

def _two_opt_open(start: Point, points: Sequence[Point], route: List[int]) -> List[int]:
    """固定起点、终点自由的 open-path 2-opt。n<=20 时非常快。"""
    n = len(route)
    if n < 3:
        return route
    improved = True
    rounds = 0
    while improved and rounds < 30:
        improved = False
        rounds += 1
        for i in range(n - 1):
            prev_p = start if i == 0 else points[route[i - 1]]
            a = points[route[i]]
            for j in range(i + 1, n):
                b = points[route[j]]
                old = distance(prev_p, a)
                new = distance(prev_p, b)
                if j + 1 < n:
                    nxt = points[route[j + 1]]
                    old += distance(b, nxt)
                    new += distance(a, nxt)
                if new + 1e-09 < old:
                    route[i:j + 1] = reversed(route[i:j + 1])
                    improved = True
        if not improved:
            base = _route_length(start, points, route)
            for i in range(n):
                node = route[i]
                rest = route[:i] + route[i + 1:]
                for j in range(len(rest) + 1):
                    cand = rest[:j] + [node] + rest[j:]
                    val = _route_length(start, points, cand)
                    if val + 1e-09 < base:
                        route = cand
                        improved = True
                        break
                if improved:
                    break
    return route

def _heuristic_open_route(start: Point, points: Sequence[Point]) -> List[int]:
    """
    多启动最近邻 + 2-opt。V2 在目标数>12时直接降级为单步最近邻，
    这是 15/16 目标案例中路线抖动的一个重要来源；V3 用这个近似 TSP。
    """
    n = len(points)
    if n <= 1:
        return list(range(n))
    seeds = list(range(n))
    best_route: Optional[List[int]] = None
    best_len = float('inf')
    for first in seeds:
        route = [first]
        unused = set(range(n))
        unused.remove(first)
        cur = points[first]
        while unused:
            k = min(unused, key=lambda j: distance(cur, points[j]))
            route.append(k)
            unused.remove(k)
            cur = points[k]
        route = _two_opt_open(start, points, route)
        val = _route_length(start, points, route)
        if val < best_len:
            best_len = val
            best_route = route
    assert best_route is not None
    return best_route

def _exact_open_route(start: Point, points: Sequence[Point]) -> List[int]:
    """Held-Karp 开放路径：固定 start，不要求回到 start。"""
    n = len(points)
    if n <= 1:
        return list(range(n))
    dp: Dict[Tuple[int, int], float] = {}
    parent: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {}
    for j, p in enumerate(points):
        dp[1 << j, j] = distance(start, p)
        parent[1 << j, j] = None
    full = (1 << n) - 1
    for mask in range(1, full + 1):
        for j in range(n):
            key = (mask, j)
            if key not in dp:
                continue
            cost = dp[key]
            rem = full ^ mask
            while rem:
                bit = rem & -rem
                k = bit.bit_length() - 1
                nkey = (mask | bit, k)
                nc = cost + distance(points[j], points[k])
                if nc < dp.get(nkey, float('inf')):
                    dp[nkey] = nc
                    parent[nkey] = key
                rem -= bit
    last = min(range(n), key=lambda j: dp[full, j])
    route: List[int] = []
    state: Optional[Tuple[int, int]] = (full, last)
    while state is not None:
        route.append(state[1])
        state = parent[state]
    route.reverse()
    return route

def _route_plan(start: Point, points: Sequence[Point]) -> List[int]:
    if len(points) <= DP_MAX_TARGETS:
        return _exact_open_route(start, points)
    return _heuristic_open_route(start, points)

def _route_dp_first(start: Point, ids: List[int], points: List[Point], terminal: Optional[Point]=None) -> int:
    """兼容旧调用；V4 实际统一交给 _route_plan。"""
    if not ids:
        raise ValueError('empty ids')
    if terminal is not None:
        points2 = list(points)
        route = _route_plan(start, points2)
    else:
        route = _route_plan(start, points)
    return ids[route[0]]

class B3Solver:
    """
    V3 的关键变化：
    1) 原点 + 经过认证的环点完成全域覆盖，默认保留七点安全布局；
    2) 搜索点原地尽量补第二个示向，减少主动定位移动；
    3) 搜索结束后按“下一真实动作点”做全局开放TSP，并每次动作后重规划；
    4) 单示向采用 600m 前进 + 250m 横移的保证可接收二次测点。
    """

    def __init__(self, api: RobotAPI) -> None:
        self.api = api
        self.unknown_channels = set(range(1, 21))
        self.tasks: Dict[int, TargetTask] = {}
        self.found_channels: set[int] = set()
        self.cleared_channels: set[int] = set()
        self.ring_points: List[Point] = [
            (SURVEY_R * math.cos(2.0 * math.pi * k / SURVEY_N), SURVEY_R * math.sin(2.0 * math.pi * k / SURVEY_N))
            for k in range(SURVEY_N)
        ]
        self.layout_name = 'seven-point-safe'
        self.scan_points: List[Point] = []
        self.remaining_anchors = set(range(SURVEY_N))
        self._fusion_attempted: set[int] = set()

    def status(self, msg: str) -> None:
        print(f'[t={self.api.virtual_time_s:9.2f}s] pos=({self.api.position[0]:8.1f},{self.api.position[1]:8.1f}) found={len(self.found_channels):2d} cleared={len(self.cleared_channels):2d} unknown={len(self.unknown_channels):2d} | {msg}')

    def rough_point(self, task: TargetTask) -> Point:
        """用于路线规划的目标粗估点。"""
        if len(task.observations) >= 2:
            poly = task.feasible_polygon()
            if poly:
                return polygon_centroid(poly)
        if not task.observations:
            return (0.0, 0.0)
        ob = task.observations[0]
        return point_along(ob.point, ob.bearing_deg, 800.0)

    def task_bound(self, task: TargetTask) -> float:
        poly = task.feasible_polygon()
        if not poly:
            return float('inf')
        _, bound = polygon_bound(poly)
        return bound

    def _scan_order(self) -> List[int]:
        chans = sorted(self.unknown_channels)
        if self.api.current_channel in self.unknown_channels:
            cur = self.api.current_channel
            chans.remove(cur)
            chans.insert(0, cur)
        return chans

    def _accept_measurement(self, ch: int, p: Point, resp: dict) -> None:
        result = resp['measure_result']
        if result == 'no_signal':
            return
        self.found_channels.add(ch)
        self.unknown_channels.discard(ch)
        if result == 'near':
            clr = self.api.clear(p, ch)
            if clr['clear_result'] == 'success':
                self.cleared_channels.add(ch)
                self.tasks.pop(ch, None)
                self.status(f'频道 {ch} 在搜索点 near，直接清除成功')
                return
            self.tasks.setdefault(ch, TargetTask(ch))
            return
        if result == 'direction':
            task = self.tasks.get(ch)
            if task is None:
                task = TargetTask(ch)
                self.tasks[ch] = task
            task.add_observation(p, float(resp['svd_deg']))
            return
        raise ApiError(f'未知 measure_result: {result}')

    def scan_unknown_at(self, p: Point) -> None:
        self.status(f'全局搜索点扫描 {p}')
        for ch in list(self._scan_order()):
            if len(self.found_channels) >= 16:
                self.status('已发现16个频道，立即停止当前搜索点剩余频道扫描')
                break
            if ch not in self.unknown_channels:
                continue
            resp = self.api.measure(p, ch)
            before = ch in self.found_channels
            self._accept_measurement(ch, p, resp)
            if not before and ch in self.found_channels:
                if resp['measure_result'] == 'direction':
                    self.status(f'发现频道 {ch}，示向度 {resp['svd_deg']:.2f}°')

    def coverage_scan_at(self, p: Point, with_reobserve: bool=False) -> None:
        """
        在 p 对所有尚未发现频道做一次完整扫描，并把 p 记入覆盖证据。
        同一点不会重复加入覆盖集合。
        """
        self.scan_unknown_at(p)
        if with_reobserve:
            self.opportunistic_reobserve(p)
        if all((distance(p, q) > 1.0 for q in self.scan_points)):
            self.scan_points.append(p)
        rad, witness = coverage_radius(self.scan_points)
        self.status(f'覆盖认证：扫描点={len(self.scan_points)}，最坏未覆盖距离≈{rad:.1f}m，见证点≈({witness[0]:.0f},{witness[1]:.0f})')

    def select_search_layout(self) -> None:
        """原点扫描后，在认证和成本约束下选择六点或七点环布局。"""
        safe_points = build_survey_points(SURVEY_N, SURVEY_R)
        compact_points = build_survey_points(COMPACT_SURVEY_N, COMPACT_SURVEY_R)
        compact_certified = layout_is_certified(compact_points)
        extra_move_s = (
            survey_path_length(COMPACT_SURVEY_N, COMPACT_SURVEY_R)
            - survey_path_length(SURVEY_N, SURVEY_R)
        ) / MOVE_SPEED_MPS
        if compact_certified and should_use_compact_layout(extra_move_s, len(self.unknown_channels)):
            self.ring_points = compact_points[1:]
            self.remaining_anchors = set(range(COMPACT_SURVEY_N))
            self.layout_name = 'six-point-compact'
            self.status(
                f'布局门控选择六点环：认证通过，额外移动≈{extra_move_s:.1f}s，'
                f'未知频道={len(self.unknown_channels)}'
            )
            return
        self.ring_points = safe_points[1:]
        self.remaining_anchors = set(range(SURVEY_N))
        self.layout_name = 'seven-point-safe'
        self.status(
            f'布局门控保留七点环：六点认证={compact_certified}，'
            f'额外移动≈{extra_move_s:.1f}s，未知频道={len(self.unknown_channels)}'
        )

    def _best_anchor_plan(self, scan_points: Optional[Sequence[Point]]=None, remaining: Optional[Sequence[int]]=None, start: Optional[Point]=None) -> Tuple[float, float, List[int], float]:
        """
        从剩余固定环点中选择一个“足以完成覆盖认证”的最优子集。

        目标函数同时考虑：
          - 实际移动距离；
          - 每多一个完整扫描点，需要对所有当前未知频道再测一遍的时间；
          - 超长单段移动的软惩罚。

        返回：
          (等效总代价m, 纯移动距离m, 环点访问顺序, 最大单段m)
        """
        if scan_points is None:
            scan_points = self.scan_points
        if remaining is None:
            remaining = sorted(self.remaining_anchors)
        else:
            remaining = list(remaining)
        if start is None:
            start = self.api.position
        if coverage_complete(scan_points):
            return (0.0, 0.0, [], 0.0)
        unknown_n = len(self.unknown_channels)
        best = None
        m = len(remaining)
        for mask in range(1, 1 << m):
            subset = [remaining[i] for i in range(m) if mask & 1 << i]
            centers = list(scan_points) + [self.ring_points[i] for i in subset]
            rad, _ = coverage_radius(centers)
            if rad > COVER_CERT_R:
                continue
            pts = [self.ring_points[i] for i in subset]
            path_d, order_local, mx = _shortest_open_path(start, pts)
            order = [subset[j] for j in order_local]
            scan_eq = SCAN_EQUIV_M_PER_UNKNOWN * unknown_n * len(subset)
            long_pen = LONG_EDGE_PENALTY * max(0.0, mx - LONG_EDGE_SOFT)
            obj = path_d + scan_eq + long_pen
            cand = (obj, path_d, order, mx)
            if best is None or cand[0] < best[0]:
                best = cand
        if best is None:
            pts = [self.ring_points[i] for i in remaining]
            path_d, order_local, mx = _shortest_open_path(start, pts)
            order = [remaining[j] for j in order_local]
            obj = path_d + SCAN_EQUIV_M_PER_UNKNOWN * unknown_n * len(order)
            return (obj, path_d, order, mx)
        return best

    def _service_can_replace_anchor(self) -> Optional[TargetTask]:
        """
        判断是否值得先做一个目标动作，再把该位置顺便作为未知频道扫描点。

        关键限制：候选目标动作必须让后续所需固定环点数量至少减少1。
        这样“在目标点扫未知频道”不是额外扫描，而是在替代原本必做的环点扫描，
        因而能把定位/清除移动与全局搜索移动真正合并。
        """
        if not self.tasks or not self.remaining_anchors:
            return None
        base_obj, base_path, base_order, _ = self._best_anchor_plan()
        if not base_order:
            return None
        base_k = len(base_order)
        cur = self.api.position
        best = None
        for ch, task in list(self.tasks.items()):
            if ch in self._fusion_attempted:
                continue
            p, action_kind, bound = self.task_action_point(task)
            if action_kind == 'fallback':
                continue
            if distance(p, (0.0, 0.0)) > AREA_R + 1e-6:
                continue
            if any(distance(p, q) <= 1.0 for q in self.scan_points):
                continue
            sim_scan = list(self.scan_points) + [p]
            cand_obj, cand_path, cand_order, _ = self._best_anchor_plan(scan_points=sim_scan, remaining=sorted(self.remaining_anchors), start=p)
            saved = base_k - len(cand_order)
            if saved < 1:
                continue
            extra_path = distance(cur, p) + cand_path - base_path
            if extra_path > SERVICE_COVER_EXTRA_MAX:
                continue
            score = -700.0 * saved + max(0.0, extra_path) + 0.15 * cand_obj + 0.02 * (0.0 if math.isinf(bound) else bound)
            cand = (score, -saved, extra_path, ch, task, p, action_kind)
            if best is None or cand < best:
                best = cand
        if best is None:
            return None
        _, neg_saved, extra_path, ch, task, p, action_kind = best
        self.status(f'频道 {ch} 的下一动作 {action_kind} 可替代 {-neg_saved} 个固定搜索点，相对纯搜索路径增量≈{extra_path:.1f}m，执行搜索-定位融合')
        return task

    def adaptive_search(self) -> None:
        """
        V6 稳定搜索：

        1) 原点完整扫描；
        2) 根据剩余未知频道数进行六/七点布局成本门控；
        3) 根据原点已发现目标选择当前环布局的最佳起点/方向；
        4) 若目标动作点能够严格替代环点，则先做一次搜索—定位融合；
        5) 沿剩余环点完成固定覆盖；
        6) 每个环点仍允许原地补第二示向；
        7) 只允许 <= SEARCH_CLEAR_DETOUR_MAX 的便宜顺路清除。

        这样彻底去掉 V5 中导致路径波动和覆盖判定复杂化的自适应替点，
        但保留目标感知的搜索终点优化与严格覆盖认证。
        """
        self.coverage_scan_at((0.0, 0.0), with_reobserve=False)
        if len(self.found_channels) >= 16:
            self.status('原点已发现16个干扰源，达到数量上限')
            return
        self.select_search_layout()
        tour = self.choose_ring_tour()
        self.status(f'V6 认证布局 {self.layout_name} 搜索顺序: {tour}')
        tour_pos = 0
        while tour_pos < len(tour):
            if len(self.found_channels) >= 16:
                self.status('已发现16个干扰源，达到题目数量上限，提前结束全局搜索')
                break
            fusion_task = self._service_can_replace_anchor()
            if fusion_task is not None:
                fusion_channel = fusion_task.channel
                fusion_point, _, _ = self.task_action_point(fusion_task)
                self._fusion_attempted.add(fusion_channel)
                self.coverage_scan_at(fusion_point, with_reobserve=True)
                current_task = self.tasks.get(fusion_channel)
                if current_task is not None:
                    self.service_task_once(current_task)
                if coverage_complete(self.scan_points):
                    self.remaining_anchors.clear()
                    break
                plan_order = self._best_anchor_plan()[2]
                if plan_order:
                    self.remaining_anchors = set(plan_order)
                continue
            idx = tour[tour_pos]
            tour_pos += 1
            if idx not in self.remaining_anchors:
                continue
            p = self.ring_points[idx]
            self.remaining_anchors.discard(idx)
            self.coverage_scan_at(p, with_reobserve=True)
            if len(self.found_channels) >= 16:
                self.status('已发现16个干扰源，当前搜索点后立即结束全局搜索')
                break
            next_idx = next((j for j in tour[tour_pos:] if j in self.remaining_anchors), None)
            if next_idx is not None:
                next_p = self.ring_points[next_idx]
                self.opportunistic_clear_on_leg(next_p)
        if len(self.found_channels) < 16:
            rad, witness = coverage_radius(self.scan_points)
            sampled = _coverage_veto_sample(self.scan_points)
            if not coverage_complete(self.scan_points):
                raise RuntimeError(f'V6 固定搜索结束但覆盖认证失败：exact={rad:.3f}m, sampled={sampled:.3f}m, witness={witness}')
            self.status(f'V6 覆盖认证完成：exact≈{rad:.2f}m，sample≈{sampled:.2f}m')

    def choose_ring_tour(self) -> List[int]:
        """
        V6：稳定固定覆盖路线，只优化“从哪边开始、最终停在哪个环点”。

        对当前正 n 边形而言，沿相邻顶点连续走完全部环点时，
        2n 种（n 个起点 × 顺/逆时针）路线的搜索移动距离相同。
        因此可以在不增加搜索路程、不破坏覆盖保证的前提下，
        用原点已经发现的目标粗位置选择更好的搜索终点。

        第一优先级：
            搜索结束后，从最终环点服务当前已发现目标的开放路线尽量短。
        第二优先级（只在第一项相同的两种方向之间起作用）：
            让单示向目标尽量早在搜索环上获得第二次示向，
            以增加后续“顺路清除”的机会。
        """
        n = len(self.ring_points)
        if not self.tasks:
            return list(range(n))
        ids = sorted(self.tasks)
        roughs = [self.rough_point(self.tasks[ch]) for ch in ids]
        best_key = None
        best_tour = list(range(n))
        for st in range(n):
            for direction in (1, -1):
                tour = [(st + direction * k) % n for k in range(n)]
                endpoint = self.ring_points[tour[-1]]
                route = _heuristic_open_route(endpoint, roughs)
                service_len = _route_length(endpoint, roughs, route)
                reobserve_rank = 0
                for task in self.tasks.values():
                    if len(task.observations) != 1:
                        continue
                    ob = task.observations[0]
                    poly = task.feasible_polygon()
                    first_k = n + 2
                    if poly:
                        for k, idx in enumerate(tour):
                            p = self.ring_points[idx]
                            if distance(ob.point, p) < OPPORTUNISTIC_MIN_BASELINE:
                                continue
                            max_d = max((distance(p, q) for q in poly))
                            if max_d <= OPPORTUNISTIC_MAX_FEASIBLE_DIST:
                                first_k = k
                                break
                    reobserve_rank += first_k
                key = (service_len, reobserve_rank)
                if best_key is None or key < best_key:
                    best_key = key
                    best_tour = tour
        return best_tour

    def opportunistic_reobserve(self, p: Point) -> None:
        """
        V4 搜索途中原地补第二示向。

        与 V3 的“粗估目标点距当前点 <= 1350m”不同，V4 直接计算
        单示向目标的完整可行域，并用当前搜索点到该可行域顶点的最大距离
        作为风险指标。这样对“是否值得在这里补测”的判断更贴近真实几何。
        """
        candidates = []
        for ch, task in list(self.tasks.items()):
            if len(task.observations) != 1:
                continue
            ob = task.observations[0]
            baseline = distance(ob.point, p)
            if baseline < OPPORTUNISTIC_MIN_BASELINE:
                continue
            poly = task.feasible_polygon()
            if not poly:
                continue
            max_feasible_dist = max((distance(p, q) for q in poly))
            if max_feasible_dist <= OPPORTUNISTIC_MAX_FEASIBLE_DIST:
                candidates.append((max_feasible_dist, -baseline, ch, task))
        candidates.sort(key=lambda x: (x[0], x[1], x[2]))
        candidates = candidates[:min(OPPORTUNISTIC_MAX_PER_POINT, OPPORTUNISTIC_HARD_CAP)]
        for max_d, _neg_baseline, ch, task in candidates:
            if ch not in self.tasks or len(self.tasks[ch].observations) != 1:
                continue
            resp = self.api.measure(p, ch)
            result = resp['measure_result']
            if result == 'direction':
                task.add_observation(p, float(resp['svd_deg']))
                poly = task.feasible_polygon()
                if poly:
                    _, bound = polygon_bound(poly)
                    self.status(f'频道 {ch} 搜索途中补得第2示向，先验最远≈{max_d:.0f}m，定位包络≈{bound:.1f}m')
                continue
            if result == 'near':
                clr = self.api.clear(p, ch)
                if clr['clear_result'] == 'success':
                    self._mark_cleared(ch)
                continue
            self.status(f'频道 {ch} 搜索途中补测无信号（先验最远≈{max_d:.0f}m），继续搜索')

    def opportunistic_clear_on_leg(self, next_p: Point) -> None:
        """
        V4：在相邻两个全局搜索点之间，只做“便宜的顺路清除”。

        对已有 >=2 个示向的任务，若把其可行域中心插入当前点 -> next_p
        所增加的路程不超过 SEARCH_CLEAR_DETOUR_MAX，则顺路尝试 /clear。
        成功时可避免搜索结束后再次返回该区域；失败只增加 3s，
        且已经到达中心，随后原地 /measure 收缩定位区域。
        """
        for _ in range(SEARCH_CLEAR_MAX_PER_LEG):
            cur = self.api.position
            direct = distance(cur, next_p)
            candidates = []
            for ch, task in list(self.tasks.items()):
                if len(task.observations) < 2:
                    continue
                poly = task.feasible_polygon()
                if not poly:
                    continue
                center, bound = polygon_bound(poly)
                if bound > SAFE_CENTER_MEASURE_BOUND:
                    continue
                detour = distance(cur, center) + distance(center, next_p) - direct
                if detour <= SEARCH_CLEAR_DETOUR_MAX:
                    candidates.append((detour + 0.02 * bound, detour, bound, ch, task, center))
            if not candidates:
                return
            candidates.sort(key=lambda x: x[0])
            _, detour, bound, ch, task, center = candidates[0]
            self.status(f'频道 {ch} 位于下一搜索边附近，顺路探针：额外绕路≈{detour:.1f}m，包络≈{bound:.1f}m')
            clr = self.api.clear(center, ch)
            if clr['clear_result'] == 'success':
                self._mark_cleared(ch)
                continue
            result = self.measure_task(task, center)
            if result == 'near':
                clr = self.api.clear(center, ch)
                if clr['clear_result'] == 'success':
                    self._mark_cleared(ch)
                    continue
            elif result == 'direction':
                continue
            return

    def survey_at(self, p: Point) -> None:
        self.scan_unknown_at(p)
        self.opportunistic_reobserve(p)

    @staticmethod
    def safe_second_points(task: TargetTask) -> Tuple[Point, Point]:
        """
        由首个 direction 构造两个对称的保证可接收二次测点：
            P = S + 600*u ± 250*v
        u 为首测示向单位向量，v 为其法向。

        该构造在保证二次测点可接收的同时，显著缩短单示向主动定位移动。
        """
        ob = task.observations[0]
        a = math.radians(ob.bearing_deg)
        u = (math.cos(a), math.sin(a))
        v = (-math.sin(a), math.cos(a))
        base = add_point(ob.point, scale_point(u, SAFE_SECOND_FORWARD))
        p1 = add_point(base, scale_point(v, SAFE_SECOND_LATERAL))
        p2 = add_point(base, scale_point(v, -SAFE_SECOND_LATERAL))
        return (p1, p2)

    def choose_safe_second(self, task: TargetTask) -> Point:
        p1, p2 = self.safe_second_points(task)
        return p1 if distance(self.api.position, p1) <= distance(self.api.position, p2) else p2

    def measure_task(self, task: TargetTask, p: Point) -> str:
        resp = self.api.measure(p, task.channel)
        result = resp['measure_result']
        if result == 'direction':
            task.add_observation(p, float(resp['svd_deg']))
        return result

    def fallback_bearing_home(self, task: TargetTask) -> None:
        """仅作为异常兜底；正常 V5 基本不会进入。"""
        ch = task.channel
        step = 80.0
        max_iter = 80
        resp = self.api.measure(self.api.position, ch)
        result = resp['measure_result']
        if result == 'near':
            clr = self.api.clear(self.api.position, ch)
            if clr['clear_result'] == 'success':
                return
        if result != 'direction':
            if not task.observations:
                raise ApiError(f'频道 {ch} 无可用观测，无法 fallback')
            ob = task.observations[-1]
            resp = self.api.measure(ob.point, ch)
            result = resp['measure_result']
            if result == 'near':
                clr = self.api.clear(ob.point, ch)
                if clr['clear_result'] == 'success':
                    return
            if result != 'direction':
                raise ApiError(f'频道 {ch} fallback 无法重新捕获信号')
        bearing = float(resp['svd_deg'])
        for _ in range(max_iter):
            heading = bearing
            p = point_along(self.api.position, heading, step)
            resp = self.api.measure(p, ch)
            result = resp['measure_result']
            if result == 'near':
                clr = self.api.clear(p, ch)
                if clr['clear_result'] == 'success':
                    return
            if result != 'direction':
                step = max(step / 2.0, 5.0)
                continue
            new_bearing = float(resp['svd_deg'])
            turn = abs(angle_diff_deg(new_bearing, heading))
            if turn > 90.0:
                step = max(step / 2.0, 5.0)
                if step <= 20.0:
                    clr = self.api.clear(self.api.position, ch)
                    if clr['clear_result'] == 'success':
                        return
            bearing = new_bearing
            if step <= 8.0:
                clr = self.api.clear(self.api.position, ch)
                if clr['clear_result'] == 'success':
                    return
        raise ApiError(f'频道 {ch} fallback 追踪超过迭代上限')

    def clear_task(self, task: TargetTask) -> None:
        ch = task.channel
        self.status(f'开始定位清除频道 {ch}，已有 {len(task.observations)} 个示向')
        for _ in range(MAX_LOCALIZE_OBS + 2):
            if len(task.observations) == 1:
                p2 = self.choose_safe_second(task)
                result = self.measure_task(task, p2)
                if result == 'near':
                    clr = self.api.clear(p2, ch)
                    if clr['clear_result'] == 'success':
                        self._mark_cleared(ch)
                        return
                if result == 'direction':
                    continue
                a, b = self.safe_second_points(task)
                other = b if distance(p2, a) < 1e-06 else a
                result = self.measure_task(task, other)
                if result == 'direction':
                    continue
                if result == 'near':
                    clr = self.api.clear(other, ch)
                    if clr['clear_result'] == 'success':
                        self._mark_cleared(ch)
                        return
                self.status(f'频道 {ch} 保证二次测点异常，进入 fallback')
                self.fallback_bearing_home(task)
                self._mark_cleared(ch)
                return
            poly = task.feasible_polygon()
            if not poly:
                self.status(f'频道 {ch} 可行域为空，进入 fallback')
                self.fallback_bearing_home(task)
                self._mark_cleared(ch)
                return
            center, bound = polygon_bound(poly)
            self.status(f'频道 {ch} 可行域包络≈{bound:.2f}m')
            if bound <= DIRECT_CLEAR_BOUND:
                clr = self.api.clear(center, ch)
                if clr['clear_result'] == 'success':
                    self._mark_cleared(ch)
                    return
                result = self.measure_task(task, center)
                if result == 'near':
                    clr = self.api.clear(center, ch)
                    if clr['clear_result'] == 'success':
                        self._mark_cleared(ch)
                        return
                if result == 'direction':
                    continue
            if bound <= SAFE_CENTER_MEASURE_BOUND:
                result = self.measure_task(task, center)
                if result == 'near':
                    clr = self.api.clear(center, ch)
                    if clr['clear_result'] == 'success':
                        self._mark_cleared(ch)
                        return
                if result == 'direction':
                    continue
            p2 = self.choose_safe_second(task)
            p1, p_other = self.safe_second_points(task)
            if min((distance(p2, ob.point) for ob in task.observations)) < 200.0:
                p2 = p_other if distance(p2, p1) < 1e-06 else p1
            result = self.measure_task(task, p2)
            if result == 'near':
                clr = self.api.clear(p2, ch)
                if clr['clear_result'] == 'success':
                    self._mark_cleared(ch)
                    return
            if result == 'direction':
                continue
            self.status(f'频道 {ch} 主动定位异常，进入 fallback')
            self.fallback_bearing_home(task)
            self._mark_cleared(ch)
            return
        self.status(f'频道 {ch} 定位迭代超限，进入 fallback')
        self.fallback_bearing_home(task)
        self._mark_cleared(ch)

    def _mark_cleared(self, ch: int) -> None:
        self.cleared_channels.add(ch)
        self.found_channels.add(ch)
        self.unknown_channels.discard(ch)
        self.tasks.pop(ch, None)
        self.status(f'频道 {ch} 清除完成')

    def task_action_point(self, task: TargetTask) -> Tuple[Point, str, float]:
        """
        返回该任务“下一步真正要去的位置”，而不是目标粗估位置。

        V2 的一个核心问题是：DP 按粗目标点排序，但真正执行时，单示向目标
        会先去另一个安全二测点，导致“规划点”和“执行点”不一致。V5 直接
        对下一动作点规划，并且每做完一次测量/清除就重新规划。
        """
        if len(task.observations) == 1:
            p = self.choose_safe_second(task)
            return (p, 'second_measure', float('inf'))
        poly = task.feasible_polygon()
        if not poly:
            return (self.rough_point(task), 'fallback', float('inf'))
        center, bound = polygon_bound(poly)
        if bound > SAFE_CENTER_MEASURE_BOUND:
            p1, p2 = self.safe_second_points(task)

            def score(p: Point) -> float:
                mind = min((distance(p, ob.point) for ob in task.observations))
                return distance(self.api.position, p) - 0.2 * mind
            p = min((p1, p2), key=score)
            return (p, 'safe_remeasure', bound)
        return (center, 'center', bound)

    def choose_next_task_action(self) -> TargetTask:
        """对所有任务的“下一真实动作点”做开放 TSP，再执行第一步。"""
        ids = sorted(self.tasks)
        points: List[Point] = []
        for ch in ids:
            p, _, _ = self.task_action_point(self.tasks[ch])
            points.append(p)
        route = _route_plan(self.api.position, points)
        return self.tasks[ids[route[0]]]

    def service_task_once(self, task: TargetTask) -> None:
        """
        只推进一个信息动作，然后立刻回到全局重规划。

        这样不会像 V2 那样“选中一个频道后无论多远都一路追到底”，尤其可避免
        单示向目标在二测后目标位置发生大修正造成的跨区域折返。
        """
        ch = task.channel
        if len(task.observations) == 1:
            p = self.choose_safe_second(task)
            result = self.measure_task(task, p)
            if result == 'near':
                clr = self.api.clear(p, ch)
                if clr['clear_result'] == 'success':
                    self._mark_cleared(ch)
                    return
            if result == 'direction':
                poly = task.feasible_polygon()
                if poly:
                    _, b = polygon_bound(poly)
                    self.status(f'频道 {ch} 完成安全二测，包络≈{b:.1f}m，返回全局重规划')
                return
            p1, p2 = self.safe_second_points(task)
            other = p2 if distance(p, p1) < 1e-06 else p1
            result = self.measure_task(task, other)
            if result == 'direction':
                return
            if result == 'near':
                clr = self.api.clear(other, ch)
                if clr['clear_result'] == 'success':
                    self._mark_cleared(ch)
                    return
            self.status(f'频道 {ch} 安全二测异常，进入 fallback')
            self.fallback_bearing_home(task)
            self._mark_cleared(ch)
            return
        poly = task.feasible_polygon()
        if not poly:
            self.status(f'频道 {ch} 可行域为空，进入 fallback')
            self.fallback_bearing_home(task)
            self._mark_cleared(ch)
            return
        center, bound = polygon_bound(poly)
        self.status(f'频道 {ch} 下一动作：集合中心，包络≈{bound:.2f}m')
        if bound > SAFE_CENTER_MEASURE_BOUND:
            p, _, _ = self.task_action_point(task)
            result = self.measure_task(task, p)
            if result == 'direction':
                return
            if result == 'near':
                clr = self.api.clear(p, ch)
                if clr['clear_result'] == 'success':
                    self._mark_cleared(ch)
                    return
            self.status(f'频道 {ch} 大包络补测异常，进入 fallback')
            self.fallback_bearing_home(task)
            self._mark_cleared(ch)
            return
        if bound <= PROBE_CLEAR_BOUND and distance(self.api.position, center) <= SAFE_CENTER_PROBE_DIST:
            clr = self.api.clear(center, ch)
            if clr['clear_result'] == 'success':
                self._mark_cleared(ch)
                return
            self.status(f'频道 {ch} 中心快速清除未命中（仅3s），原地补测')
        elif bound <= PROBE_CLEAR_BOUND:
            self.status(f'频道 {ch} 中心探针距离过远，跳过试探并直接测向')
        result = self.measure_task(task, center)
        if result == 'near':
            clr = self.api.clear(center, ch)
            if clr['clear_result'] == 'success':
                self._mark_cleared(ch)
                return
        if result == 'direction':
            return
        self.status(f'频道 {ch} 集合中心测量异常，进入 fallback')
        self.fallback_bearing_home(task)
        self._mark_cleared(ch)

    def run(self) -> None:
        enter = self.api.enter()
        self.status(f'/enter 成功，现实剩余时间 {enter.get('remaining_real_duration_s')} s')
        self.status('启动 V6 稳定固定覆盖搜索')
        self.adaptive_search()
        one = sum((1 for t in self.tasks.values() if len(t.observations) == 1))
        two_plus = sum((1 for t in self.tasks.values() if len(t.observations) >= 2))
        self.status(f'全局搜索完成：待清除={len(self.tasks)}，单示向={one}，双/多示向={two_plus}')
        while self.tasks:
            task = self.choose_next_task_action()
            self.service_task_once(task)
        self.status('全部已发现目标清除完毕，准备 /exit')
        exit_resp = self.api.exit()
        self.status(f'/exit: {exit_resp.get('exit_reason')}')
        n = len(self.cleared_channels)
        if n > 0:
            avg = self.api.virtual_time_s / n
            print('\n========== 本次结果 V6 STABLE ==========')
            print(f'清除频道数: {n}')
            print(f'虚拟总时间: {self.api.virtual_time_s:.3f} s')
            print(f'平均定位清除时间: {avg:.3f} s/个')
            print(f'已清除频道: {sorted(self.cleared_channels)}')
            print(f'本地策略日志: {self.api.log_path}')
        else:
            print('本次未清除目标。')

def self_test() -> None:
    r = SURVEY_R
    half_sector = math.pi / SURVEY_N
    outer_mid = math.sqrt(AREA_R ** 2 + r ** 2 - 2.0 * AREA_R * r * math.cos(half_sector))
    center_ring_voronoi = r / (2.0 * math.cos(half_sector))
    edge = 2.0 * r * math.sin(half_sector)
    full_search_path = r + (SURVEY_N - 1) * edge
    print('=== V6 STABLE 几何自检 ===')
    print(f'SURVEY_N = {SURVEY_N}')
    print(f'SURVEY_R = {r:.6f} m')
    print(f'相邻环点距离 = {edge:.6f} m')
    print(f'完整环搜索移动距离 = {full_search_path:.6f} m')
    print(f'外边界扇区中点最坏距离 = {outer_mid:.6f} m')
    print(f'中心/环点 Voronoi 交界距离 = {center_ring_voronoi:.6f} m')
    assert outer_mid <= SURVEY_COVER_R + 1e-06
    assert outer_mid < MIN_RECV_R
    assert center_ring_voronoi < MIN_RECV_R
    pts = [(0.0, 0.0)] + [(r * math.cos(2.0 * math.pi * k / SURVEY_N), r * math.sin(2.0 * math.pi * k / SURVEY_N)) for k in range(SURVEY_N)]
    worst = 0.0
    worst_p = (0.0, 0.0)
    step = 20.0
    x = -AREA_R
    while x <= AREA_R + 1e-09:
        y = -AREA_R
        while y <= AREA_R + 1e-09:
            if x * x + y * y <= AREA_R * AREA_R + 1e-09:
                d = min((distance((x, y), q) for q in pts))
                if d > worst:
                    worst = d
                    worst_p = (x, y)
            y += step
        x += step
    print(f'20 m 网格离散检查：最大最近点距离 ≈ {worst:.3f} m @ {worst_p}')
    assert worst < MIN_RECV_R
    task = TargetTask(channel=1)
    task.add_observation((0.0, 0.0), 0.0)
    task.add_observation((300.0 * math.cos(math.radians(30.0)), 300.0 * math.sin(math.radians(30.0))), 330.0)
    poly = task.feasible_polygon()
    assert poly
    c, b = polygon_bound(poly)
    print(f'多边形模块：顶点={len(poly)}，中心={c}，包络半径={b:.2f} m')
    alpha = math.radians(BEARING_ERR_DEG)
    worst2 = 0.0
    worst_arg = None
    for ii in range(1001):
        rho = 5.0001 + (MAX_RECV_R - 5.0001) * ii / 1000
        for j in range(101):
            delta = -alpha + 2.0 * alpha * j / 100
            tx, ty = (rho * math.cos(delta), rho * math.sin(delta))
            d2 = math.hypot(tx - SAFE_SECOND_FORWARD, ty - SAFE_SECOND_LATERAL)
            if d2 > worst2:
                worst2 = d2
                worst_arg = (rho, math.degrees(delta))
    print(f'V4 安全二次测点最坏目标距离 ≈ {worst2:.3f} m @ {worst_arg}')
    assert worst2 < MIN_RECV_R
    sample = [(100.0, 200.0), (-500.0, 700.0), (900.0, -200.0), (-800.0, -600.0), (300.0, 1100.0), (1200.0, 500.0)]
    rt = _route_plan((0.0, 0.0), sample)
    assert sorted(rt) == list(range(len(sample)))
    print(f'路线模块样例长度 = {_route_length((0.0, 0.0), sample, rt):.3f} m')
    centers = [(0.0, 0.0)] + [(r * math.cos(2.0 * math.pi * k / SURVEY_N), r * math.sin(2.0 * math.pi * k / SURVEY_N)) for k in range(SURVEY_N)]
    cr0, w0 = coverage_radius([(0.0, 0.0)])
    print(f'仅原点扫描时覆盖半径 = {cr0:.6f} m（正确值应为1800m）')
    assert abs(cr0 - AREA_R) < 1e-06
    assert not coverage_complete([(0.0, 0.0)])
    cr, witness = coverage_radius(centers)
    print(f'V6 STABLE精确覆盖认证半径 = {cr:.6f} m，见证点={witness}')
    assert cr <= COVER_CERT_R
    assert coverage_complete(centers)
    reduced = centers[:-1]
    cr2, _ = coverage_radius(reduced)
    print(f'去掉一个环点后的覆盖半径 = {cr2:.6f} m')
    assert cr2 > COVER_CERT_R
    fixed_pts = [(0.0, 0.0)] + [(SURVEY_R * math.cos(2.0 * math.pi * k / SURVEY_N), SURVEY_R * math.sin(2.0 * math.pi * k / SURVEY_N)) for k in range(SURVEY_N)]
    fixed_cov, fixed_w = coverage_radius(fixed_pts)
    fixed_sample = _coverage_veto_sample(fixed_pts)
    fixed_edge = 2.0 * SURVEY_R * math.sin(math.pi / SURVEY_N)
    fixed_path = SURVEY_R + (SURVEY_N - 1) * fixed_edge
    print(f'V6固定覆盖半径 = {fixed_cov:.6f} m，采样复核={fixed_sample:.6f} m')
    print(f'V6固定搜索路径 = {fixed_path:.6f} m')
    assert fixed_cov <= COVER_CERT_R
    assert fixed_sample <= COVER_CERT_R
    assert fixed_cov < MIN_RECV_R
    print('SELF TEST PASSED')

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description='2026 数模 B 题问题3自动搜索定位清除 V3')
    p.add_argument('--robot-id', default=ROBOT_ID, help='参赛队号')
    p.add_argument('--base-url', default=BASE_URL, help='模拟器接口地址')
    p.add_argument('--log', default=LOCAL_LOG, help='本地 JSONL 日志文件')
    p.add_argument('--self-test', action='store_true', help='只做几何/算法自检，不连接模拟器')
    return p.parse_args()

def main() -> None:
    args = parse_args()
    if args.self_test:
        self_test()
        return
    if args.robot_id == '<参赛队号>':
        raise SystemExit('请先修改 ROBOT_ID，或运行：python B3_robot_solver_v3.py --robot-id 你的参赛队号')
    api = RobotAPI(args.base_url, args.robot_id, args.log)
    solver = B3Solver(api)
    try:
        solver.run()
    except KeyboardInterrupt:
        print('\n收到 Ctrl+C，中止。若测试仍开放，请根据现场情况决定是否手工结束。')
        raise
    except Exception as e:
        print(f'\n程序异常：{type(e).__name__}: {e}')
        print(f'最近虚拟时间：{api.virtual_time_s:.3f} s')
        print(f'最近位置：{api.position}')
        print(f'本地日志：{api.log_path}')
        raise
if __name__ == '__main__':
    main()
