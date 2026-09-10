# auto_search.py
import math
from robot_client import Robot

def angle_diff(a, b):
    d = abs(a - b) % 360
    return min(d, 360 - d)

def locate_by_grid_search(measurements):
    if len(measurements) < 2:
        return None
    best_score = float('inf')
    best_pos = None
    for x in range(-1800, 1801, 50):
        for y in range(-1800, 1801, 50):
            if math.hypot(x, y) > 1800:
                continue
            score = 0
            for (px, py), theta in measurements:
                pred = math.degrees(math.atan2(y - py, x - px))
                pred = (pred + 360) % 360
                score += angle_diff(pred, theta) ** 2
            if score < best_score:
                best_score = score
                best_pos = (x, y)
    if best_pos is None:
        return None
    fine_best_score = best_score
    fine_best_pos = best_pos
    for x in range(best_pos[0] - 60, best_pos[0] + 61, 5):
        for y in range(best_pos[1] - 60, best_pos[1] + 61, 5):
            if math.hypot(x, y) > 1800:
                continue
            score = 0
            for (px, py), theta in measurements:
                pred = math.degrees(math.atan2(y - py, x - px))
                pred = (pred + 360) % 360
                score += angle_diff(pred, theta) ** 2
            if score < fine_best_score:
                fine_best_score = score
                fine_best_pos = (x, y)
    return fine_best_pos


def approach_and_clear(robot, est_pos, ch):
    print(f" -> 频道{ch} 从估算位置({est_pos[0]:.1f},{est_pos[1]:.1f})开始逼近...")
    x, y = est_pos
    steps = [20, 20, 15, 15, 10, 10, 8, 8, 5, 5, 3, 3, 3, 3, 3]
    last_bearing = None
    for i, step in enumerate(steps):
        resp = robot.measure(x, y, ch)
        if not resp or not resp.get("accepted"):
            break
        result = resp.get("measure_result")
        if result == "near":
            print(f"    第{i+1}步 位置({x:.1f},{y:.1f}) 距离过近！尝试清除...")
            cr = robot.clear(x, y, ch)
            if cr.get("clear_result") == "success":
                print(f"    [成功] 频道{ch} 清除成功！")
                return True
            return False
        elif result == "direction":
            bearing = resp["svd_deg"]
            rad = math.radians(bearing)
            x += step * math.cos(rad)
            y += step * math.sin(rad)
            print(f"    第{i+1}步 位置({x:.1f},{y:.1f}) 示向度{bearing:.1f}° 前进{step}米")
            last_bearing = bearing
        elif result == "no_signal":
            print(f"    第{i+1}步 位置({x:.1f},{y:.1f}) 信号丢失，回退...")
            if last_bearing is not None:
                rad = math.radians(last_bearing)
                x -= 3 * math.cos(rad)
                y -= 3 * math.sin(rad)
            else:
                break
    print(f" -> 频道{ch} 逼近失败，放弃。")
    return False


def auto_scan_and_clear():
    robot = Robot(robot_id="202619007122")
    resp = robot.enter()
    if not resp.get("accepted"):
        print("进入失败")
        return
    print(f"进入成功！可用现实时间: {resp['remaining_real_duration_s']} 秒")

    # 9个扫描点，覆盖更均匀
    grid_points = [
        (0, 0),
        (700, 0), (-700, 0), (0, 700), (0, -700),
        (500, 500), (-500, 500), (500, -500), (-500, -500)
    ]

    detected_info = {}
    cleared_channels = set()

    # 第一阶段：扫描收集示向度
    for pos in grid_points:
        x, y = pos
        if robot.virtual_time > 360000 * 0.85:
            print("虚拟时间快到了，提前结束扫描")
            break
        print(f"\n--- 在 ({x},{y}) 扫描频道1-20 ---")
        for ch in range(1, 21):
            if ch in cleared_channels:
                continue
            resp = robot.measure(x, y, ch)
            if not resp or not resp.get("accepted"):
                continue
            result = resp.get("measure_result")
            if result == "direction":
                bearing = resp["svd_deg"]
                if ch not in detected_info:
                    detected_info[ch] = []
                detected_info[ch].append((pos, bearing))
                print(f"  频道{ch}: direction, 示向度={bearing:.2f}")
            elif result == "near":
                print(f"  频道{ch}: near! 直接清除")
                cr = robot.clear(x, y, ch)
                if cr.get("clear_result") == "success":
                    print(f"    [成功] 频道{ch} 清除成功！")
                    cleared_channels.add(ch)

    print(f"\n发现信号的频道: {list(detected_info.keys())}")

    # 第二阶段：对每个有>=2次测量的频道定位 + 逼近清除
    for ch, measurements in detected_info.items():
        if ch in cleared_channels:
            continue
        if len(measurements) < 2:
            continue
        print(f"\n{'='*50}")
        print(f"频道{ch} 有 {len(measurements)} 次测量，开始定位")
        est = locate_by_grid_search(measurements)
        if est is None:
            continue
        print(f"  定位估算位置: ({est[0]:.1f}, {est[1]:.1f})")
        success = approach_and_clear(robot, est, ch)
        if success:
            cleared_channels.add(ch)

    print(f"\n扫描完成。已清除频道: {list(cleared_channels)}")
    print(f"未清除频道: {[ch for ch in detected_info if ch not in cleared_channels]}")
    robot.exit()


if __name__ == "__main__":
    auto_scan_and_clear()