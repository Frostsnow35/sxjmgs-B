# test_run.py
#自动记录数据的测试脚本
import math
import csv
import os
from datetime import datetime
from robot_client import Robot

# ====== 可调参数 ======
GRID_POINTS = [
    (0, 0),
    (600, 0), (-600, 0), (0, 600), (0, -600),
    (1200, 0), (-1200, 0), (0, 1200), (0, -1200),
    (850, 850), (-850, 850), (850, -850), (-850, -850),
    (500, 0), (-500, 0), (0, 500), (0, -500)
]
CHANNEL_ORDER = list(range(1, 21))
# =====================

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

def approach_and_clear(robot, est_pos, ch, max_steps=15):
    x, y = est_pos
    steps = [20, 20, 15, 15, 10, 10, 8, 8, 5, 5, 3, 3, 3, 3, 3][:max_steps]
    last_bearing = None
    for i, step in enumerate(steps):
        resp = robot.measure(x, y, ch)
        if not resp or not resp.get("accepted"):
            return False, i + 1
        result = resp.get("measure_result")
        if result == "near":
            cr = robot.clear(x, y, ch)
            if cr.get("clear_result") == "success":
                return True, i + 1
            return False, i + 1
        elif result == "direction":
            bearing = resp["svd_deg"]
            rad = math.radians(bearing)
            x += step * math.cos(rad)
            y += step * math.sin(rad)
            last_bearing = bearing
        elif result == "no_signal":
            if last_bearing is not None:
                rad = math.radians(last_bearing)
                x -= 3 * math.cos(rad)
                y -= 3 * math.sin(rad)
            else:
                return False, i + 1
    return False, len(steps)

def run_one_test(test_label="test"):
    robot = Robot(robot_id="202619007122")
    resp = robot.enter()
    if not resp.get("accepted"):
        return None
    start_vt = 0
    detected_info = {}
    cleared_channels = set()
    scan_measures = 0
    approach_measures = 0
    approach_clears = 0

    # 第一阶段：扫描
    for pos in GRID_POINTS:
        x, y = pos
        if robot.virtual_time > 360000 * 0.85:
            break
        for ch in CHANNEL_ORDER:
            if ch in cleared_channels:
                continue
            resp = robot.measure(x, y, ch)
            scan_measures += 1
            if not resp or not resp.get("accepted"):
                continue
            result = resp.get("measure_result")
            if result == "direction":
                bearing = resp["svd_deg"]
                if ch not in detected_info:
                    detected_info[ch] = []
                detected_info[ch].append((pos, bearing))
            elif result == "near":
                cr = robot.clear(x, y, ch)
                if cr.get("clear_result") == "success":
                    cleared_channels.add(ch)

    # 第二阶段：定位+逼近清除
    failed_channels = []
    for ch, measurements in detected_info.items():
        if ch in cleared_channels:
            continue
        if len(measurements) < 2:
            failed_channels.append((ch, "insufficient_measurements"))
            continue
        est = locate_by_grid_search(measurements)
        if est is None:
            failed_channels.append((ch, "locate_failed"))
            continue
        success, steps_used = approach_and_clear(robot, est, ch)
        approach_measures += steps_used
        if success:
            cleared_channels.add(ch)
            approach_clears += 1
        else:
            failed_channels.append((ch, f"approach_failed_{steps_used}steps"))

    exit_resp = robot.exit()
    total_vt = robot.virtual_time

    # 统计
    result = {
        "test_label": test_label,
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "found_channels": len(detected_info),
        "cleared_channels": len(cleared_channels),
        "failed_channels": len(failed_channels),
        "clear_rate": f"{len(cleared_channels)/max(len(detected_info),1)*100:.1f}%",
        "total_virtual_time": round(total_vt, 1),
        "scan_measures": scan_measures,
        "approach_measures": approach_measures,
        "failed_detail": "; ".join([f"{ch}:{reason}" for ch, reason in failed_channels]),
    }
    return result

def main():
    results = []
    num_runs = 1  # 改成 1，每局单独跑
    for i in range(num_runs):
        label = f"run_{i+1}"
        print(f"\n{'='*60}")
        print(f"开始演练 {label}")
        print(f"{'='*60}")
        r = run_one_test(label)
        if r:
            results.append(r)
            print(f"完成 {label}: 发现{r['found_channels']}个, 清除{r['cleared_channels']}个, 清除率{r['clear_rate']}, 总时间{r['total_virtual_time']}秒")
        else:
            print(f"{label} 失败")

    csv_file = "演练记录.csv"
    file_exists = os.path.exists(csv_file)
    with open(csv_file, "a", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        if not file_exists:
            writer.writeheader()
        for r in results:
            writer.writerow(r)
    print(f"\n结果已保存到 {csv_file}")

if __name__ == "__main__":
    main()