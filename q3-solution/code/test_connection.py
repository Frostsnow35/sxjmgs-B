# test_connection.py
#附件2第11节的示例 验证接口
from robot_client import Robot

def main():
    # 队号必须和模拟器登录的一致！
    robot = Robot(base_url="http://127.0.0.1:2026", robot_id="202619007122")

    # 1. 进入
    resp = robot.enter()
    if not resp.get("accepted"):
        print("进入失败:", resp)
        return
    print(f"进入成功！可用现实时间: {resp['remaining_real_duration_s']} 秒")

    # 2. 按照示例走一遍流程
    # 步骤2：移动到(300,400)，检测频道1
    resp = robot.measure(300, 400, 1)
    print(f"第2步 measure(300,400,1): 虚拟时间={resp['virtual_time_s']}, 结果={resp['measure_result']}")

    # 步骤3：原地不动，检测频道2
    resp = robot.measure(300, 400, 2)
    print(f"第3步 measure(300,400,2): 虚拟时间={resp['virtual_time_s']}, 结果={resp['measure_result']}")

    # 步骤4：移动到(300,0)，清除频道3
    resp = robot.clear(300, 0, 3)
    print(f"第4步 clear(300,0,3): 虚拟时间={resp['virtual_time_s']}, 结果={resp.get('clear_result')}")

    # 步骤5：原地不动，检测频道2（因为clear不切频道，当前频道还是2）
    resp = robot.measure(300, 0, 2)
    print(f"第5步 measure(300,0,2): 虚拟时间={resp['virtual_time_s']}, 结果={resp['measure_result']}")

    # 6. 退出
    resp = robot.exit()
    print(f"退出: 虚拟时间={resp['virtual_time_s']}, 退出原因={resp.get('exit_reason')}")

    # 预期最后虚拟时间应该是199秒，你可以对照检查

if __name__ == "__main__":
    main()