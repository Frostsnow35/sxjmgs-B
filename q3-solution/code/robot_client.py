# robot_client.py
#接口封装
#request_id 生成、网络重试、状态记录（当前位置、当前频道、虚拟时间）。

import json
import time
import requests

class Robot:
    def __init__(self, base_url="http://127.0.0.1:2026", robot_id="202619007122"):
        self.base_url = base_url
        self.robot_id = robot_id
        self.request_counter = 0
        self.current_channel = 1          # 测向机当前频道，初始1
        self.current_position = (0.0, 0.0) # 初始位置
        self.virtual_time = 0.0            # 最近一次accepted=true的虚拟时刻
        self.logs = []                     # 记录所有交互，方便后续导出

    def _next_id(self, prefix):
        self.request_counter += 1
        return f"{prefix}-{self.request_counter}"

    def _post(self, path, payload, retry=3):
        """发送HTTP请求，带重试机制（网络中断时复用同一个request_id）"""
        for attempt in range(retry):
            try:
                resp = requests.post(
                    self.base_url + path,
                    json=payload,
                    headers={"Content-Type": "application/json"},
                    timeout=10
                )
                data = resp.json()
                self.logs.append({"path": path, "payload": payload, "response": data})
                return data
            except Exception as e:
                if attempt == retry - 1:
                    print(f"[ERROR] 请求失败: {path}, {e}")
                    raise
                print(f"[WARN] 网络异常，重试 {attempt+1}/3 ...")
                time.sleep(1)

    def _base_payload(self, request_id):
        return {
            "arena_id": "default",
            "robot_id": self.robot_id,
            "request_id": request_id
        }

    def enter(self):
        payload = self._base_payload(self._next_id("enter"))
        resp = self._post("/enter", payload)
        if resp.get("accepted"):
            self.virtual_time = resp.get("virtual_time_s", 0.0)
        return resp

    def measure(self, x, y, channel):
        payload = self._base_payload(self._next_id("measure"))
        payload["position"] = {"x": x, "y": y}
        payload["channel"] = channel
        resp = self._post("/measure", payload)
        if resp.get("accepted"):
            self.virtual_time = resp.get("virtual_time_s", self.virtual_time)
            self.current_channel = channel
            self.current_position = (x, y)
        return resp

    def clear(self, x, y, channel):
        payload = self._base_payload(self._next_id("clear"))
        payload["position"] = {"x": x, "y": y}
        payload["channel"] = channel
        resp = self._post("/clear", payload)
        if resp.get("accepted"):
            self.virtual_time = resp.get("virtual_time_s", self.virtual_time)
            self.current_position = (x, y)
            # 注意：/clear 不会改变测向机频道状态
        return resp

    def exit(self):
        payload = self._base_payload(self._next_id("exit"))
        resp = self._post("/exit", payload)
        if resp.get("accepted"):
            self.virtual_time = resp.get("virtual_time_s", self.virtual_time)
        return resp