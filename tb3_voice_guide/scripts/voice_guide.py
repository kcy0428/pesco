#!/usr/bin/env python3
"""
음성 안내로봇 — Gemini Live API (STT+LLM+TTS 통합) + Nav2 자율주행.

흐름:
  🎤 마이크 -> Gemini Live (음성인식+대화+함수호출) -> navigate_to(destination)
    -> 웨이포인트 조회(이름->map 좌표) -> Nav2 NavigateToPose 목표 전송 -> 🔊 Gemini 음성 응답

실행 (VMware, 마이크·스피커 있는 머신):
  export GEMINI_API_KEY="..."      # 또는 ~/.bashrc 에 등록
  source /opt/ros/jazzy/setup.bash && source ~/voice_venv/bin/activate
  export ROS_DOMAIN_ID=30
  python3 voice_guide.py --waypoints ~/D435i/tb3_voice_guide/config/waypoints.yaml

의존성(venv): google-genai, sounddevice, pyyaml, numpy<2  (+ 시스템 libportaudio2)
전제: Nav2 가 저장한 지도로 실행 중 (localization + navigation), 로봇 bringup 실행 중.
"""

import os
import sys
import argparse
import asyncio
import threading
import queue
import math

import yaml
import numpy as np
import sounddevice as sd

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from google import genai
from google.genai import types

MODEL = "gemini-2.5-flash-native-audio-latest"   # Live 네이티브 음성 모델 (models.list 로 확인)
SEND_SR = 16000   # 마이크 입력 샘플레이트 (Gemini 입력 규격)
RECV_SR = 24000   # Gemini 오디오 출력 샘플레이트
BLOCK = 1600      # 0.1s @ 16kHz


# ─────────────────────── ROS 2 (Nav2 목표 전송) ───────────────────────
class NavBridge(Node):
    def __init__(self, waypoints):
        super().__init__('voice_guide')
        self.waypoints = waypoints
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def resolve(self, name):
        """목적지 이름/별칭 -> 웨이포인트 키. 없으면 None."""
        name = (name or '').strip()
        for key, wp in self.waypoints.items():
            if name == key or name in wp.get('aliases', []):
                return key
        # 부분 일치 보조
        for key, wp in self.waypoints.items():
            if name and (name in key or any(name in a for a in wp.get('aliases', []))):
                return key
        return None

    def go(self, name):
        key = self.resolve(name)
        if key is None:
            return f"'{name}' 은(는) 등록된 장소가 아닙니다. 가능한 곳: {', '.join(self.waypoints)}"
        wp = self.waypoints[key]
        if not self.client.wait_for_server(timeout_sec=2.0):
            return "Nav2 가 아직 준비되지 않았습니다 (navigate_to_pose 액션 서버 없음)."
        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(wp['x'])
        p.pose.position.y = float(wp['y'])
        yaw = float(wp.get('yaw', 0.0))
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = p
        self.client.send_goal_async(goal)   # 비동기 전송 (즉시 반환)
        self.get_logger().info(f"Nav2 목표 전송: {key} ({wp['x']:.2f}, {wp['y']:.2f})")
        return f"{key} 으로 안내를 시작합니다."


# ─────────────────────── Gemini Live 세션 ───────────────────────
def build_config(waypoints):
    places = ', '.join(waypoints.keys())
    system = (
        "너는 실내 안내로봇의 음성 비서다. 한국어로 짧고 친절하게 대답한다. "
        f"사용자가 가고 싶은 장소를 말하면 navigate_to 함수를 호출해 로봇을 보낸다. "
        f"등록된 장소: {places}. 등록되지 않은 곳을 요청하면 가능한 장소를 안내한다."
    )
    nav_decl = types.FunctionDeclaration(
        name="navigate_to",
        description="로봇을 지정한 장소로 자율주행시킨다",
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={"destination": types.Schema(type=types.Type.STRING,
                                                    description="목적지 이름 (등록된 장소 중 하나)")},
            required=["destination"],
        ),
    )
    return types.LiveConnectConfig(
        response_modalities=["AUDIO"],   # 네이티브 음성 모델이 한국어 음성으로 응답
        system_instruction=types.Content(parts=[types.Part(text=system)]),
        tools=[types.Tool(function_declarations=[nav_decl])],
    )


async def run(nav: NavBridge, waypoints):
    key = os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")
    if not key:
        print("ERROR: GEMINI_API_KEY 환경변수가 없습니다. export GEMINI_API_KEY=... 하세요.")
        return
    client = genai.Client(api_key=key)

    mic_q: queue.Queue = queue.Queue()

    def mic_cb(indata, frames, t, status):
        mic_q.put(bytes(indata))

    out_stream = sd.RawOutputStream(samplerate=RECV_SR, channels=1, dtype='int16')
    out_stream.start()

    async with client.aio.live.connect(model=MODEL, config=build_config(waypoints)) as session:
        print("🎤 음성 안내 시작 — 말해보세요 (예: '회의실 가줘'). Ctrl+C 로 종료.")

        async def send_mic():
            loop = asyncio.get_event_loop()
            with sd.RawInputStream(samplerate=SEND_SR, blocksize=BLOCK, channels=1,
                                   dtype='int16', callback=mic_cb):
                while True:
                    data = await loop.run_in_executor(None, mic_q.get)
                    await session.send_realtime_input(
                        audio=types.Blob(data=data, mime_type=f"audio/pcm;rate={SEND_SR}"))

        async def recv():
            while True:
                async for resp in session.receive():
                    # 1) 음성 응답 재생
                    if resp.data:
                        out_stream.write(resp.data)
                    # 2) 함수 호출 처리 (navigate_to)
                    if resp.tool_call:
                        responses = []
                        for fc in resp.tool_call.function_calls:
                            if fc.name == "navigate_to":
                                dest = fc.args.get("destination", "")
                                result = nav.go(dest)
                                print(f"[navigate_to] {dest} -> {result}")
                                responses.append(types.FunctionResponse(
                                    id=fc.id, name=fc.name, response={"result": result}))
                        if responses:
                            await session.send_tool_response(function_responses=responses)

        await asyncio.gather(send_mic(), recv())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--waypoints', default='waypoints.yaml')
    args, _ = ap.parse_known_args()
    with open(args.waypoints) as f:
        waypoints = yaml.safe_load(f)['waypoints']

    rclpy.init()
    nav = NavBridge(waypoints)
    spin = threading.Thread(target=rclpy.spin, args=(nav,), daemon=True)
    spin.start()
    try:
        asyncio.run(run(nav, waypoints))
    except KeyboardInterrupt:
        pass
    finally:
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
