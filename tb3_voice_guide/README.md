# tb3_voice_guide

음성 안내로봇 — **Gemini Live API**(STT+LLM+TTS 통합) + Nav2 자율주행.

```
🎤 마이크 -> Gemini Live (음성인식+대화+함수호출 navigate_to)
  -> 웨이포인트 조회(이름->map 좌표) -> Nav2 NavigateToPose -> 🔊 Gemini 음성 응답
```
Gemini Live 하나가 음성인식·대화·음성합성·의도추출(function calling)을 모두 담당 → 별도 STT/LLM/TTS 불필요.

## 1. Gemini API 키 등록
`google-genai` SDK 는 **환경변수**에서 키를 읽는다 (코드에 적지 않음):
```bash
# ~/.bashrc 에 (VMware, 음성 실행 머신)
echo 'export GEMINI_API_KEY="당신의_키"' >> ~/.bashrc && source ~/.bashrc
```
- `GEMINI_API_KEY` 또는 `GOOGLE_API_KEY` 를 자동 인식.
- **git 커밋 금지** (키는 환경변수로만).

## 2. 설치 (VMware)
```bash
sudo apt install -y libportaudio2
python3 -m venv --system-site-packages ~/voice_venv
source ~/voice_venv/bin/activate
pip install google-genai sounddevice pyyaml "numpy<2"
```

## 3. 전제 — Nav2 (저장한 지도로 자율주행)
음성 명령 "X 가줘" 를 실행하려면 Nav2 가 돌고 있어야 한다:
```bash
# 로봇 Pi: bringup + Nav2 (저장한 lab_map)
ros2 launch turtlebot3_bringup robot.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/maps/lab_map.yaml
# RViz 에서 2D Pose Estimate 로 초기 위치 지정
```
`/navigate_to_pose` 액션 서버가 떠 있으면 준비 완료.

## 4. 웨이포인트 등록
`config/waypoints.yaml` 에 장소 이름 -> map 좌표(x,y,yaw) 기록.
현재 위치 확인: `ros2 run tf2_ros tf2_echo map base_footprint` 로 로봇을 옮겨가며 좌표를 적는다.
별칭(aliases)도 넣으면 Gemini 가 유연하게 매칭.

## 5. 실행
```bash
source /opt/ros/jazzy/setup.bash && source ~/voice_venv/bin/activate
export ROS_DOMAIN_ID=30
python3 ~/D435i/tb3_voice_guide/scripts/voice_guide.py \
  --waypoints ~/D435i/tb3_voice_guide/config/waypoints.yaml
```
"회의실 가줘" 라고 말하면 → Gemini 가 navigate_to("회의실") 호출 → Nav2 목표 전송 → 음성으로 안내.

## 파일
- `scripts/voice_guide.py` — Gemini Live + Nav2 음성 노드
- `config/waypoints.yaml` — 장소 이름 -> 좌표

## 메모
- 모델: `gemini-2.0-flash-live-001` (Live 스트리밍). 바뀌면 코드 상단 MODEL 수정.
- YOLO 로 찾은 객체 위치를 웨이포인트에 추가하면 "냉장고 앞으로 가줘" 도 가능.
- 다음 단계: Nav2 파라미터 튜닝, 웨이포인트 실측, 음성 대화 흐름 다듬기.
