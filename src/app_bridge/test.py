import websocket
import json
import time

def send_navigation_goal(target_x, target_y, theta=0.0):
    # --- 1. 환경 설정 (C++ 노드 설정과 동일해야 함) ---
    uri = "ws://localhost:5000"  # C++ 노드에서 설정한 포트
    world_max_x = 550
    resolution = 0.002           # 1픽셀당 0.002m
    
    # --- 2. 좌표 컨버팅 로직 (Pixel -> Meter) ---
    half_size = (world_max_x * resolution) / 2.0  # 0.55m
    
    # x: 픽셀 -> 미터 변환 후 중심 이동
    nav2_x = (target_x * resolution) - half_size
    
    # y: 픽셀 뒤집기 -> 미터 변환 후 중심 이동
    nav2_y = ((world_max_x - target_y) * resolution) - half_size

    # --- 3. JSON 메시지 구성 ---
    # C++ 노드의 on_client_message에서 요구하는 형식
    payload = {
        "type": "navigate",
        "x": nav2_x,
        "y": nav2_y,
        "theta": theta
    }

    # --- 4. WebSocket 전송 ---
    try:
        ws = websocket.create_connection(uri)
        json_data = json.dumps(payload)
        ws.send(json_data)
        
        print(f"📡 [송신] 앱 픽셀 좌표: ({target_x}, {target_y})")
        print(f"🤖 [변환] 로봇 미터 좌표: ({nav2_x:.3f}, {nav2_y:.3f})")
        print(f"✅ 메시지 전송 완료: {json_data}")
        
        ws.close()
    except Exception as e:
        print(f"❌ 연결 실패: {e}")

if __name__ == "__main__":
    # 예: 앱 화면에서 400, 100 지점을 터치했을 때
    # (실제 로봇은 원점 기준 오른쪽 위 어딘가로 이동하게 됨)
    send_navigation_goal(400, 100, 0.0)