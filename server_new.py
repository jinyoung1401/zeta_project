import asyncio
import socketio
import subprocess
import logging
from aiohttp import web

# 로깅 설정
logging.basicConfig(level=logging.INFO)

# Socket.IO 서버 생성
sio = socketio.AsyncServer(async_mode='aiohttp')
app = web.Application()
sio.attach(app)

# 라우트 매핑 정의
route_messages = {
    'ready.py': '식당으로 이동 중',
    'go_A.py': '현동홀로 이동 중',
    'go_B.py': '느헤미야홀로 이동 중',
    'go_C.py': '오석관으로 이동 중',
    'return_A.py': '현동홀에서 복귀 중',
    'return_B.py': '느헤미야홀에서 복귀 중',
    'return_C.py': '오석관에서 복귀 중',
}

# 라우트 매핑의 메시지를 키로 하는 사전 추가
route_map = {
    'go_to_store': 'ready.py',
    'go_to_hyeondong': 'go_A.py',
    'go_to_nehemiah': 'go_B.py',
    'go_to_oseok': 'go_C.py',
    'return_from_hyeondong': 'return_A.py',
    'return_from_nehemiah': 'return_B.py',
    'return_from_oseok': 'return_C.py',
}

async def notify_progress(sid, script_name, waypoint_number):
    message = route_messages.get(script_name, '이동 중')
    await sio.emit('navigation_status', f'{message} (좌표 {waypoint_number})', room=sid)

async def notify_completion(sid):
    await sio.emit('navigation_status', '모든 좌표를 성공적으로 방문했습니다.', room=sid)

# 클라이언트 연결 처리
@sio.event
async def connect(sid, environ):
    logging.info(f"클라이언트 {sid} 연결됨.")

# 클라이언트 연결 해제 처리
@sio.event
async def disconnect(sid):
    logging.info(f"클라이언트 {sid} 연결 해제됨.")

# 클라이언트 메시지 처리
@sio.event
async def message(sid, data):
    if data.startswith('go_to_') or data.startswith('return_from_'):
        script_name = route_map.get(data)
        if script_name:
            try:
                # 해당 Python 스크립트를 비동기로 실행
                process = await asyncio.create_subprocess_exec(
                    '/usr/bin/python3',
                    f'/home/jinyoung/zeta_ws/src/zeta2_edu_autonomous/zeta2_navigation/maps/office/{script_name}'
                )
                
                # 각 좌표에 대한 도착 알림 설정
                waypoint_number = 1  # 예시: 첫 번째 좌표
                await notify_progress(sid, script_name, waypoint_number)
                
                # 스크립트의 진행 상황을 감시
                await process.wait()
                await notify_completion(sid)  # 모든 좌표 방문 후 알림

            except Exception as e:
                logging.error(f"스크립트 실행 중 오류 발생: {e}")
                await sio.emit('navigation_status', f'오류 발생: {e}', room=sid)

# 웹 서버 시작
if __name__ == "__main__":
    ipAddress = '127.0.0.1'
    logging.info(f"Socket.IO 서버 시작: http://{ipAddress}:8765")
    web.run_app(app, host=ipAddress, port=8765)
