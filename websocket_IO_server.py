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
route_map = {
    'run_route_1': 'ready.py',
    'run_route_2': 'go_A.py',
    'run_route_3': 'go_B.py',
    'run_route_4': 'go_C.py',
    'run_route_5': 'return_A.py',
    'run_route_6': 'return_B.py',
    'run_route_7': 'return_C.py',
}

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
    logging.info(f"{sid}로부터 메시지 수신: {data}")
    
    if data == 'ping':
        await sio.emit('message', 'pong', room=sid)
        return
    
    if data.startswith('run_route_'):
        script_name = route_map.get(data)
        if script_name:
            # 해당 Python 스크립트를 비동기로 실행
            await asyncio.create_subprocess_exec(
                '/usr/bin/python3',
                f'/home/jinyoung/zeta_ws/src/zeta2_edu_autonomous/zeta2_navigation/maps/office/{script_name}'
            )

# 웹 서버 시작
if __name__ == "__main__":
    logging.info("Socket.IO 서버 시작: http://192.168.1.121:8765")
    web.run_app(app, host='192.168.1.121', port=8765)
