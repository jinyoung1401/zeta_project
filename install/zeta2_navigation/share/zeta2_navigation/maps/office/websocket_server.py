import asyncio
import websockets
import json

class WebSocketServer:
    def __init__(self, host='localhost', port=9090):
        self.host = host
        self.port = port

    async def handler(self, websocket, path):
        async for message in websocket:
            data = json.loads(message)
            # 원하는 작업을 수행 (예: 경로 설정)
            if data['op'] == 'publish' and data['topic'] == '/goal_pose':
                # goal_pose 처리 코드 추가
                print(f"목표 지점: {data['msg']['pose']}")

    def start(self):
        start_server = websockets.serve(self.handler, self.host, self.port)
        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()

if __name__ == '__main__':
    server = WebSocketServer()
    server.start()
