import asyncio
import websockets
import subprocess
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)

async def run_navigation(websocket, path):
    try:
        async for message in websocket:
            logging.info(f"Received message: {message}")
            if message == 'ping':
                await websocket.send('pong')
                continue
            
            if message.startswith('run_route_'):
                route_map = {
                    'run_route_1': 'ready.py',
                    'run_route_2': 'go_A.py',
                    'run_route_3': 'go_B.py',
                    'run_route_4': 'go_C.py',
                    'run_route_5': 'return_A.py',
                    'run_route_6': 'return_B.py',
                    'run_route_7': 'return_C.py',
                }
                script_name = route_map.get(message)
                if script_name:
                    await asyncio.create_subprocess_exec(
                        '/usr/bin/python3',
                        f'/home/jinyoung/zeta_ws/src/zeta2_edu_autonomous/zeta2_navigation/maps/office/{script_name}'
                    )
    except websockets.ConnectionClosed:
        logging.warning("WebSocket connection closed.")


async def main():
    async with websockets.serve(run_navigation, "192.168.1.121", 8765, ping_interval=10):  # 핑 간격을 5초로 설정
        logging.info("WebSocket server started on ws://192.168.1.121:8765")
        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    asyncio.run(main())
