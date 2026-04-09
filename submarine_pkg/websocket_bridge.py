#!/usr/bin/env python3

from geometry_msgs.msg import Vector3
import rclpy
from rclpy.node import Node
import json
import threading
import time
import asyncio
from websockets.asyncio.server import serve

class SocketBridge(Node):
    def __init__(self):
        super().__init__('websocket_bridge')

        self.latest_roll = 0.0
        self.latest_pitch = 0.0
        self.latest_yaw = 0.0
        self.euler_lock = threading.Lock()

        self.client_sockets = set()
        self.clients_lock = threading.Lock()

        self.loop = None  # will be set in _run_ws_server

        self.euler_subscription = self.create_subscription(
            Vector3,
            '/imu/euler_angles',
            self.euler_callback,
            10
        )

        self.host = '0.0.0.0'
        self.port = 8765

        self.ws_thread = threading.Thread(target=self._run_ws_server, daemon=True)
        self.ws_thread.start()

        self.broadcast_thread = threading.Thread(target=self._broadcast_data, daemon=True)
        self.broadcast_thread.start()

        self.get_logger().info(f'WebSocket server started on ws://{self.host}:{self.port}')

    def euler_callback(self, msg: Vector3):
        with self.euler_lock:
            self.latest_roll = msg.x
            self.latest_pitch = msg.y
            self.latest_yaw = msg.z

    def _run_ws_server(self):
        async def handler(websocket):
            addr = websocket.remote_address
            self.get_logger().info(f'WebSocket client connected from {addr}')
            with self.clients_lock:
                self.client_sockets.add(websocket)
            try:
                async for message in websocket:
                    pass
            except Exception as e:
                self.get_logger().info(f'WebSocket error from {addr}: {e}')
            finally:
                with self.clients_lock:
                    self.client_sockets.discard(websocket)
                self.get_logger().info(f'WebSocket client disconnected from {addr}')

        async def main():
            self.loop = asyncio.get_event_loop()
            async with serve(handler, self.host, self.port):
                self.get_logger().info(f'WebSocket listening on ws://{self.host}:{self.port}')
                await asyncio.Future()

        asyncio.run(main())

    def _broadcast_data(self):
        while True:
            try:
                # Wait until event loop is ready
                if self.loop is None:
                    time.sleep(0.1)
                    continue

                with self.euler_lock:
                    current_roll = self.latest_roll
                    current_pitch = self.latest_pitch
                    current_yaw = self.latest_yaw

                boat_data = {
                    "pitch": round(current_pitch, 2),
                    "roll": round(current_roll, 2),
                    "speed": 4.0,
                    "battery_voltage": 11.8,
                    "rudder_angle": 1.0,
                    "yaw": round(current_yaw, 2)
                }

                message = json.dumps(boat_data) + '\n'

                with self.clients_lock:
                    clients_snapshot = list(self.client_sockets)

                for client in clients_snapshot:
                    asyncio.run_coroutine_threadsafe(
                        client.send(message),
                        self.loop
                    )

                time.sleep(0.05)  # 20 Hz

            except Exception as e:
                self.get_logger().error(f'Broadcast error: {e}')
                time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SocketBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
