#!/usr/bin/env python3

from geometry_msgs.msg import Vector3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import socket
import json
import threading
import time

class SocketBridge(Node):
    def __init__(self):
        super().__init__('socket_bridge')
        
        # Initialize pitch value
        self.latest_roll = 0.0
        self.latest_pitch = 0.0
        self.latest_yaw = 0.0
        self.euler_lock = threading.Lock()
        
        # Connected clients
        self.client_sockets = []
        self.clients_lock = threading.Lock()
        
        # Create subscription to IMU euler angles topic
        self.euler_subscription = self.create_subscription(
            Vector3,
            '/imu/euler_angles',
            self.euler_callback,
            10
        )
        
        # Socket server configuration
        self.host = '0.0.0.0'
        self.port = 8765
        
        # Start server
        self.server_thread = threading.Thread(target=self._run_server, daemon=True)
        self.server_thread.start()
        
        # Start broadcast thread
        self.broadcast_thread = threading.Thread(target=self._broadcast_data, daemon=True)
        self.broadcast_thread.start()
        
        self.get_logger().info(f'Socket server started on {self.host}:{self.port}')

    def euler_callback(self, msg: Vector3):
        """Callback for IMU euler angle messages"""
        with self.euler_lock:
            self.latest_roll = msg.x
            self.latest_pitch = msg.y
            self.latest_yaw = msg.z

    def _run_server(self):
        """Run TCP server"""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((self.host, self.port))
        server_socket.listen(5)
        
        self.get_logger().info(f'Server listening on {self.host}:{self.port}')
        
        while True:
            try:
                client_socket, addr = server_socket.accept()
                self.get_logger().info(f'Client connected from {addr}')
                
                # Add to clients list
                with self.clients_lock:
                    self.client_sockets.append(client_socket)
                
                # Start thread to handle this client
                client_thread = threading.Thread(
                    target=self._handle_client, 
                    args=(client_socket, addr),
                    daemon=True
                )
                client_thread.start()
                
            except Exception as e:
                self.get_logger().error(f'Server error: {e}')

    def _handle_client(self, client_socket, addr):
        """Handle individual client"""
        try:
            while True:
                # Just keep the connection alive
                data = client_socket.recv(1024)
                if not data:
                    break
        except Exception as e:
            self.get_logger().info(f'Client {addr} disconnected: {e}')
        finally:
            # Remove from clients list
            with self.clients_lock:
                if client_socket in self.client_sockets:
                    self.client_sockets.remove(client_socket)
            client_socket.close()

    def _broadcast_data(self):
        """Broadcast pitch data to all clients at 20Hz"""
        while True:
            try:
                # Get current orientation
                with self.euler_lock:
                    current_roll = self.latest_roll
                    current_pitch = self.latest_pitch
                    current_yaw = self.latest_yaw

                # Create complete message with all data from boat
                boat_data = {
                    "pitch": round(current_pitch, 2),
                    "roll": round(current_roll, 2),
                    "speed": 4.0,  # Placeholder for speed
                    "battery_voltage": 11.8,  # Placeholder for battery voltage
                    "rudder_angle": 1.0,  # Placeholder for rudder angle
                    "yaw": round(current_yaw, 2)
                }

                message = json.dumps(boat_data) + '\n'

                # Send to all clients
                with self.clients_lock:
                    dead_clients = []
                    for client in self.client_sockets[:]:  # Copy list
                        try:
                            client.send(message.encode('utf-8'))
                        except Exception as e:
                            dead_clients.append(client)
                    
                    # Remove dead clients
                    for client in dead_clients:
                        if client in self.client_sockets:
                            self.client_sockets.remove(client)
                        try:
                            client.close()
                        except:
                            pass
                
                # Sleep for 50ms (20Hz)
                time.sleep(0.05)
                
            except Exception as e:
                self.get_logger().error(f'Broadcast error: {e}')
                time.sleep(0.1)

def main(args=None):
    """Main entry point"""
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