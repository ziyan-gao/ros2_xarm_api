"""Offline test: reserve participant ports 0..100; the configured node must start.

Run in a --network none container with ROS sourced, never on the robot network.
"""
import os
from pathlib import Path
import socket
import subprocess
import sys

config = Path(sys.argv[1]).resolve()
sockets = []
try:
    for index in range(101):
        for port in (7410+2*index, 7411+2*index):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('0.0.0.0', port))
            sockets.append(sock)
    env = dict(os.environ, RMW_IMPLEMENTATION='rmw_cyclonedds_cpp', ROS_DOMAIN_ID='0',
               CYCLONEDDS_URI=config.as_uri())
    code = "import rclpy; rclpy.init(); n=rclpy.create_node('capacity_probe'); n.destroy_node(); rclpy.shutdown()"
    result = subprocess.run([sys.executable, '-c', code], env=env, capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stderr
    print('PASS: DDS node starts with participant ports 0..100 occupied')
finally:
    for sock in sockets:
        sock.close()
