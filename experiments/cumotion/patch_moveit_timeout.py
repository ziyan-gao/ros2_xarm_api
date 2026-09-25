"""Patch the pinned NVIDIA MoveIt plugin's hard-coded five-second wait."""
from pathlib import Path


path = Path('/opt/cumotion_plugin_src/isaac_ros_cumotion_moveit/src/cumotion_interface.cpp')
source = path.read_text(encoding='utf-8')
old = 'constexpr unsigned kTimeoutIntervalInSeconds = 5;'
new = 'constexpr unsigned kTimeoutIntervalInSeconds = 75;'
if source.count(old) != 1:
    raise RuntimeError('Unexpected NVIDIA plugin source; refusing blind timeout patch')
path.write_text(source.replace(old, new), encoding='utf-8')
