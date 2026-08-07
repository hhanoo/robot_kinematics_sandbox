# Make robot_kinematics / robot_trajectory importable when running pytest
# from the source tree (colcon install handles this at runtime)
import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_SRC / "robot_kinematics"))
sys.path.insert(0, str(_SRC / "robot_trajectory"))
