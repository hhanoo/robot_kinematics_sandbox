# Make robot_kinematics importable when running pytest from the source tree
# (colcon install handles this at runtime)
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "robot_kinematics"))
