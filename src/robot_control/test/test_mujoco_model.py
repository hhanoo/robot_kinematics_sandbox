"""
MuJoCo Model Unit Tests
=======================
Every check here guards a failure that once passed silently: the model
still compiled and stepped, and only the numbers were wrong.
"""

import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")
if shutil.which("xacro") is None:
    pytest.skip("the URDF is expanded with xacro", allow_module_level=True)

from robot_control.mujoco_model import build_model  # noqa: E402
from robot_kinematics.chain import XACRO_PATH, default_urdf, load_default  # noqa: E402

HOME = np.array([0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0])
CONFIG = Path(__file__).resolve().parents[1] / "config" / "mujoco_model.yaml"


@pytest.fixture(scope="module")
def chain():
    return load_default()


@pytest.fixture(scope="module")
def model(chain):
    return build_model(XACRO_PATH, XACRO_PATH.parents[1], chain.joint_names, CONFIG)


def settle(model, target, seconds):
    """Start at HOME, command target, and return where the joints end up."""
    data = mujoco.MjData(model)
    data.qpos[:] = HOME
    data.ctrl[:] = target
    while data.time < seconds:
        mujoco.mj_step(model, data)
    return data.qpos.copy()


class TestModel:
    def test_masses_match_urdf(self, model, chain):
        # Recomputing inertia from meshes would change these
        root = ET.fromstring(default_urdf())
        masses = {
            link.get("name"): float(link.find("inertial/mass").get("value"))
            for link in root.iter("link")
            if link.find("inertial") is not None
        }
        # The root link is fused into the static world body
        masses.pop(chain.base_link)
        np.testing.assert_allclose(
            sorted(model.body_mass[model.body_mass > 0]), sorted(masses.values())
        )

    def test_one_servo_per_joint(self, model, chain):
        # URDF <mujoco><actuator> is silently ignored
        joint = mujoco.mjtObj.mjOBJ_JOINT
        driven = [
            mujoco.mj_id2name(model, joint, model.actuator_trnid[i, 0])
            for i in range(model.nu)
        ]
        assert driven == list(chain.joint_names)

    def test_meshes_are_drawable(self, model):
        # discardvisual drops every non-colliding geom
        meshes = model.geom_type == mujoco.mjtGeom.mjGEOM_MESH
        visible = model.geom_group < 3
        assert np.any(meshes & visible)


class TestServo:
    def test_shoulder_moves_while_the_rest_hold(self, model):
        # A base_link contact once pinned the shoulder
        target = HOME.copy()
        target[0] = 0.5
        np.testing.assert_allclose(settle(model, target, 3.0), target, atol=1e-3)
