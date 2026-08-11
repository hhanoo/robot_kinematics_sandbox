"""
Quaternion Conversion Tests
===========================
Roundtrip against rotation matrices from the FK core.
"""

import math

import numpy as np

from robot_control.conversions import matrix_to_quat, quat_to_matrix


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class TestQuatMatrix:
    def test_identity(self):
        np.testing.assert_allclose(quat_to_matrix(0, 0, 0, 1), np.eye(3), atol=1e-12)

    def test_quarter_turn_z(self):
        # q = (0, 0, sin(45deg), cos(45deg)) is a +90deg rotation about z
        s = math.sin(math.pi / 4)
        c = math.cos(math.pi / 4)
        np.testing.assert_allclose(
            quat_to_matrix(0, 0, s, c), _rot_z(math.pi / 2), atol=1e-12
        )

    def test_roundtrip_random(self):
        rng = np.random.default_rng(3)
        for _ in range(20):
            # Random rotation via QR keeps det=+1 after correction
            A = rng.normal(size=(3, 3))
            Q, _ = np.linalg.qr(A)
            if np.linalg.det(Q) < 0:
                Q[:, 0] = -Q[:, 0]
            x, y, z, w = matrix_to_quat(Q)
            np.testing.assert_allclose(quat_to_matrix(x, y, z, w), Q, atol=1e-9)
