"""
Self-Collision Unit Tests
=========================
The capsules must enclose the collision meshes they were fitted to, the
reference and working postures must stay clear, and an obviously folded
posture must be reported.
"""

import math

import numpy as np
import pytest

from robot_kinematics.chain import load_default
from robot_kinematics.collision import (
    check_self_collision,
    load_model,
    place_capsules,
    segment_distance,
    self_collision_pairs,
)

HOME = np.array([0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0])
# Elbow folded fully back: the forearm lies on the upper arm
FOLDED = np.array([0.0, -math.pi / 2, math.pi, 0.0, 0.0, 0.0])


@pytest.fixture(scope="module")
def model():
    return load_model()


class TestSegmentDistance:
    def test_parallel_segments(self):
        d = segment_distance(
            np.array([0.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, 2.0, 0.0]),
            np.array([1.0, 2.0, 0.0]),
        )
        assert d == pytest.approx(2.0)

    def test_crossing_segments_touch(self):
        d = segment_distance(
            np.array([-1.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, -1.0, 0.0]),
            np.array([0.0, 1.0, 0.0]),
        )
        assert d == pytest.approx(0.0, abs=1e-12)

    def test_endpoints_decide_when_projections_fall_outside(self):
        # Both segments lie on the x axis, separated by a gap of 1
        d = segment_distance(
            np.array([0.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]),
            np.array([2.0, 0.0, 0.0]),
            np.array([3.0, 0.0, 0.0]),
        )
        assert d == pytest.approx(1.0)


class TestCapsuleFit:
    def test_every_collision_link_has_a_capsule(self, model):
        assert [c.link for c in model.capsules] == [
            "base_link",
            "link1",
            "link2",
            "link3",
            "link4",
            "link5",
            "link6",
        ]

    def test_capsules_contain_their_meshes(self, model):
        """The fit is only safe if no mesh vertex sticks out of the capsule."""
        pytest.importorskip("xacro")
        import xml.etree.ElementTree as ET

        from robot_kinematics.chain import default_urdf
        from robot_kinematics.collision import PACKAGE_ROOT, _collision_points

        links = {
            el.attrib["name"]: el for el in ET.fromstring(default_urdf()).iter("link")
        }
        placed = dict(load_default().link_frames)

        for cap in model.capsules:
            pts = _collision_points(links[cap.link], PACKAGE_ROOT)
            T = placed[cap.link][1]
            pts = pts @ T[:3, :3].T + T[:3, 3]
            d = np.array([segment_distance(cap.p0, cap.p1, p, p) for p in pts])
            assert d.max() <= cap.radius, f"{cap.link} sticks out of its capsule"

    def test_neighbours_are_not_checked(self, model):
        for i, j in model.pairs:
            assert model.capsules[j].frame - model.capsules[i].frame > 1


class TestSelfCollision:
    def test_zero_pose_is_clear(self):
        assert self_collision_pairs(np.zeros(6)) == []

    def test_home_pose_is_clear(self):
        assert self_collision_pairs(HOME) == []

    def test_folded_elbow_collides(self):
        assert check_self_collision(FOLDED)

    def test_random_near_home_stays_clear(self):
        rng = np.random.default_rng(5)
        for _ in range(20):
            assert not check_self_collision(HOME + rng.uniform(-0.2, 0.2, 6))

    def test_margin_only_widens_the_report(self):
        wide = self_collision_pairs(HOME, margin=0.5)
        assert set(self_collision_pairs(HOME)) <= set(wide)
        assert wide

    def test_capsules_follow_the_posture(self, model):
        """Placement must use FK, not the rest pose."""
        a = place_capsules(np.zeros(6), capsules=model.capsules)
        b = place_capsules(HOME, capsules=model.capsules)
        assert not np.allclose(a, b)
        np.testing.assert_allclose(a[0], b[0], atol=1e-12)  # base_link cannot move
