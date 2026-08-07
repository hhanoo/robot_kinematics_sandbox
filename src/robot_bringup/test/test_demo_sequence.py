"""
Demo Sequence Unit Tests
========================
Whole-sequence continuity plus per-segment geometry checked through FK.
"""

import numpy as np

from robot_bringup.demo_sequence import build_demo_sequence
from robot_kinematics.fk import fk

DT = 0.02


def _segment(seq, name):
    for s in seq.segments:
        if s.name == name:
            return s
    raise AssertionError(f"segment {name} not found")


class TestSequenceGlobal:
    def test_shape_and_finite(self):
        seq = build_demo_sequence(dt=DT)
        assert seq.q.ndim == 2 and seq.q.shape[1] == 6
        assert np.all(np.isfinite(seq.q))
        assert seq.dt == DT

    def test_starts_at_zero_pose(self):
        seq = build_demo_sequence(dt=DT)
        np.testing.assert_allclose(seq.q[0], np.zeros(6), atol=1e-9)

    def test_ends_at_home(self):
        seq = build_demo_sequence(dt=DT)
        home = _segment(seq, "home")
        np.testing.assert_allclose(seq.q[-1], seq.q[home.end - 1], atol=1e-6)

    def test_continuity_no_joint_jumps(self):
        seq = build_demo_sequence(dt=DT)
        assert np.max(np.abs(np.diff(seq.q, axis=0))) < 0.1

    def test_segments_cover_whole_sequence(self):
        seq = build_demo_sequence(dt=DT)
        assert seq.segments[0].start == 0
        assert seq.segments[-1].end == len(seq.q)
        for a, b in zip(seq.segments[:-1], seq.segments[1:]):
            assert a.end == b.start


class TestSegmentGeometry:
    def test_line_segment_is_straight(self):
        seq = build_demo_sequence(dt=DT)
        seg = _segment(seq, "line")
        pts = np.array([fk(q)[:3, 3] for q in seq.q[seg.start : seg.end]])
        p0, p1 = pts[0], pts[-1]
        d = (p1 - p0) / np.linalg.norm(p1 - p0)
        # Distance of every point from the p0-p1 line
        offsets = (pts - p0) - np.outer((pts - p0) @ d, d)
        assert np.max(np.linalg.norm(offsets, axis=1)) < 1e-3

    def test_circle_segment_stays_on_circle(self):
        seq = build_demo_sequence(dt=DT)
        seg = _segment(seq, "circle")
        pts = np.array([fk(q)[:3, 3] for q in seq.q[seg.start : seg.end]])
        # Full circle: centroid = center, radii must be constant
        center = pts.mean(axis=0)
        radii = np.linalg.norm(pts - center, axis=1)
        assert radii.std() < 1e-3
        assert radii.mean() > 0.03

    def test_circle_keeps_orientation(self):
        seq = build_demo_sequence(dt=DT)
        seg = _segment(seq, "circle")
        R0 = fk(seq.q[seg.start])[:3, :3]
        for q in seq.q[seg.start : seg.end : 10]:
            R = fk(q)[:3, :3]
            np.testing.assert_allclose(R, R0, atol=1e-3)
