"""
Self-Collision Check
====================
Each link is one capsule (segment + radius) fitted to its <collision>
mesh, and two links collide when the closest distance between their
segments falls below the sum of the radii. Mesh-to-mesh distance costs
far more than a 50 Hz loop can afford, so the mesh is approximated once
and only segment distances run per posture.

The capsule axis is whichever principal component gives the smallest
enclosing circle. A link whose extents are nearly equal has no single
longest direction, so all three are tried; taking only the first misses
the tighter fit by more than a centimetre on the shoulder.

Radii are rounded up to the millimetre, so the capsule always contains
the mesh.
"""

import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np

from robot_kinematics.chain import default_urdf, load_default, rpy_matrix
from robot_kinematics.fk import fk_frames

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
MESH_PREFIX = "package://"
CENTER_ITERS = 300


# =========================================================
# Capsule data
# =========================================================
@dataclass(frozen=True)
class Capsule:
    """Segment endpoints and radius, in the frame the link hangs from."""

    link: str
    frame: int
    p0: np.ndarray
    p1: np.ndarray
    radius: float


# =========================================================
# Mesh loading
# =========================================================
def stl_vertices(path):
    """Triangle vertices of a binary STL, in the file's own frame."""
    blob = Path(path).read_bytes()
    count = struct.unpack("<I", blob[80:84])[0]
    record = np.dtype([("normal", "<3f4"), ("corners", "<9f4"), ("attr", "<u2")])
    tris = np.frombuffer(blob, dtype=record, count=count, offset=84)
    return tris["corners"].reshape(-1, 3).astype(float)


def _resolve(filename, mesh_root):
    """Turn a package:// mesh URI into a path under mesh_root."""
    if not filename.startswith(MESH_PREFIX):
        return Path(filename)
    return Path(mesh_root) / filename[len(MESH_PREFIX) :]


def _collision_points(link, mesh_root):
    """Collision mesh vertices of one URDF link, placed in the link frame."""
    collision = link.find("collision")
    if collision is None:
        return None
    mesh = collision.find("geometry/mesh")
    if mesh is None:
        return None

    R, t = np.eye(3), np.zeros(3)
    origin = collision.find("origin")
    if origin is not None:
        rpy = [float(v) for v in origin.attrib.get("rpy", "0 0 0").split()]
        xyz = [float(v) for v in origin.attrib.get("xyz", "0 0 0").split()]
        R, t = rpy_matrix(*rpy), np.array(xyz)

    pts = stl_vertices(_resolve(mesh.attrib["filename"], mesh_root))
    return pts @ R.T + t


# =========================================================
# Capsule fit
# =========================================================
def frame_from_z(axis):
    """Shortest rotation taking +z onto axis."""
    z = axis / np.linalg.norm(axis)
    v = np.cross([0.0, 0.0, 1.0], z)
    s = np.linalg.norm(v)
    if s < 1e-12:
        return np.eye(3) if z[2] > 0 else np.diag([1.0, -1.0, -1.0])
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * (1.0 - z[2]) / (s * s)


def enclosing_circle(pts, iters=CENTER_ITERS):
    """Approximate 1-center of 2D points (Badoiu-Clarkson)."""
    c = pts.mean(axis=0)
    for k in range(iters):
        c = c + (pts[np.argmax(((pts - c) ** 2).sum(axis=1))] - c) / (k + 2)
    return c, float(np.sqrt(((pts - c) ** 2).sum(axis=1).max()))


def _capsule_for_axis(pts, axis, iters=CENTER_ITERS):
    """Tightest capsule for a fixed axis: (p0, p1, radius)."""
    R = frame_from_z(axis)
    local = pts @ R
    center, radius = enclosing_circle(local[:, :2], iters)
    lo, hi = local[:, 2].min(), local[:, 2].max()
    return (
        R @ np.array([center[0], center[1], lo]),
        R @ np.array([center[0], center[1], hi]),
        radius,
    )


def fit_capsule(pts):
    """Bounding capsule of a point cloud as (p0, p1, radius)."""
    # 1. Try every principal axis, not just the longest
    _, _, principal = np.linalg.svd(pts - pts.mean(axis=0), full_matrices=False)
    best = min((_capsule_for_axis(pts, v) for v in principal), key=lambda c: c[2])
    p0, p1, radius = best
    return p0, p1, np.ceil(radius * 1000.0) / 1000.0


# =========================================================
# Model assembly
# =========================================================
def build_capsules(urdf_xml=None, chain=None, mesh_root=PACKAGE_ROOT):
    """Capsules for every link on the chain that carries a collision mesh."""
    if urdf_xml is None:
        urdf_xml = default_urdf()
    if chain is None:
        chain = load_default()

    root = ET.fromstring(urdf_xml)
    placed = dict(chain.link_frames)

    capsules = []
    for link in root.iter("link"):
        name = link.attrib["name"]
        if name not in placed:
            continue
        pts = _collision_points(link, mesh_root)
        if pts is None:
            continue

        index, T = placed[name]
        p0, p1, radius = fit_capsule(pts)
        capsules.append(
            Capsule(
                link=name,
                frame=index,
                p0=T[:3, :3] @ p0 + T[:3, 3],
                p1=T[:3, :3] @ p1 + T[:3, 3],
                radius=radius,
            )
        )
    return tuple(capsules)


def check_pairs(capsules, chain=None):
    """Capsule index pairs worth testing.

    Neighbours on the chain are joined by a joint and always touch. Pairs
    that still overlap at q = 0 overlap because the capsules are coarser
    than the links, not because the robot can fold that way, and a pair
    that reports a collision in the URDF's own reference posture can never
    report a real one.
    """
    ends = place_capsules(np.zeros(len(chain or load_default())), chain, capsules)
    kept = []
    for i in range(len(capsules)):
        for j in range(i + 1, len(capsules)):
            if capsules[j].frame - capsules[i].frame <= 1:
                continue
            d = segment_distance(ends[i, 0], ends[i, 1], ends[j, 0], ends[j, 1])
            if d < capsules[i].radius + capsules[j].radius:
                continue
            kept.append((i, j))
    return tuple(kept)


@dataclass(frozen=True)
class CollisionModel:
    """Capsules and the pairs worth testing, both derived from the URDF."""

    capsules: tuple
    pairs: tuple


def build_model(urdf_xml=None, chain=None, mesh_root=PACKAGE_ROOT):
    """Fit every capsule and pick the pairs, straight from the URDF."""
    capsules = build_capsules(urdf_xml, chain, mesh_root)
    return CollisionModel(capsules=capsules, pairs=check_pairs(capsules, chain))


@lru_cache(maxsize=None)
def load_model():
    """Collision model for the bundled robot, built once and cached."""
    return build_model()


# =========================================================
# Distance and collision test
# =========================================================
def segment_distance(p0, p1, q0, q1):
    """Closest distance between segments p0-p1 and q0-q1 (Ericson 5.1.9)."""
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    eps = 1e-12
    if a <= eps and e <= eps:
        return float(np.linalg.norm(r))
    if a <= eps:
        s, t = 0.0, np.clip(f / e, 0.0, 1.0)
    else:
        c = d1 @ r
        if e <= eps:
            s, t = np.clip(-c / a, 0.0, 1.0), 0.0
        else:
            b = d1 @ d2
            denom = a * e - b * b
            s = np.clip((b * f - c * e) / denom, 0.0, 1.0) if denom > eps else 0.0
            t = (b * s + f) / e
            # Clamp t back into the segment, then redo s for that t
            if t < 0.0:
                s, t = np.clip(-c / a, 0.0, 1.0), 0.0
            elif t > 1.0:
                s, t = np.clip((b - c) / a, 0.0, 1.0), 1.0
    return float(np.linalg.norm((p0 + d1 * s) - (q0 + d2 * t)))


def place_capsules(q, chain=None, capsules=None):
    """Capsule endpoints in the base frame, shape (m, 2, 3)."""
    if capsules is None:
        capsules = load_model().capsules
    frames = fk_frames(q, chain)
    out = np.empty((len(capsules), 2, 3))
    for i, cap in enumerate(capsules):
        R, p = frames[cap.frame][:3, :3], frames[cap.frame][:3, 3]
        out[i, 0] = R @ cap.p0 + p
        out[i, 1] = R @ cap.p1 + p
    return out


def self_collision_pairs(q, chain=None, model=None, margin=0.0):
    """Link name pairs whose capsules are closer than the radii allow."""
    if model is None:
        model = load_model()
    capsules = model.capsules
    ends = place_capsules(q, chain, capsules)

    hits = []
    for i, j in model.pairs:
        d = segment_distance(ends[i, 0], ends[i, 1], ends[j, 0], ends[j, 1])
        if d < capsules[i].radius + capsules[j].radius + margin:
            hits.append((capsules[i].link, capsules[j].link))
    return hits


def check_self_collision(q, chain=None, model=None, margin=0.0):
    """True when any checked link pair collides."""
    return bool(self_collision_pairs(q, chain, model, margin))
