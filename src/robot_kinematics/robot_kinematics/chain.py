"""
Kinematic Chain
===============
Serial chain built from a URDF, following the KDL segment model: the
base-to-tip path is reduced to one movable joint per segment, with the
fixed joints around it folded into the transforms before and after.

Per segment the chain keeps:
- pre:  previous link frame -> joint frame (fixed origins + joint origin)
- axis: rotation axis expressed in the joint frame
- post: after the rotation -> child link frame (trailing fixed origins)

Parsing uses only the standard library, so the core stays free of ROS.
Expanding a xacro does need the xacro module, and that happens once in
load_default().
"""

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np

XACRO_PATH = (
    Path(__file__).resolve().parents[2]
    / "robot_description"
    / "urdf"
    / "ur10e.urdf.xacro"
)

MOVABLE = ("revolute", "continuous")


# =========================================================
# Chain data
# =========================================================
@dataclass(frozen=True)
class Segment:
    """One movable joint with the fixed transforms folded around it."""

    name: str
    pre: np.ndarray
    axis: np.ndarray
    post: np.ndarray
    lower: float
    upper: float
    velocity: float


@dataclass(frozen=True)
class Chain:
    """Segments from base to tip, plus the link names they end at."""

    segments: tuple
    link_names: tuple

    def __len__(self):
        return len(self.segments)

    @property
    def limits(self):
        """(n, 2) lower/upper position limits [rad]."""
        return np.array([[s.lower, s.upper] for s in self.segments])

    @property
    def max_velocity(self):
        """(n,) joint velocity limits [rad/s]."""
        return np.array([s.velocity for s in self.segments])

    @property
    def joint_names(self):
        """Movable joint names in chain order."""
        return tuple(s.name for s in self.segments)


# =========================================================
# URDF element helpers
# =========================================================
def rpy_matrix(roll, pitch, yaw):
    """URDF rpy convention: Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_rotation(axis, angle):
    """Rodrigues rotation of angle about a unit axis."""
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    k = 1.0 - c
    return np.array(
        [
            [c + x * x * k, x * y * k - z * s, x * z * k + y * s],
            [y * x * k + z * s, c + y * y * k, y * z * k - x * s],
            [z * x * k - y * s, z * y * k + x * s, c + z * z * k],
        ]
    )


def _origin(joint):
    """4x4 transform of a joint's <origin>; identity when absent."""
    T = np.eye(4)
    elem = joint.find("origin")
    if elem is None:
        return T
    xyz = [float(v) for v in elem.attrib.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in elem.attrib.get("rpy", "0 0 0").split()]
    T[:3, :3] = rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def _axis(joint):
    """Unit rotation axis of a joint; URDF defaults to x."""
    elem = joint.find("axis")
    raw = elem.attrib["xyz"] if elem is not None else "1 0 0"
    axis = np.array([float(v) for v in raw.split()], dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0.0:
        raise ValueError(f"joint '{joint.attrib['name']}' has a zero axis")
    return axis / norm


def _limits(joint):
    """(lower, upper, velocity); continuous joints get an unbounded range."""
    elem = joint.find("limit")
    if elem is None:
        return -np.inf, np.inf, np.inf
    return (
        float(elem.attrib.get("lower", -np.inf)),
        float(elem.attrib.get("upper", np.inf)),
        float(elem.attrib.get("velocity", np.inf)),
    )


# =========================================================
# Chain construction
# =========================================================
def _endpoints(root, base, tip):
    """Resolve base and tip links, defaulting to the only root and leaf."""
    links = {el.attrib["name"] for el in root.iter("link")}
    children = {j.find("child").attrib["link"] for j in root.iter("joint")}
    parents = {j.find("parent").attrib["link"] for j in root.iter("joint")}

    if base is None:
        roots = links - children
        if len(roots) != 1:
            raise ValueError(f"cannot pick a base link, candidates: {sorted(roots)}")
        base = roots.pop()
    if tip is None:
        leaves = links - parents
        if len(leaves) != 1:
            raise ValueError(f"cannot pick a tip link, candidates: {sorted(leaves)}")
        tip = leaves.pop()
    return base, tip


def _path(root, base, tip):
    """Joint elements from base to tip, walking parents back from tip."""
    by_child = {j.find("child").attrib["link"]: j for j in root.iter("joint")}
    chain, link = [], tip
    while link != base:
        if link not in by_child:
            raise ValueError(f"'{tip}' is not connected to '{base}' ('{link}' is free)")
        joint = by_child[link]
        chain.append(joint)
        link = joint.find("parent").attrib["link"]
    chain.reverse()
    return chain


def from_urdf(urdf_xml, base=None, tip=None):
    """Build a Chain from URDF text (already expanded, no xacro syntax)."""
    root = ET.fromstring(urdf_xml)
    base, tip = _endpoints(root, base, tip)

    # Group each movable joint with the fixed joints after it
    groups, leading = [], []
    trailing = leading
    for joint in _path(root, base, tip):
        if joint.attrib["type"] in MOVABLE:
            groups.append([joint, []])
            trailing = groups[-1][1]
        elif joint.attrib["type"] == "fixed":
            trailing.append(joint)
        else:
            kind = joint.attrib["type"]
            raise ValueError(f"joint '{joint.attrib['name']}' has type '{kind}'")

    if not groups:
        raise ValueError(f"no movable joint between '{base}' and '{tip}'")

    segments, link_names = [], [base]
    pending = leading
    for movable, fixed in groups:
        pre = np.eye(4)
        for j in pending:
            pre = pre @ _origin(j)
        pre = pre @ _origin(movable)

        post = np.eye(4)
        for j in fixed:
            post = post @ _origin(j)

        lower, upper, velocity = _limits(movable)
        segments.append(
            Segment(
                name=movable.attrib["name"],
                pre=pre,
                axis=_axis(movable),
                post=post,
                lower=lower,
                upper=upper,
                velocity=velocity,
            )
        )
        last = fixed[-1] if fixed else movable
        link_names.append(last.find("child").attrib["link"])
        pending = []

    return Chain(segments=tuple(segments), link_names=tuple(link_names))


def from_urdf_file(path, base=None, tip=None):
    """Build a Chain from a plain .urdf file on disk."""
    return from_urdf(Path(path).read_text(), base, tip)


@lru_cache(maxsize=None)
def load_default():
    """Chain for the bundled robot_description xacro, expanded once.

    Expanding needs the xacro module (ros-humble-xacro). Callers outside a
    ROS environment should build a Chain from a URDF string instead.
    """
    try:
        import xacro
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise RuntimeError(
            "the default chain expands ur10e.urdf.xacro and needs the 'xacro' "
            "module; pass a Chain built with from_urdf() instead"
        ) from exc
    return from_urdf(xacro.process_file(str(XACRO_PATH)).toxml())
