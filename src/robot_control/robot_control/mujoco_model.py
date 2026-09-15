#!/usr/bin/env python3
"""
MuJoCo Model Builder
====================
Turns the same URDF the ROS nodes use into a MuJoCo model. Three things
the URDF cannot express are added on the way through:

- armature   : gearbox inertia reflected to the joint, without which the
               wrist axes (inertia ~2e-4) diverge at the 2 ms timestep
- gravcomp   : gravity compensation, as a real UR controller does
- actuators  : URDF has no actuator concept, so position servos are
               declared on the MuJoCo side

Self-collision is switched off, matching Gazebo, because motion_server
already screens every trajectory with the capsule model. MuJoCo would
otherwise pin the shoulder: it skips contact filtering when the parent is
the world body, and the DH split puts a virtual link between base_link
and link1 so the pair never looks like parent and child.

The model is edited through MjSpec rather than XML text. Saving to MJCF
and reloading drops every <inertial>, which makes MuJoCo recompute mass
and inertia from the collision meshes.

MuJoCo also cannot resolve package:// URIs, so mesh paths are rewritten
to the share directory before parsing.
"""

import subprocess
import tempfile
from pathlib import Path

import mujoco
import yaml

# =========================================================
# Model parameters
# =========================================================
# Overridden by config/mujoco_model.yaml
DEFAULTS = {
    "armature": 0.1,  # [kg m^2] reflected rotor inertia
    "kp": 5000.0,  # [N m / rad]
    "kv": 100.0,  # [N m s / rad]
    "gravity_compensation": True,
}
VISUAL_GROUP = 2  # MuJoCo draws groups 0-2 and hides 3 and above
COLLISION_GROUP = 3


def load_config(path):
    """Settings from config/mujoco_model.yaml, falling back to DEFAULTS."""
    config = dict(DEFAULTS)
    if path and Path(path).is_file():
        loaded = yaml.safe_load(Path(path).read_text()) or {}
        config.update(loaded.get("mujoco_model", {}))
    return config


# =========================================================
# URDF -> MuJoCo
# =========================================================
def urdf_for_mujoco(xacro_path, share_dir):
    """Expand the xacro and point every mesh URI at the share directory."""
    urdf = subprocess.run(
        ["xacro", str(xacro_path)], capture_output=True, text=True, check=True
    ).stdout
    share_dir = Path(share_dir)
    return urdf.replace(f"package://{share_dir.name}/", f"{share_dir}/")


def add_position_servo(spec, joint_name, kp, kv):
    """Declare one position-controlled actuator, MuJoCo's <position/>."""
    act = spec.add_actuator()
    act.name = f"{joint_name}_servo"
    act.target = joint_name
    act.trntype = mujoco.mjtTrn.mjTRN_JOINT
    act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
    act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
    act.gainprm[0] = kp
    act.biasprm[1] = -kp
    act.biasprm[2] = -kv
    return act


def add_scene(spec):
    """A URDF carries no lights, sky or floor, so the viewer starts black."""
    sky = spec.add_texture()
    sky.name = "sky"
    sky.type = mujoco.mjtTexture.mjTEXTURE_SKYBOX
    sky.builtin = mujoco.mjtBuiltin.mjBUILTIN_GRADIENT
    sky.rgb1 = [0.3, 0.5, 0.7]
    sky.rgb2 = [0.0, 0.0, 0.0]
    sky.width = sky.height = 512

    checker = spec.add_texture()
    checker.name = "checker"
    checker.type = mujoco.mjtTexture.mjTEXTURE_2D
    checker.builtin = mujoco.mjtBuiltin.mjBUILTIN_CHECKER
    checker.rgb1 = [0.2, 0.3, 0.4]
    checker.rgb2 = [0.3, 0.4, 0.5]
    checker.width = checker.height = 512

    ground = spec.add_material()
    ground.name = "ground"
    ground.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "checker"
    ground.texrepeat = [4, 4]
    ground.reflectance = 0.1

    floor = spec.worldbody.add_geom()
    floor.name = "floor"
    floor.type = mujoco.mjtGeom.mjGEOM_PLANE
    floor.size = [0.0, 0.0, 0.05]
    floor.material = "ground"

    for x, y in ((2.0, 2.0), (-2.0, -2.0)):
        light = spec.worldbody.add_light()
        light.pos = [x, y, 3.0]
        light.dir = [-x, -y, -3.0]
        light.diffuse = [0.6, 0.6, 0.6]
        light.specular = [0.2, 0.2, 0.2]


def read_palette(mtl_path):
    """Diffuse colour per material name, written beside the converted meshes."""
    palette, current = {}, None
    for line in mtl_path.read_text().splitlines():
        tag, _, rest = line.partition(" ")
        if tag == "newmtl":
            current = rest.strip()
        elif tag == "Kd" and current:
            palette[current] = tuple(float(v) for v in rest.split()[:3]) + (1.0,)
    return palette


def add_visual_meshes(spec, mesh_dir):
    """Draw the coloured meshes MuJoCo would otherwise never see.

    MuJoCo's OBJ reader keeps only the first material group of a file.
    """
    palette = read_palette(mesh_dir / "materials.mtl")
    for body in spec.bodies:
        for collision in list(body.geoms):
            if not collision.meshname:
                continue
            for path in sorted((mesh_dir / collision.meshname).glob("*.obj")):
                mesh = spec.add_mesh()
                mesh.name = f"{collision.meshname}_{path.stem}"
                mesh.file = str(path)

                visual = body.add_geom()
                visual.type = mujoco.mjtGeom.mjGEOM_MESH
                visual.meshname = mesh.name
                visual.pos = collision.pos
                visual.quat = collision.quat
                visual.rgba = palette[path.stem]
                visual.group = VISUAL_GROUP
                visual.contype = 0
                visual.conaffinity = 0
            collision.group = COLLISION_GROUP


def build_model(xacro_path, share_dir, joint_names, config_path=None):
    """Compile the URDF into a MuJoCo model with servos attached."""
    config = load_config(config_path)
    with tempfile.TemporaryDirectory() as tmp:
        urdf_path = Path(tmp) / "robot.urdf"
        urdf_path.write_text(urdf_for_mujoco(xacro_path, share_dir))
        spec = mujoco.MjSpec.from_file(str(urdf_path))

    # Stiff servos need the implicit integrator to stay stable
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    for joint in spec.joints:
        joint.armature = config["armature"]

    # URDF import throws away non-colliding geoms
    spec.compiler.discardvisual = False
    gravcomp = 1.0 if config["gravity_compensation"] else 0.0
    for body in spec.bodies:
        body.gravcomp = gravcomp
        for geom in body.geoms:
            geom.contype = 0
            geom.conaffinity = 0

    mesh_dir = Path(share_dir) / "meshes" / "visual_mujoco"
    if (mesh_dir / "materials.mtl").exists():
        add_visual_meshes(spec, mesh_dir)
    for name in joint_names:
        add_position_servo(spec, name, config["kp"], config["kv"])
    add_scene(spec)
    return spec.compile()
