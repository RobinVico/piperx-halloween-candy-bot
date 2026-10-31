"""Build a MuJoCo model of the PiPER X (+ AgileX gripper) from the official URDF in agx_arm_urdf.

Output: sim/piper_x_scene.xml (+ meshes copied to sim/assets). Kinematic only — the teleop
script sets qpos directly, so no actuators are needed.
"""

import re
import shutil
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "third_party/agx_arm_urdf"
OUT = ROOT / "sim"
ASSETS = OUT / "assets"

COLORS = {  # per-link visual colour (MuJoCo can't read the DAE colours)
    "base_link": "0.15 0.15 0.15 1",
    "link1": "0.85 0.85 0.85 1",
    "link2": "0.85 0.85 0.85 1",
    "link3": "0.85 0.85 0.85 1",
    "link4": "0.15 0.15 0.15 1",
    "link5": "0.85 0.85 0.85 1",
    "link6": "0.15 0.15 0.15 1",
    "flange_link": "0.15 0.15 0.15 1",
    "gripper_base": "0.2 0.2 0.2 1",
    "gripper_link1": "0.6 0.6 0.6 1",
    "gripper_link2": "0.6 0.6 0.6 1",
}


def strip_robot(xml: str) -> str:
    """Return the body of a <robot> element (everything between the tags)."""
    body = re.search(r"<robot[^>]*>(.*)</robot>", xml, re.S).group(1)
    return re.sub(r"<xacro:include[^>]*/>", "", body)


def main():
    ASSETS.mkdir(parents=True, exist_ok=True)
    for stl in list((SRC / "piper_x/meshes").glob("*.stl")) + list((SRC / "agx_gripper/meshes").glob("*.stl")):
        shutil.copy(stl, ASSETS / stl.name)

    arm = strip_robot((SRC / "piper_x/urdf/piper_x_description.urdf").read_text(encoding="utf-8"))
    grip = strip_robot((SRC / "piper_x/urdf/piper_x_with_gripper_description.xacro").read_text(encoding="utf-8"))
    body = arm + grip
    # Use the collision STLs for visuals too, referenced by basename (meshdir = assets).
    body = re.sub(r'filename="package://[^"]*/(?:dae/)?([^/"]+)\.(?:dae|stl)"', r'filename="\1.stl"', body)
    # Drop the "world" link + fixed joint so MuJoCo makes base_link a child of worldbody.
    body = re.sub(r'<link name="world"\s*/>', "", body)
    body = re.sub(r'<joint name="world_to_base_link".*?</joint>', "", body, flags=re.S)
    # Drop the massless virtual "gripper" joint; the teleop sets both finger joints directly.
    body = re.sub(r'<link name="gripper_link"\s*/>', "", body)
    body = re.sub(r'<joint name="gripper" type="prismatic">.*?</joint>', "", body, flags=re.S)
    body = re.sub(r"<mimic[^>]*/>", "", body)

    urdf = (
        '<robot name="piper_x">\n'
        f'  <mujoco><compiler meshdir="{ASSETS.as_posix()}" balanceinertia="true" discardvisual="false" fusestatic="false"/></mujoco>\n'
        f"{body}\n</robot>\n"
    )
    urdf_path = OUT / "piper_x_combined.urdf"
    urdf_path.write_text(urdf, encoding="utf-8")

    spec = mujoco.MjSpec.from_file(str(urdf_path))
    spec.compiler.meshdir = "assets"
    spec.modelname = "piper_x"

    for g in spec.geoms:
        body_name = g.parent.name if hasattr(g, "parent") else None
        if body_name in COLORS:
            g.rgba = [float(x) for x in COLORS[body_name].split()]
        g.contype = 0
        g.conaffinity = 0

    world = spec.worldbody
    world.add_light(pos=[0, 0, 2], dir=[0, 0, -1], diffuse=[0.8, 0.8, 0.8])
    world.add_light(pos=[1, 1, 1.5], dir=[-1, -1, -1], diffuse=[0.4, 0.4, 0.4])
    tex = spec.add_texture(name="grid", type=mujoco.mjtTexture.mjTEXTURE_2D,
                           builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                           rgb1=[0.25, 0.28, 0.32], rgb2=[0.18, 0.2, 0.24], width=300, height=300)
    mat = spec.add_material(name="grid")
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"
    mat.texrepeat = [8, 8]
    world.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1, 1, 0.05], material="grid")
    spec.option.gravity = [0, 0, 0]

    model = spec.compile()
    xml_path = OUT / "piper_x_scene.xml"
    xml_path.write_text(spec.to_xml(), encoding="utf-8")

    print(f"Wrote {xml_path}")
    print(f"nq={model.nq} nbody={model.nbody} ngeom={model.ngeom}")
    for j in range(model.njnt):
        lo, hi = model.jnt_range[j]
        print(f"  joint {model.joint(j).name:16s} range=[{lo:+.3f}, {hi:+.3f}]")


if __name__ == "__main__":
    main()
