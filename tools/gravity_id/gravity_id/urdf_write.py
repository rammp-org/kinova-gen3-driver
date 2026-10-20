"""Write a fitted correction back into the URDF's <inertial> tags (spec, Phase L).

A body of the reduced model is a rigid group of URDF links. Gravity sees only the
group's total m and m*c, so the whole correction goes on the group's heaviest link:
new mass, COM moved; its rotational inertia about its own COM and its rpy are kept,
so the tensor stays positive-definite. Nothing else in the file changes.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np
import pinocchio as pin

from gravity_id.model import NUM_JOINTS, PARAMS_PER_BODY, ArmModel


@dataclass
class LinkChange:
    link: str
    mass_old: float
    mass_new: float
    com_old: np.ndarray  # link frame, m
    com_new: np.ndarray


def _inertial(link: ET.Element) -> tuple[float, np.ndarray]:
    inertial = link.find("inertial")
    if inertial is None or inertial.find("mass") is None:
        return 0.0, np.zeros(3)
    origin = inertial.find("origin")
    xyz = np.zeros(3) if origin is None else np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
    return float(inertial.find("mass").get("value")), xyz


def _rigid_links(root: ET.Element, arm_joint: str) -> set[str]:
    """Links joined to `arm_joint`'s child by fixed joints only. Links behind a joint the
    driver merely LOCKS (gripper fingers at q = 0) are excluded: a correction written there
    would be right for the driver and wrong for anyone who moves the fingers."""
    joints = root.findall("joint")
    child = next(j.find("child").get("link") for j in joints if j.get("name") == arm_joint)
    out, frontier = {child}, [child]
    while frontier:
        parent = frontier.pop()
        for j in joints:
            if j.get("type") == "fixed" and j.find("parent").get("link") == parent:
                out.add(j.find("child").get("link"))
                frontier.append(j.find("child").get("link"))
    return out


def write_corrected_urdf(urdf_in: str, ee_frame: str, dpi: np.ndarray, urdf_out: str) -> list[LinkChange]:
    arm = ArmModel.load(urdf_in, ee_frame)
    tree = ET.parse(urdf_in, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    links = {l.get("name"): l for l in tree.getroot().iter("link")}
    changes = []
    for j in range(1, NUM_JOINTS + 1):
        k = PARAMS_PER_BODY * (j - 1)
        dm, dh = dpi[k], dpi[k + 1 : k + 4]
        if dm == 0.0 and not dh.any():
            continue
        rigid = _rigid_links(tree.getroot(), arm.joint_names[j - 1])
        group = [f for f in arm.model.frames if f.type == pin.FrameType.BODY and f.parentJoint == j and f.name in rigid]
        frame = max(group, key=lambda f: _inertial(links[f.name])[0])
        m, c = _inertial(links[frame.name])
        if m <= 0.0:
            raise ValueError(f"body {j}: no link in its rigid group has an <inertial> to correct")
        if m + dm <= 0.0:
            raise ValueError(f"body {j}: correction {dm:+.3f} kg would leave {frame.name} with mass {m + dm:.3f} kg")
        p, R = frame.placement.translation, frame.placement.rotation  # link frame -> joint frame
        m_new = m + dm
        c_new = R.T @ ((m * (p + R @ c) + dh) / m_new - p)
        inertial = links[frame.name].find("inertial")
        inertial.find("mass").set("value", f"{m_new:.9g}")
        origin = inertial.find("origin")
        if origin is None:
            origin = ET.SubElement(inertial, "origin", rpy="0 0 0")
        origin.set("xyz", " ".join(f"{v:.9g}" for v in c_new))
        changes.append(LinkChange(frame.name, m, m_new, c, c_new))
    tree.write(urdf_out, encoding="unicode", xml_declaration=True)
    return changes
