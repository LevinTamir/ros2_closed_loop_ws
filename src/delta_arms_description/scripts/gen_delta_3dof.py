#!/usr/bin/env python3
"""Generate urdf/delta_3dof.urdf for the 3-DoF delta (ported from PARA_ENGINEER.urdf).

The source URDF expresses its closed loops with a custom <constraint type="spherical_joint">
extension that vanilla ROS/Gazebo cannot parse, plus a MoveIt-only virtual tool chain. Here we
port the geometry into the volcaniarm pure-Gazebo pattern.

Topology (effective leg = R-U-U, a classic translational-delta leg):
  * each leg is an open chain  base -> R(actuated) -> U(2 revolutes) -> forearm -> forearm_tip,
  * the moving platform tool0 is reached through ONE spanning-tree path: leg 1's forearm_tip
    connects to tool0 via two real revolute joints (a passive U-joint). gz Harmonic's sdformat
    drops URDF `floating` joints, so a real-joint spanning tree is used instead of a free body,
  * legs 2 and 3 close their loops with a DetachableJoint weld (forearm_tip <-> a dummy that hangs
    off tool0 on its own passive U-joint).

The assembled home configuration (theta = 0) is baked into the joint origins so that at q = 0 every
welded pair of frames is COINCIDENT -> the DetachableJoints attach with zero impulse (no snap), and
leg 1's spanning-tree path places tool0 exactly at the platform centre.

Run:  python3 scripts/gen_delta_3dof.py   (writes ../urdf/delta_3dof.urdf relative to this file)
"""
import math
import os

# ---- geometry (metres / radians), taken from PARA_ENGINEER.urdf -----------------------------
R_B = 0.0417      # base pivot radius (actuated joint origin)
L1  = 0.0758      # upper arm length (Oberarm)
L2  = 0.1668      # forearm length (Unterarm), tip at x = L2 in the forearm frame
R_P = 0.0276      # platform attach radius on tool0
BASE_Z = 1.0      # base_link height above ground (world fixed joint)

PHI = [0.0, 2.0943951023931953, 4.1887902047863905]   # 120 deg apart

# ---- assembled configuration (theta_home = 0: upper arms horizontal) ------------------------
elbow_r = R_B + L1
gap     = elbow_r - R_P
Z_P     = math.sqrt(L2 * L2 - gap * gap)   # platform drop below the base pivot plane
BETA    = math.atan2(Z_P, -gap)            # universal-joint U1 home bend (about local Y)

# leg-1 spanning-tree transform: forearm_tip frame (R = Ry(BETA)) -> tool0 (R = I, platform centre)
def _ry(a, v):
    c, s = math.cos(a), math.sin(a)
    x, y, z = v
    return (x * c + z * s, y, -x * s + z * c)
_TREE_P = _ry(-BETA, (-R_P, 0.0, 0.0))     # translation of tool0 in the forearm_tip frame

# masses / inertias (source has many zero inertias; Gazebo rejects zero / non-physical tensors)
M_UPPER = 1.636
M_FORE  = 0.167
M_TOOL  = 1.073


def fmt(v):
    return f"{v:.6f}"


def inertial(mass, ixx, iyy, izz, xyz="0 0 0"):
    return (f'    <inertial>\n'
            f'      <origin xyz="{xyz}" rpy="0 0 0"/>\n'
            f'      <mass value="{mass}"/>\n'
            f'      <inertia ixx="{ixx}" ixy="0" ixz="0" iyy="{iyy}" iyz="0" izz="{izz}"/>\n'
            f'    </inertial>')


def tiny_inertial():
    # massless helper link (virtual / tip / dummy) -- isotropic, satisfies triangle inequality
    return inertial(1e-3, 1e-6, 1e-6, 1e-6)


# per-link colours (rgba) so the mechanism is easy to read in Gazebo/RViz
COL_BASE  = "0.25 0.25 0.25 1.0"   # dark grey
COL_TOOL  = "0.95 0.75 0.10 1.0"   # gold platform
LEG_UPPER = {1: "0.85 0.20 0.20 1.0", 2: "0.20 0.70 0.25 1.0", 3: "0.25 0.45 0.90 1.0"}
LEG_FORE  = {1: "0.95 0.55 0.55 1.0", 2: "0.55 0.90 0.60 1.0", 3: "0.55 0.75 1.00 1.0"}


def mesh_visual(name, xyz="0 0 0", rpy="0 0 0", color=None, mat=None):
    mat_xml = ""
    if color is not None:
        mat_name = mat if mat else f"mat_{name}"
        mat_xml = (f'\n      <material name="{mat_name}">'
                   f'\n        <color rgba="{color}"/>'
                   f'\n      </material>')
    return (f'    <visual>\n'
            f'      <origin xyz="{xyz}" rpy="{rpy}"/>\n'
            f'      <geometry>\n'
            f'        <mesh filename="package://delta_arms_description/meshes/PARA_ENGINEER/{name}.stl" scale="0.001 0.001 0.001"/>\n'
            f'      </geometry>{mat_xml}\n'
            f'    </visual>')


def rev_joint(name, parent, child, xyz, rpy, axis, lower, upper, eff="99999.0",
              vel="99999.0", damping="0.01"):
    return (f'  <joint name="{name}" type="revolute">\n'
            f'    <parent link="{parent}"/>\n'
            f'    <child link="{child}"/>\n'
            f'    <origin xyz="{xyz}" rpy="{rpy}"/>\n'
            f'    <axis xyz="{axis}"/>\n'
            f'    <limit effort="{eff}" lower="{lower}" upper="{upper}" velocity="{vel}"/>\n'
            f'    <dynamics damping="{damping}" friction="0.0"/>\n'
            f'  </joint>')


def open_leg(n, phi):
    """base -> R(actuated) -> U1 -> U2 -> forearm -> forearm_tip (massless, preserved)."""
    bx, by = R_B * math.cos(phi), R_B * math.sin(phi)
    s = [f'  <!-- ===================== leg {n} (phi = {math.degrees(phi):.0f} deg) ===================== -->']
    # upper arm
    s.append(f'  <link name="Chain{n}_link_1">')
    s.append(mesh_visual("Oberarm", rpy="1.570 0 1.570", color=LEG_UPPER[n], mat=f"mat_upper_{n}"))
    s.append(inertial(M_UPPER, 1e-3, 3.2e-3, 3.2e-3, xyz="0.0379 0 0"))
    s.append('  </link>')
    s.append(rev_joint(f'Chain{n}_1', 'base_link', f'Chain{n}_link_1',
                       f'{fmt(bx)} {fmt(by)} 0', f'0 0 {fmt(phi)}', '0 1 0',
                       '-1.0297442586766543', '1.4311699866353502',
                       eff='230.0', vel='785.3981633974483', damping='0.1'))
    # universal joint U1 (about Y) -- home bend BETA baked into the origin
    s.append(f'  <link name="Chain{n}_link_virtual">')
    s.append(tiny_inertial())
    s.append('  </link>')
    s.append(rev_joint(f'Chain{n}_U1', f'Chain{n}_link_1', f'Chain{n}_link_virtual',
                       f'{fmt(L1)} 0 0', f'0 {fmt(BETA)} 0', '0 1 0',
                       '-3.141592653589793', '3.141592653589793'))
    # universal joint U2 (about Z) -- home 0
    s.append(f'  <link name="Chain{n}_forearm">')
    s.append(mesh_visual("Unterarm", rpy="0 3.141 0", color=LEG_FORE[n], mat=f"mat_fore_{n}"))
    s.append(inertial(M_FORE, 5e-5, 1.6e-3, 1.6e-3, xyz="0.0834 0 0"))
    s.append('  </link>')
    s.append(rev_joint(f'Chain{n}_U2', f'Chain{n}_link_virtual', f'Chain{n}_forearm',
                       '0 0 0', '0 0 0', '0 0 1',
                       '-1.5707963267948966', '1.5707963267948966'))
    # massless forearm tip (closure point)
    s.append(f'  <link name="Chain{n}_forearm_tip">')
    s.append(tiny_inertial())
    s.append('  </link>')
    s.append(f'  <joint name="Chain{n}_forearm_tip_joint" type="fixed">')
    s.append(f'    <parent link="Chain{n}_forearm"/>')
    s.append(f'    <child link="Chain{n}_forearm_tip"/>')
    s.append(f'    <origin xyz="{fmt(L2)} 0 0" rpy="0 0 0"/>')
    s.append('  </joint>')
    s.append(f'  <gazebo reference="Chain{n}_forearm_tip_joint">')
    s.append('    <preserveFixedJoint>true</preserveFixedJoint>')
    s.append('  </gazebo>')
    return "\n".join(s)


def spherical_chain(prefix, parent, child, first_xyz, first_rpy):
    """Three orthogonal revolutes through a common point == a spherical (ball) joint.

    All three axes intersect at the joint origin (intermediate links have zero-translation
    origins) and are mutually orthogonal, so the chain reproduces the 3-DOF ball joint that the
    source URDF expresses as <constraint type="spherical_joint">. The home transform (position +
    orientation) is carried entirely by the first revolute's origin, so with all three values at 0
    the child frame lands exactly where the old 2-revolute version did (zero-snap weld preserved).
    """
    v1, v2 = f'{prefix}_sph_v1', f'{prefix}_sph_v2'
    lim = ('-3.141592653589793', '3.141592653589793')
    s = [f'  <link name="{v1}">', tiny_inertial(), '  </link>',
         f'  <link name="{v2}">', tiny_inertial(), '  </link>',
         rev_joint(f'{prefix}_jointA', parent, v1, first_xyz, first_rpy, '1 0 0', *lim),
         rev_joint(f'{prefix}_jointB', v1, v2, '0 0 0', '0 0 0', '0 1 0', *lim),
         rev_joint(f'{prefix}_jointC', v2, child, '0 0 0', '0 0 0', '0 0 1', *lim)]
    return "\n".join(s)


def leg1_tree_closure():
    """Spanning-tree path: leg-1 forearm_tip -> (spherical joint) -> tool0 (the moving platform)."""
    s = ['  <!-- ===================== platform (tool0) via leg-1 spanning tree ===================== -->']
    s.append('  <link name="tool0">')
    s.append(mesh_visual("Werkzeugtraeger", rpy="0 0 0.53", color=COL_TOOL, mat="mat_tool"))
    s.append(inertial(M_TOOL, 1.5e-3, 1.5e-3, 2e-3))
    s.append('  </link>')
    s.append(spherical_chain('Chain1_closure', 'Chain1_forearm_tip', 'tool0',
                             f'{fmt(_TREE_P[0])} {fmt(_TREE_P[1])} {fmt(_TREE_P[2])}',
                             f'0 {fmt(-BETA)} 0'))
    return "\n".join(s)


def legN_loop_closure(n, phi):
    """Loop-closure path: tool0 -> (spherical joint) -> dummy, welded to forearm_tip (DetachableJoint)."""
    px, py = R_P * math.cos(phi), R_P * math.sin(phi)
    s = [f'  <!-- ----- leg {n} loop closure (DetachableJoint weld) ----- -->']
    s.append(f'  <link name="Chain{n}_closure_dummy">')
    s.append(tiny_inertial())
    s.append('  </link>')
    # spherical joint: first revolute origin = Rz(phi)*Ry(BETA) at the platform attach point,
    # so the dummy frame is coincident with the forearm tip at q=0 (identity weld).
    s.append(spherical_chain(f'Chain{n}_closure', 'tool0', f'Chain{n}_closure_dummy',
                             f'{fmt(px)} {fmt(py)} 0', f'0 {fmt(BETA)} {fmt(phi)}'))
    s.append('  <gazebo>')
    s.append('    <plugin filename="libgz-sim-detachable-joint-system.so"')
    s.append('            name="gz::sim::systems::DetachableJoint">')
    s.append(f'      <parent_link>Chain{n}_forearm_tip</parent_link>')
    s.append('      <child_model>deltaarm_3dof</child_model>')
    s.append(f'      <child_link>Chain{n}_closure_dummy</child_link>')
    s.append('    </plugin>')
    s.append('  </gazebo>')
    return "\n".join(s)


def position_controller(n):
    return (f'  <gazebo>\n'
            f'    <plugin filename="libgz-sim-joint-position-controller-system.so"\n'
            f'            name="gz::sim::systems::JointPositionController">\n'
            f'      <joint_name>Chain{n}_1</joint_name>\n'
            f'      <topic>/deltaarm_3dof/Chain{n}_1/cmd_pos</topic>\n'
            f'      <p_gain>50</p_gain>\n'
            f'      <i_gain>1</i_gain>\n'
            f'      <d_gain>5</d_gain>\n'
            f'      <i_max>5</i_max>\n'
            f'      <i_min>-5</i_min>\n'
            f'      <cmd_max>50</cmd_max>\n'
            f'      <cmd_min>-50</cmd_min>\n'
            f'    </plugin>\n'
            f'  </gazebo>')


def build():
    out = ['<?xml version="1.0" ?>',
           '<!-- AUTOGENERATED by scripts/gen_delta_3dof.py - DO NOT EDIT BY HAND -->',
           '<!-- 3-DoF delta (ported from PARA_ENGINEER.urdf) into the volcaniarm closed-loop pattern. -->',
           f'<!-- Assembled home: theta=0, platform drop Z_P={Z_P:.5f} m, U1 home bend BETA={BETA:.5f} rad. -->',
           '<robot name="deltaarm_3dof">',
           '',
           '  <link name="world"/>',
           '  <joint name="world_to_base" type="fixed">',
           f'    <origin xyz="0 0 {fmt(BASE_Z)}" rpy="0 0 0"/>',
           '    <parent link="world"/>',
           '    <child link="base_link"/>',
           '  </joint>',
           '',
           '  <link name="base_link">',
           mesh_visual("Kopfplatte", xyz="0 0 0.025", rpy="0 0 -0.53", color=COL_BASE, mat="mat_base"),
           inertial(1.0, 1.5e-2, 1.5e-2, 2e-2),
           '  </link>',
           '']
    for i in range(3):
        out.append(open_leg(i + 1, PHI[i]))
        out.append('')
    out.append(leg1_tree_closure())
    out.append('')
    for i in (1, 2):                       # legs 2 and 3 close via DetachableJoint
        out.append(legN_loop_closure(i + 1, PHI[i]))
        out.append('')
    out.append('  <!-- actuated joint controllers -->')
    for n in (1, 2, 3):
        out.append(position_controller(n))
    out.append('')
    out.append('  <!-- broadcast all joint states back to ROS2 -->')
    out.append('  <gazebo>')
    out.append('    <plugin filename="libgz-sim-joint-state-publisher-system.so"')
    out.append('            name="gz::sim::systems::JointStatePublisher"/>')
    out.append('  </gazebo>')
    out.append('')
    out.append('</robot>')
    return "\n".join(out) + "\n"


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    dst = os.path.join(here, "..", "urdf", "delta_3dof.urdf")
    with open(dst, "w") as f:
        f.write(build())
    print(f"wrote {os.path.normpath(dst)}")
    print(f"  Z_P={Z_P:.5f} m  BETA={BETA:.5f} rad ({math.degrees(BETA):.2f} deg)  "
          f"tree_p={tuple(round(x,6) for x in _TREE_P)}")
