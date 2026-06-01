#!/usr/bin/env python3
"""Generate urdf/delta_3dof.urdf for the 3-DoF delta (ported from PARA_ENGINEER.urdf).

The source URDF expresses its closed loops with a custom <constraint type="spherical_joint">
extension that vanilla ROS/Gazebo cannot parse, plus a MoveIt-only virtual tool chain. Here we
port the geometry into the volcaniarm pure-Gazebo (DetachableJoint) pattern.

A delta is a *translational* parallel robot: the platform always stays level (3 DOF, pure
translation). Rather than rely on the parallelogram physics to enforce that (with ball joints the
platform can fold into a tilted assembly under gravity), tool0 is connected to the base through a
passive 3-prismatic X-Y-Z chain. That gives tool0 exactly 3 translational DOF and ZERO rotational
DOF, so it is guaranteed level by construction. The three legs then drive its position:

  base -> R(actuated) -> upper arm -> spherical -> forearm -> tip
  tip  -- DetachableJoint weld -->  dummy -> spherical -> tool0

Each rod is therefore spherical at both ends (the classic delta S-S rod). The weld + 3R spherical
chain reproduces the source's spherical_joint while letting the loop close across the workspace.

The assembled home (theta = 0) is baked into the joint origins, so at q = 0 every welded pair of
frames is coincident (zero-impulse attach) and tool0 sits at the platform centre.

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
BETA    = math.atan2(Z_P, -gap)            # forearm home bend (about local Y)

# masses / inertias -- lightened vs the source (heavy masses on small links make the closed-loop
# constraints numerically stiff and let the welds sag); values are physical for a ~0.3 m delta.
M_UPPER = 0.30
M_FORE  = 0.08
M_TOOL  = 0.30

# per-link colours (rgba). All arm links share the base-link colour; only the moving platform
# (tool0 / the EE) gets a distinct colour so it stays easy to spot.
COL_BASE = "0.25 0.25 0.25 1.0"   # dark grey -- base and all arm links
COL_TOOL = "0.95 0.75 0.10 1.0"   # gold platform (EE)


def fmt(v):
    return f"{v:.6f}"


def inertial(mass, ixx, iyy, izz, xyz="0 0 0"):
    return (f'    <inertial>\n'
            f'      <origin xyz="{xyz}" rpy="0 0 0"/>\n'
            f'      <mass value="{mass}"/>\n'
            f'      <inertia ixx="{ixx}" ixy="0" ixz="0" iyy="{iyy}" iyz="0" izz="{izz}"/>\n'
            f'    </inertial>')


def tiny_inertial():
    return inertial(0.01, 1e-5, 1e-5, 1e-5)


def mesh_visual(name, xyz="0 0 0", rpy="0 0 0", color=None, mat=None):
    mat_xml = ""
    if color is not None:
        mat_xml = (f'\n      <material name="{mat or ("mat_" + name)}">'
                   f'\n        <color rgba="{color}"/>'
                   f'\n      </material>')
    return (f'    <visual>\n'
            f'      <origin xyz="{xyz}" rpy="{rpy}"/>\n'
            f'      <geometry>\n'
            f'        <mesh filename="package://delta_arms_description/meshes/PARA_ENGINEER/{name}.stl" scale="0.001 0.001 0.001"/>\n'
            f'      </geometry>{mat_xml}\n'
            f'    </visual>')


def rev_joint(name, parent, child, xyz, rpy, axis, lower, upper, eff="99999.0",
              vel="99999.0", damping="0.02", jtype="revolute"):
    return (f'  <joint name="{name}" type="{jtype}">\n'
            f'    <parent link="{parent}"/>\n'
            f'    <child link="{child}"/>\n'
            f'    <origin xyz="{xyz}" rpy="{rpy}"/>\n'
            f'    <axis xyz="{axis}"/>\n'
            f'    <limit effort="{eff}" lower="{lower}" upper="{upper}" velocity="{vel}"/>\n'
            f'    <dynamics damping="{damping}" friction="0.0"/>\n'
            f'  </joint>')


def spherical_chain(prefix, parent, child, first_xyz, first_rpy):
    """Three orthogonal revolutes through a common point == a spherical (ball) joint.

    Intermediate links have zero-translation origins, so all three axes intersect at the joint
    origin and the child position is fixed under the rotations; only the first revolute carries the
    home transform. Reproduces the source <constraint type="spherical_joint">.
    """
    v1, v2 = f'{prefix}_v1', f'{prefix}_v2'
    lim = ('-3.141592653589793', '3.141592653589793')
    return "\n".join([
        f'  <link name="{v1}">', tiny_inertial(), '  </link>',
        f'  <link name="{v2}">', tiny_inertial(), '  </link>',
        rev_joint(f'{prefix}_A', parent, v1, first_xyz, first_rpy, '1 0 0', *lim),
        rev_joint(f'{prefix}_B', v1, v2, '0 0 0', '0 0 0', '0 1 0', *lim),
        rev_joint(f'{prefix}_C', v2, child, '0 0 0', '0 0 0', '0 0 1', *lim)])


def platform_ppp():
    """Passive prismatic X-Y-Z chain base_link -> tool0: 3 translational DOF, always level."""
    lim = ('-0.5', '0.5')
    return "\n".join([
        '  <!-- ===================== moving platform (tool0) ===================== -->',
        '  <!-- 3 passive prismatic joints => tool0 has 3 translational DOF and stays level. -->',
        '  <link name="tool_px"><inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.01"/>'
        '<inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial></link>',
        '  <link name="tool_py"><inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.01"/>'
        '<inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial></link>',
        '  <link name="tool0">',
        mesh_visual("Werkzeugtraeger", rpy="0 0 0.53", color=COL_TOOL, mat="mat_tool"),
        inertial(M_TOOL, 3e-4, 3e-4, 5e-4),
        '  </link>',
        rev_joint('platform_x', 'base_link', 'tool_px', f'0 0 {fmt(-Z_P)}', '0 0 0', '1 0 0',
                  *lim, jtype='prismatic', damping='0.5'),
        rev_joint('platform_y', 'tool_px', 'tool_py', '0 0 0', '0 0 0', '0 1 0',
                  *lim, jtype='prismatic', damping='0.5'),
        rev_joint('platform_z', 'tool_py', 'tool0', '0 0 0', '0 0 0', '0 0 1',
                  *lim, jtype='prismatic', damping='0.5')])


def open_leg(n, phi):
    """base -> R(actuated) -> upper arm -> spherical -> forearm -> tip (massless, preserved)."""
    bx, by = R_B * math.cos(phi), R_B * math.sin(phi)
    s = [f'  <!-- ===================== leg {n} (phi = {math.degrees(phi):.0f} deg) ===================== -->']
    s.append(f'  <link name="Chain{n}_link_1">')
    s.append(mesh_visual("Oberarm", rpy="1.570 0 1.570", color=COL_BASE, mat="mat_arm"))
    s.append(inertial(M_UPPER, 5e-5, 6e-4, 6e-4, xyz="0.0379 0 0"))
    s.append('  </link>')
    s.append(rev_joint(f'Chain{n}_1', 'base_link', f'Chain{n}_link_1',
                       f'{fmt(bx)} {fmt(by)} 0', f'0 0 {fmt(phi)}', '0 1 0',
                       '-1.0297442586766543', '1.4311699866353502',
                       eff='230.0', vel='785.3981633974483', damping='0.1'))
    # forearm rod, spherical (ball) joint at the top, home bend BETA baked in
    s.append(f'  <link name="Chain{n}_forearm">')
    s.append(mesh_visual("Unterarm", rpy="0 3.141 0", color=COL_BASE, mat="mat_arm"))
    s.append(inertial(M_FORE, 1e-5, 7e-4, 7e-4, xyz="0.0834 0 0"))
    s.append('  </link>')
    s.append(spherical_chain(f'Chain{n}_top', f'Chain{n}_link_1', f'Chain{n}_forearm',
                             f'{fmt(L1)} 0 0', f'0 {fmt(BETA)} 0'))
    # massless rod tip (closure point)
    s.append(f'  <link name="Chain{n}_tip">')
    s.append(tiny_inertial())
    s.append('  </link>')
    s.append(f'  <joint name="Chain{n}_tip_joint" type="fixed">')
    s.append(f'    <parent link="Chain{n}_forearm"/>')
    s.append(f'    <child link="Chain{n}_tip"/>')
    s.append(f'    <origin xyz="{fmt(L2)} 0 0" rpy="0 0 0"/>')
    s.append('  </joint>')
    s.append(f'  <gazebo reference="Chain{n}_tip_joint">')
    s.append('    <preserveFixedJoint>true</preserveFixedJoint>')
    s.append('  </gazebo>')
    return "\n".join(s)


def weld(n, phi):
    """Loop closure: tool0 -> spherical joint -> dummy, welded to the rod tip (DetachableJoint)."""
    px, py = R_P * math.cos(phi), R_P * math.sin(phi)
    s = [f'  <!-- leg {n}: DetachableJoint weld -->']
    s.append(f'  <link name="Chain{n}_dummy">')
    s.append(tiny_inertial())
    s.append('  </link>')
    # first revolute origin orientation Rz(phi)*Ry(BETA) -> rpy (0, BETA, phi); position = attach pt
    s.append(spherical_chain(f'Chain{n}_cl', 'tool0', f'Chain{n}_dummy',
                             f'{fmt(px)} {fmt(py)} 0', f'0 {fmt(BETA)} {fmt(phi)}'))
    s.append('  <gazebo>')
    s.append('    <plugin filename="libgz-sim-detachable-joint-system.so"')
    s.append('            name="gz::sim::systems::DetachableJoint">')
    s.append(f'      <parent_link>Chain{n}_tip</parent_link>')
    s.append('      <child_model>deltaarm_3dof</child_model>')
    s.append(f'      <child_link>Chain{n}_dummy</child_link>')
    s.append('    </plugin>')
    s.append('  </gazebo>')
    return "\n".join(s)


def position_controller(n):
    return (f'  <gazebo>\n'
            f'    <plugin filename="libgz-sim-joint-position-controller-system.so"\n'
            f'            name="gz::sim::systems::JointPositionController">\n'
            f'      <joint_name>Chain{n}_1</joint_name>\n'
            f'      <topic>/deltaarm_3dof/Chain{n}_1/cmd_pos</topic>\n'
            f'      <p_gain>20</p_gain>\n'
            f'      <i_gain>0.5</i_gain>\n'
            f'      <d_gain>3</d_gain>\n'
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
           f'<!-- Assembled home: theta=0, platform drop Z_P={Z_P:.5f} m, forearm home bend BETA={BETA:.5f} rad. -->',
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
    out.append(platform_ppp())
    out.append('')
    for i in range(3):
        out.append(weld(i + 1, PHI[i]))
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
    print(f"  Z_P={Z_P:.5f} m  BETA={BETA:.5f} rad ({math.degrees(BETA):.2f} deg)")
