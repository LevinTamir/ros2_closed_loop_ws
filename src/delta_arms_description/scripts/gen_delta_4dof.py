#!/usr/bin/env python3
"""Generate urdf/delta_4dof.urdf for the 4-DoF delta (ported from A_00036-02.urdf).

Same closed-loop technique as the 3-DoF delta (see gen_delta_3dof.py): tool_holder (the moving
platform) hangs from a passive 3-prismatic X-Y-Z chain so it has 3 translational DOF and stays
level by construction; three spherical-ended (S-S) rods drive its position via DetachableJoint welds.

The 4-DoF adds the A_00036 central column:
  * tool0  -- a flange that rotates about z relative to tool_holder; this revolute is the actuated
    4th DOF (the EE rotation).
  * a telescopic central shaft (Teleskopwelle) from the base centre down to tool0: spherical at the
    top so it tilts to follow the platform, a passive prismatic so it telescopes, and a
    DetachableJoint weld (+3R spherical) at the bottom so it tracks the platform.

Dimensions are taken from A_00036-02.urdf (a ~1 m-reach robot). Run:
  python3 scripts/gen_delta_4dof.py   (writes ../urdf/delta_4dof.urdf relative to this file)
"""
import math
import os

# ---- geometry (metres / radians), from A_00036-02.urdf --------------------------------------
R_B = 0.25        # base pivot radius (Chain*_1 origin)
L1  = 0.375       # upper arm length (Oberarm), pivot of forearm
L2  = 0.96        # forearm length (Unterarm), constraint parent_origin x
R_P = 0.07        # platform attach radius on tool_holder (constraint child_origin x)
BASE_Z = 1.6      # base_link height above ground

PHI = [0.0, 2.0943951023931953, 4.1887902047863905]

elbow_r = R_B + L1
gap     = elbow_r - R_P
Z_P     = math.sqrt(L2 * L2 - gap * gap)   # platform drop below the base pivot plane
BETA    = math.atan2(Z_P, -gap)            # forearm home bend (about local Y)

# masses / inertias -- lightened from the source so the closed-loop welds stay stiff
M_UPPER, M_FORE = 0.50, 0.15
M_HOLDER, M_TOOL0 = 0.40, 0.20
M_SHAFT = 0.20

COL_BASE  = "0.25 0.25 0.25 1.0"   # dark grey -- base + all arm links
COL_TOOL  = "0.95 0.75 0.10 1.0"   # gold platform / flange (EE)
COL_SHAFT = "0.45 0.45 0.50 1.0"   # central telescopic shaft

MESH = "package://delta_arms_description/meshes/A_00036"


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
            f'        <mesh filename="{MESH}/{name}.stl" scale="0.001 0.001 0.001"/>\n'
            f'      </geometry>{mat_xml}\n'
            f'    </visual>')


def joint(name, parent, child, xyz, rpy, axis, lower, upper, eff="99999.0",
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
    """3 orthogonal revolutes through a common point == a spherical (ball) joint."""
    v1, v2 = f'{prefix}_v1', f'{prefix}_v2'
    lim = ('-3.141592653589793', '3.141592653589793')
    return "\n".join([
        f'  <link name="{v1}">', tiny_inertial(), '  </link>',
        f'  <link name="{v2}">', tiny_inertial(), '  </link>',
        joint(f'{prefix}_A', parent, v1, first_xyz, first_rpy, '1 0 0', *lim),
        joint(f'{prefix}_B', v1, v2, '0 0 0', '0 0 0', '0 1 0', *lim),
        joint(f'{prefix}_C', v2, child, '0 0 0', '0 0 0', '0 0 1', *lim)])


def detachable(parent_link, child_link):
    return "\n".join([
        '  <gazebo>',
        '    <plugin filename="libgz-sim-detachable-joint-system.so"',
        '            name="gz::sim::systems::DetachableJoint">',
        f'      <parent_link>{parent_link}</parent_link>',
        '      <child_model>deltaarm_4dof</child_model>',
        f'      <child_link>{child_link}</child_link>',
        '    </plugin>',
        '  </gazebo>'])


def platform():
    """tool_holder via passive 3-prismatic (level); tool0 = flange rotating about z (4th DOF)."""
    lim = ('-0.7', '0.7')
    return "\n".join([
        '  <!-- ===================== moving platform ===================== -->',
        '  <!-- tool_holder: 3 passive prismatic joints => 3 translational DOF, stays level -->',
        '  <link name="tool_px"><inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.01"/>'
        '<inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial></link>',
        '  <link name="tool_py"><inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.01"/>'
        '<inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial></link>',
        '  <link name="tool_holder">',
        mesh_visual("Werkzeugtraeger", xyz="0 0 0.022", color=COL_TOOL, mat="mat_tool"),
        inertial(M_HOLDER, 1e-3, 1e-3, 2e-3),
        '  </link>',
        joint('platform_x', 'base_link', 'tool_px', f'0 0 {fmt(-Z_P)}', '0 0 0', '1 0 0',
              *lim, jtype='prismatic', damping='0.5'),
        joint('platform_y', 'tool_px', 'tool_py', '0 0 0', '0 0 0', '0 1 0',
              *lim, jtype='prismatic', damping='0.5'),
        joint('platform_z', 'tool_py', 'tool_holder', '0 0 0', '0 0 0', '0 0 1',
              *lim, jtype='prismatic', damping='0.5'),
        '  <!-- tool0: flange that rotates about z relative to the platform = actuated 4th DOF -->',
        '  <link name="tool0">',
        mesh_visual("Flansch", xyz="0 0 0.022", color=COL_TOOL, mat="mat_tool"),
        inertial(M_TOOL0, 5e-4, 5e-4, 8e-4),
        '  </link>',
        joint('Chain4_1', 'tool_holder', 'tool0', '0 0 0', '0 0 0', '0 0 1',
              '-7.0', '7.0', eff='42.0', vel='785.0', damping='0.1')])


def central_shaft():
    """Telescopic Teleskopwelle: base centre -> spherical -> prismatic -> weld to tool0."""
    s = ['  <!-- ===================== central telescopic shaft (4th-axis column) ===================== -->',
         '  <link name="Chain4_top_link">',
         mesh_visual("Teleskopwelle_oben", color=COL_SHAFT, mat="mat_shaft"),
         inertial(M_SHAFT, 5e-4, 5e-4, 2e-4),
         '  </link>',
         '  <link name="Chain4_bottom_link">',
         mesh_visual("Teleskopwelle_unten", color=COL_SHAFT, mat="mat_shaft"),
         inertial(M_SHAFT, 5e-4, 5e-4, 2e-4),
         '  </link>',
         '  <link name="Chain4_tip">', tiny_inertial(), '  </link>']
    # spherical at the base centre so the shaft tilts to follow the platform
    s.append(spherical_chain('Chain4_top', 'base_link', 'Chain4_top_link', '0 0 0', '0 0 0'))
    # passive prismatic; origin carries the home length so the shaft spans base->platform at q=0
    s.append(joint('Chain4_4', 'Chain4_top_link', 'Chain4_bottom_link',
                   f'0 0 {fmt(-Z_P)}', '0 0 0', '0 0 -1', '0.0', '0.7',
                   eff='1000.0', vel='10.0', damping='0.2', jtype='prismatic'))
    # tip at the bottom-link origin -> coincident with tool0 (platform centre) at home
    s.append('  <joint name="Chain4_tip_joint" type="fixed">')
    s.append('    <parent link="Chain4_bottom_link"/>')
    s.append('    <child link="Chain4_tip"/>')
    s.append('    <origin xyz="0 0 0" rpy="0 0 0"/>')
    s.append('  </joint>')
    s.append('  <gazebo reference="Chain4_tip_joint"><preserveFixedJoint>true</preserveFixedJoint></gazebo>')
    # weld the shaft bottom to tool0 (spherical: position only, so tool0 still rotates under it)
    s.append('  <link name="Chain4_dummy">')
    s.append(tiny_inertial())
    s.append('  </link>')
    s.append(spherical_chain('Chain4_cl', 'tool0', 'Chain4_dummy', '0 0 0', '0 0 0'))
    s.append(detachable('Chain4_tip', 'Chain4_dummy'))
    return "\n".join(s)


def open_leg(n, phi):
    bx, by = R_B * math.cos(phi), R_B * math.sin(phi)
    s = [f'  <!-- ===================== leg {n} (phi = {math.degrees(phi):.0f} deg) ===================== -->']
    s.append(f'  <link name="Chain{n}_link_1">')
    s.append(mesh_visual("Oberarm", rpy="1.570 0 0", color=COL_BASE, mat="mat_arm"))
    s.append(inertial(M_UPPER, 5e-4, 6e-3, 6e-3, xyz="0.19 0 0"))
    s.append('  </link>')
    s.append(joint(f'Chain{n}_1', 'base_link', f'Chain{n}_link_1',
                   f'{fmt(bx)} {fmt(by)} 0', f'0 0 {fmt(phi)}', '0 1 0',
                   '-1.0297442586766543', '1.4311699866353502',
                   eff='230.0', vel='785.3981633974483', damping='0.1'))
    s.append(f'  <link name="Chain{n}_forearm">')
    s.append(mesh_visual("Unterarm", rpy="-1.570 0 0", color=COL_BASE, mat="mat_arm"))
    s.append(inertial(M_FORE, 5e-5, 1.2e-2, 1.2e-2, xyz="0.48 0 0"))
    s.append('  </link>')
    s.append(spherical_chain(f'Chain{n}_top', f'Chain{n}_link_1', f'Chain{n}_forearm',
                             f'{fmt(L1)} 0 0', f'0 {fmt(BETA)} 0'))
    s.append(f'  <link name="Chain{n}_tip">')
    s.append(tiny_inertial())
    s.append('  </link>')
    s.append(f'  <joint name="Chain{n}_tip_joint" type="fixed">')
    s.append(f'    <parent link="Chain{n}_forearm"/>')
    s.append(f'    <child link="Chain{n}_tip"/>')
    s.append(f'    <origin xyz="{fmt(L2)} 0 0" rpy="0 0 0"/>')
    s.append('  </joint>')
    s.append(f'  <gazebo reference="Chain{n}_tip_joint"><preserveFixedJoint>true</preserveFixedJoint></gazebo>')
    return "\n".join(s)


def weld(n, phi):
    px, py = R_P * math.cos(phi), R_P * math.sin(phi)
    s = [f'  <!-- leg {n}: DetachableJoint weld -->',
         f'  <link name="Chain{n}_dummy">', tiny_inertial(), '  </link>',
         spherical_chain(f'Chain{n}_cl', 'tool_holder', f'Chain{n}_dummy',
                         f'{fmt(px)} {fmt(py)} 0', f'0 {fmt(BETA)} {fmt(phi)}'),
         detachable(f'Chain{n}_tip', f'Chain{n}_dummy')]
    return "\n".join(s)


def position_controller(joint_name, topic, p="30", d="4"):
    return (f'  <gazebo>\n'
            f'    <plugin filename="libgz-sim-joint-position-controller-system.so"\n'
            f'            name="gz::sim::systems::JointPositionController">\n'
            f'      <joint_name>{joint_name}</joint_name>\n'
            f'      <topic>{topic}</topic>\n'
            f'      <p_gain>{p}</p_gain>\n'
            f'      <i_gain>0.5</i_gain>\n'
            f'      <d_gain>{d}</d_gain>\n'
            f'      <i_max>10</i_max>\n'
            f'      <i_min>-10</i_min>\n'
            f'      <cmd_max>200</cmd_max>\n'
            f'      <cmd_min>-200</cmd_min>\n'
            f'    </plugin>\n'
            f'  </gazebo>')


def build():
    out = ['<?xml version="1.0" ?>',
           '<!-- AUTOGENERATED by scripts/gen_delta_4dof.py - DO NOT EDIT BY HAND -->',
           '<!-- 4-DoF delta (ported from A_00036-02.urdf) into the volcaniarm closed-loop pattern. -->',
           f'<!-- Assembled home: theta=0, platform drop Z_P={Z_P:.4f} m, forearm home bend BETA={BETA:.4f} rad. -->',
           '<robot name="deltaarm_4dof">',
           '',
           '  <link name="world"/>',
           '  <joint name="world_to_base" type="fixed">',
           f'    <origin xyz="0 0 {fmt(BASE_Z)}" rpy="0 0 0"/>',
           '    <parent link="world"/>',
           '    <child link="base_link"/>',
           '  </joint>',
           '',
           '  <link name="base_link">',
           mesh_visual("Kopfplatte", color=COL_BASE, mat="mat_base"),
           inertial(1.0, 3e-2, 3e-2, 5e-2),
           '  </link>',
           '']
    for i in range(3):
        out.append(open_leg(i + 1, PHI[i]))
        out.append('')
    out.append(platform())
    out.append('')
    out.append(central_shaft())
    out.append('')
    for i in range(3):
        out.append(weld(i + 1, PHI[i]))
        out.append('')
    out.append('  <!-- actuated joint controllers: 3 arms + 1 EE rotation -->')
    for n in (1, 2, 3):
        out.append(position_controller(f'Chain{n}_1', f'/deltaarm_4dof/Chain{n}_1/cmd_pos'))
    out.append(position_controller('Chain4_1', '/deltaarm_4dof/Chain4_1/cmd_pos', p="15", d="2"))
    out.append('')
    out.append('  <gazebo>')
    out.append('    <plugin filename="libgz-sim-joint-state-publisher-system.so"')
    out.append('            name="gz::sim::systems::JointStatePublisher"/>')
    out.append('  </gazebo>')
    out.append('')
    out.append('</robot>')
    return "\n".join(out) + "\n"


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    dst = os.path.join(here, "..", "urdf", "delta_4dof.urdf")
    with open(dst, "w") as f:
        f.write(build())
    print(f"wrote {os.path.normpath(dst)}")
    print(f"  Z_P={Z_P:.4f} m  BETA={BETA:.4f} rad ({math.degrees(BETA):.2f} deg)")
