# ROS2 Closed-Loop Kinematics Workspace

A test bed for modeling and simulating robots with **closed kinematic chains** in the new Gazebo
simulator (Gazebo Harmonic) with ROS 2. Plain URDF can only describe open kinematic *trees*; this
repo shows a clean, repeatable way to close loops at runtime using Gazebo's `DetachableJoint`
system plugin, and applies it to three robots of increasing complexity.

Everything lives in a single package, [`delta_arms_description`](src/delta_arms_description):

| Example | What it is | Launch |
|---|---|---|
| **5-bar linkage** | a planar (2D) 5-bar parallel arm | `fivebar_linkage.launch.py` |
| **3-DoF delta** | a translational delta parallel robot | `3dof_delta.launch.py` |
| **4-DoF delta** | a delta with a central telescopic shaft that adds an end-effector rotation axis | `4dof_delta.launch.py` |

<p align="center">
  <img src=".media/fivebar_linkage_gazebo.png" alt="5-bar linkage in Gazebo" width="32%">
  <img src=".media/3dof_delta_gazebo.png" alt="3-DoF delta in Gazebo" width="32%">
  <img src=".media/4dof_delta_gazebo.png" alt="4-DoF delta in Gazebo" width="32%">
</p>

The 5-bar is the reference example: it is the simplest closed loop, and the
[implementation guide](#implementation-guide-adapting-your-own-robot) below walks through exactly how
its URDF, world, and launch file are wired up. The two deltas reuse the same pattern.

## Requirements

- ROS 2 Jazzy (tested)
- Gazebo Harmonic (`gz sim`)
- `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`

## Installation

```bash
git clone git@github.com:LevinTamir/ros2_closed_loop_ws.git
cd ros2_closed_loop_ws
colcon build
source install/setup.bash
```

## Usage

Open two terminals: one runs the simulation, the other sends joint commands (positions in radians,
`std_msgs/Float64` on `/<robot>/<joint>/cmd_pos`).

### 5-bar linkage (the reference example)

```bash
# terminal 1 — simulation
ros2 launch delta_arms_description fivebar_linkage.launch.py
```

```bash
# terminal 2 — the two motors (Chain1_1 = left, Chain2_1 = right)
ros2 topic pub -1 /fivebar_linkage/Chain1_1/cmd_pos std_msgs/msg/Float64 "{data: 0.3}"
ros2 topic pub -1 /fivebar_linkage/Chain2_1/cmd_pos std_msgs/msg/Float64 "{data: -0.3}"
```

### 3-DoF delta

```bash
ros2 launch delta_arms_description 3dof_delta.launch.py
```

```bash
# equal angles on all three arms move the platform up/down; unequal angles move it
# sideways — the platform always stays level
ros2 topic pub -1 /delta_3dof/Chain1_1/cmd_pos std_msgs/msg/Float64 "{data: 0.4}"
ros2 topic pub -1 /delta_3dof/Chain2_1/cmd_pos std_msgs/msg/Float64 "{data: 0.4}"
ros2 topic pub -1 /delta_3dof/Chain3_1/cmd_pos std_msgs/msg/Float64 "{data: 0.4}"
```

### 4-DoF delta

```bash
ros2 launch delta_arms_description 4dof_delta.launch.py
```

```bash
# the three arms translate the platform; Chain4_1 rotates the end-effector flange
ros2 topic pub -1 /delta_4dof/Chain1_1/cmd_pos std_msgs/msg/Float64 "{data: 0.3}"
ros2 topic pub -1 /delta_4dof/Chain4_1/cmd_pos std_msgs/msg/Float64 "{data: 1.0}"
```

> Note: the delta command topics use `delta_3dof` / `delta_4dof` (not `3dof_delta`) because ROS 2
> topic names may not start with a digit.

## Repository structure

```
delta_arms_description/
├── urdf/
│   ├── fivebar_linkage.urdf      # hand-written 5-bar (reference example)
│   ├── 3dof_delta.urdf           # generated — do not edit by hand
│   └── 4dof_delta.urdf           # generated — do not edit by hand
├── launch/
│   ├── fivebar_linkage.launch.py # one launch file per robot (same structure)
│   ├── 3dof_delta.launch.py
│   └── 4dof_delta.launch.py
├── worlds/
│   ├── fivebar_world.sdf         # DART physics world for the 5-bar
│   └── delta_world.sdf           # DART world for the deltas (dantzig solver)
├── meshes/
│   ├── fivebar_linkage/          # one mesh folder per robot
│   ├── PARA_ENGINEER/            # 3-DoF delta meshes
│   └── A_00036/                  # 4-DoF delta meshes
├── scripts/
│   ├── gen_delta_3dof.py         # generators that emit the delta URDFs
│   └── gen_delta_4dof.py
├── CMakeLists.txt                # installs launch/ meshes/ urdf/ worlds/
└── package.xml
```

## How the closed loops are modeled

A URDF is a *tree* — every link has exactly one parent — so it cannot, on its own, express a loop.
The loop is instead closed **at runtime** by Gazebo's `DetachableJoint` plugin, which welds two links
together once the model is spawned. The trick that makes this clean is to arrange the two welded
links so their frames are **coincident at the robot's assembled "home" pose**; the weld then attaches
with no snap or impulse. Actuated joints are driven by `JointPositionController` plugins over
`cmd_pos` topics, and all joint states are republished by a `JointStatePublisher` plugin.

The two deltas extend this: each forearm rod is given a ball joint at each end (a `DetachableJoint`
weld plus a 3-revolute "spherical" chain reproduces the source `spherical_joint` constraints), and
the moving platform hangs from a passive 3-prismatic X-Y-Z chain so it keeps exactly 3 translational
DOF and stays level by construction. Because the delta geometry is repetitive, their URDFs are
produced by the generators in [`scripts/`](src/delta_arms_description/scripts) rather than written by
hand.

## Implementation guide: adapting your own robot

This is the recipe for turning an ordinary URDF into a closed-loop Gazebo model, illustrated with the
**5-bar linkage** ([`urdf/fivebar_linkage.urdf`](src/delta_arms_description/urdf/fivebar_linkage.urdf)).
A 5-bar has two serial arms (`Chain1` = left, `Chain2` = right) whose tips must meet — that meeting
point is the loop to close.

### 1. Anchor the robot to the world

Add a `world` link and a fixed joint that places the base. (Here the base sits 0.98 m up so the arm
hangs in front of it.)

```xml
<link name="world"/>
<joint name="world_to_base" type="fixed">
  <origin xyz="0 0 0.98" rpy="0 0 3.14159"/>
  <parent link="world"/>
  <child link="base_link"/>
</joint>
```

### 2. Model the open chains, ending each in a massless "tip" at the closure point

Keep the kinematics as a normal open tree. On the chain that will be the *parent* of the weld, add a
tiny massless link fixed exactly at the physical closure point. Tell Gazebo **not** to merge this
fixed joint away, otherwise the link disappears and the plugin can't find it:

```xml
<joint name="Chain2_tip_joint" type="fixed">
  <origin xyz="0 0 0.6225" rpy="0 0 0"/>
  <parent link="Chain2_link_2"/>     <!-- right-arm leaf -->
  <child  link="Chain2_tip"/>        <!-- massless tip at the closure point -->
</joint>
<gazebo reference="Chain2_tip_joint">
  <preserveFixedJoint>true</preserveFixedJoint>
</gazebo>
```

### 3. Add a coincident "dummy" on the other chain

On the *child* chain, add a matching dummy link. The key is its joint `origin`: choose the `rpy` so
that, at the home pose (all joints at 0), the dummy's world frame **coincides** with `Chain2_tip`'s.
Here a `revolute` provides the one passive DOF the closed 5-bar needs, and `rpy="1.8398 0 0"` aligns
the two frames:

```xml
<joint name="Chain1_closure" type="revolute">
  <origin xyz="0 0 0.6225" rpy="1.8398 0 0"/>  <!-- tuned so Chain1_dummy == Chain2_tip at q=0 -->
  <parent link="Chain1_link_2"/>               <!-- left-arm leaf -->
  <child  link="Chain1_dummy"/>
  <axis xyz="1 0 0"/>
  <limit lower="-3.14" upper="3.14" effort="0" velocity="10"/>
</joint>
```

### 4. Close the loop with a DetachableJoint

A single plugin welds the two leaves. `child_model` **must equal the name the robot is spawned with**
(see the launch file). Because the frames coincide at home, the weld is an identity transform — no
snap:

```xml
<gazebo>
  <plugin filename="libgz-sim-detachable-joint-system.so"
          name="gz::sim::systems::DetachableJoint">
    <parent_link>Chain2_tip</parent_link>
    <child_model>fivebar_linkage</child_model>
    <child_link>Chain1_dummy</child_link>
  </plugin>
</gazebo>
```

### 5. Drive the actuated joints

One `JointPositionController` per motor, each listening on its own `cmd_pos` topic with PID gains:

```xml
<gazebo>
  <plugin filename="libgz-sim-joint-position-controller-system.so"
          name="gz::sim::systems::JointPositionController">
    <joint_name>Chain1_1</joint_name>
    <topic>/fivebar_linkage/Chain1_1/cmd_pos</topic>
    <p_gain>10</p_gain> <i_gain>1</i_gain> <d_gain>1</d_gain>
    <i_max>5</i_max> <i_min>-5</i_min> <cmd_max>10</cmd_max> <cmd_min>-10</cmd_min>
  </plugin>
</gazebo>
```

### 6. Publish joint states

One plugin republishes every joint position so the rest of the system (TF, RViz, your nodes) can see
the full state, including the passive joints:

```xml
<gazebo>
  <plugin filename="libgz-sim-joint-state-publisher-system.so"
          name="gz::sim::systems::JointStatePublisher"/>
</gazebo>
```

### 7. The Gazebo world

Closed loops are stiff, so the world ([`worlds/fivebar_world.sdf`](src/delta_arms_description/worlds/fivebar_world.sdf))
selects the **DART** physics engine and loads the standard system plugins. For tougher loops (the
deltas) switch the solver from `pgs` to `dantzig` and shrink the step — see `delta_world.sdf`:

```xml
<physics name="1ms" type="dart">
  <max_step_size>0.001</max_step_size>
  <dart>
    <solver><solver_type>pgs</solver_type></solver>  <!-- dantzig for stiffer loops -->
    <collision_detector>ode</collision_detector>
  </dart>
</physics>
<plugin filename="gz-sim-physics-system"           name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system"     name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-contact-system"           name="gz::sim::systems::Contact"/>
```

### 8. The launch file

The launch file ([`launch/fivebar_linkage.launch.py`](src/delta_arms_description/launch/fivebar_linkage.launch.py))
ties it together: read the URDF into `robot_description`, set `GZ_SIM_RESOURCE_PATH` so `package://`
mesh paths resolve, start Gazebo with the world, spawn the model from the `robot_description` topic,
and bridge `/clock` plus the `cmd_pos` topics with `ros_gz_bridge`:

```python
# spawn the model — the -name MUST match <child_model> in the DetachableJoint
Node(package="ros_gz_sim", executable="create",
     arguments=["-topic", "robot_description", "-name", "fivebar_linkage"])

# bridge ROS <-> Gazebo. Syntax: <topic>@<ros_type>[<gz_type>  ('[' = gz->ros, ']' = ros->gz)
Node(package="ros_gz_bridge", executable="parameter_bridge",
     arguments=[
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/fivebar_linkage/Chain1_1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
        "/fivebar_linkage/Chain2_1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
     ])
```

### For parallel robots (the deltas)

The deltas apply steps 1–8 per leg, but each forearm rod uses a **ball joint at both ends** (a
`DetachableJoint` weld plus a 3-revolute spherical chain) instead of a single revolute, and the moving
platform hangs from a **passive 3-prismatic X-Y-Z chain** so it keeps 3 translational DOF and stays
level. The repetitive geometry — and the home pose that makes every weld coincident — is computed and
emitted by [`scripts/gen_delta_3dof.py`](src/delta_arms_description/scripts/gen_delta_3dof.py) and
[`gen_delta_4dof.py`](src/delta_arms_description/scripts/gen_delta_4dof.py); re-run them to regenerate
the URDFs after changing dimensions.

## Acknowledgements

- [ros2_closed_loop_demo](https://github.com/wiartallajan/ros2_closed_loop_demo)
- [gz_attach_links](https://github.com/oKermorgant/gz_attach_links)
- [joint_state_transformer_example](https://github.com/HIT-Robotics/joint_state_transformer_example)
  — source of the delta robot descriptions (Heilbronn University of Applied Sciences, supported by
  Autonox robotics GmbH).
