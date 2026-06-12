#!/usr/bin/env python3
"""joint_state_transformer + a closed-loop preview expander, in ONE process.

MoveIt plans only the phantom chain, so its planned-path preview moves tool0 but leaves the
real arm links frozen (MoveIt cannot solve the loop). This subclass reuses the transformer's
already-loaded robot_model + Levenberg-Marquardt IK to expand MoveIt's preview trajectory
(/display_planned_path, in phantom EE joints) into a FULL trajectory over every joint
(active + passive), and republishes it on /display_planned_path_full for a ghost MoveIt
Trajectory display. No extra node/process and no second model load.
"""
import numpy as np
import rclpy
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robot_model import levenbergMarquardt
from joint_state_transformer.joint_state_transformer import JointStateTransformer


class TransformerWithPreview(JointStateTransformer):
    def __init__(self):
        super().__init__()
        self.declare_parameter('preview_input_topic', '/display_planned_path')
        self.declare_parameter('preview_output_topic', '/display_planned_path_full')
        in_t = self.get_parameter('preview_input_topic').value
        out_t = self.get_parameter('preview_output_topic').value
        self.preview_pub = self.create_publisher(DisplayTrajectory, out_t, 1)
        self.preview_sub = self.create_subscription(DisplayTrajectory, in_t, self.preview_cb, 1)
        self.get_logger().info(f'closed-loop preview expander active: {in_t} -> {out_t}')

    def preview_cb(self, msg: DisplayTrajectory) -> None:
        if not self.initialized:
            return
        out = DisplayTrajectory()
        out.model_id = msg.model_id
        out.trajectory_start = msg.trajectory_start  # start state already carries full /joint_states
        for rt in msg.trajectory:
            jt = rt.joint_trajectory
            if not jt.joint_names or not jt.points:
                continue
            try:
                mask_int = self.robot_model.joints.getMask(list(jt.joint_names))
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'preview getMask failed for {list(jt.joint_names)}: {e}')
                continue
            mask = mask_int.astype(bool)
            full = JointTrajectory()
            full.header = jt.header
            full.joint_names = list(self._joint_names)
            state = self.last_state.copy()
            for p in jt.points:
                state[mask] = p.positions
                result, ok = levenbergMarquardt(
                    self.robot_model, mask_int, state,
                    tol=self.tolerance, max_iterations=self.max_iter,
                    max_step_line_search=self.max_step_line_search)
                if not ok:
                    continue
                state = result
                fp = JointTrajectoryPoint()
                fp.positions = list(result)
                fp.time_from_start = p.time_from_start
                full.points.append(fp)
            new_rt = RobotTrajectory()
            new_rt.joint_trajectory = full
            out.trajectory.append(new_rt)
        self.preview_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TransformerWithPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()  # idempotent: avoids "rcl_shutdown already called" on SIGINT


if __name__ == '__main__':
    main()
