# -*- coding: utf-8 -*-
import io
p = "/root/RoboDojo/env/planner_manager/curobo_planner.py"
s = io.open(p, encoding="utf-8").read()
if "[bd] frame reconciliation" not in s:
    old = '''        target_pose_p, target_pose_q = self._trans_from_world_to_base(world_base_pose, world_target_pose)
        target_pose_p[0] += self.frame_bias[0]
        target_pose_p[1] += self.frame_bias[1]
        target_pose_p[2] += self.frame_bias[2]
        goal_tool_poses = self._build_goal_pose(target_pose_p, target_pose_q)
        current_state = self._build_joint_state(curr_joint_pos)
'''
    new = '''        target_pose_p, target_pose_q = self._trans_from_world_to_base(world_base_pose, world_target_pose)
        # [bd] frame reconciliation (2026-09-25): the observed end-effector pose (the simulator's body pose of the ee link) and
        # curobo's FK of the same joints can differ by a constant offset when the USD and URDF link origins differ
        # (G1 + Inspire: 0.149 m => every target unreachable, continuous IK never converged, right arm never moved).
        # Measure the offset at the current joints and express the target in curobo's own FK frame. Zero for x5/franka.
        try:
            _obs_p, _obs_q = self._trans_from_world_to_base(world_base_pose, np.array(real_robot_pose, dtype=np.float32))
            _st = self._build_joint_state(curr_joint_pos)
            _kin = self.motion_planner.compute_kinematics(_st)
            _pose = _kin.tool_poses.to_dict()[self.ee_link]
            _fk_p = np.asarray(_pose.position.reshape(-1)[:3].detach().cpu(), dtype=np.float32)
            _off = _fk_p - np.asarray(_obs_p, dtype=np.float32).reshape(3)
            if float(np.linalg.norm(_off)) > 0.005:
                if not getattr(self, "_bd_said_off", False):
                    print(f"[bd] observed ee vs curobo FK differ by {float(np.linalg.norm(_off)):.4f} m; targets shifted into the FK frame", flush=True)
                    self._bd_said_off = True
                target_pose_p = (np.asarray(target_pose_p, dtype=np.float32).reshape(3) + _off).astype(np.float32)
        except Exception as _ex:
            print("[bd] frame reconciliation failed:", repr(_ex), flush=True)
        target_pose_p[0] += self.frame_bias[0]
        target_pose_p[1] += self.frame_bias[1]
        target_pose_p[2] += self.frame_bias[2]
        goal_tool_poses = self._build_goal_pose(target_pose_p, target_pose_q)
        current_state = self._build_joint_state(curr_joint_pos)
'''
    assert s.count(old) == 1, "anchor"
    s = s.replace(old, new)
    io.open(p, "w", encoding="utf-8").write(s)
    print("patched")
else:
    print("already patched")
