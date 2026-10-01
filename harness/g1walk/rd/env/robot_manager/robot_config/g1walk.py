# body-driver walking humanoid rig (大并行 §2 第 40 条,路 8): Isaac Lab's own G1_29DOF_CFG (Isaac 资产库 Robots/Unitree/G1/g1.usd, root free,
# Dex3 hands), legs and feet driven by the vendor walking controller NVIDIA WBC-AGILE velocity_height_g1 (Apache-2.0, task/RoboDojo/bd/agile_vh.py).
# The legs' and feet's motors are set the way that policy was trained (agile/rl_env/assets/robots/unitree_g1.py, G1_29DOF, used by
# velocity_height_env_cfg.py): DC motors with the kp / kd its LEAPP yaml / ONNX gives (hips 100, knees 200, ankles 20; 2.5 / 5 / 0.2, 0.1),
# effort limits 88 / 139 / 88 / 139 (hip yaw / roll / pitch, knee) and 50 (ankles), 10 rad/s velocity limit on the torque-speed curve and in PhysX,
# armature from the motor types (7520-14, 7520-22, 2 x 5020), stall torque 180 / 80; self-collisions on, as it was trained.
# (Isaac Lab's own G1_29DOF_CFG legs differ: hip roll 88 N m, 32 / 20 rad/s, armature 0.03 — that one is what Isaac Lab's older
#  agile_locomotion.pt was trained with; harness/g1walk/walk_test.py --policy old keeps it for comparison.)
# Waist, arms and hands stay as Isaac Lab's G1_29DOF_CFG has them (the walking controller does not drive them).
from isaaclab.actuators import DCMotorCfg
from isaaclab_assets.robots.unitree import G1_29DOF_CFG

ARMATURE_5020 = 0.003609725        # agile/rl_env/assets/robots/unitree_g1.py
ARMATURE_7520_14 = 0.010177520
ARMATURE_7520_22 = 0.025101925


def agile_leg_actuators():
    legs_effort = {".*_hip_yaw_joint": 88.0, ".*_hip_roll_joint": 139.0, ".*_hip_pitch_joint": 88.0, ".*_knee_joint": 139.0}
    return {
        "legs": DCMotorCfg(
            joint_names_expr=[".*_hip_yaw_joint", ".*_hip_roll_joint", ".*_hip_pitch_joint", ".*_knee_joint"],
            effort_limit=dict(legs_effort), effort_limit_sim=dict(legs_effort),
            velocity_limit=10.0, velocity_limit_sim=10.0,
            stiffness={".*_hip_pitch_joint": 100.0, ".*_hip_roll_joint": 100.0, ".*_hip_yaw_joint": 100.0, ".*_knee_joint": 200.0},
            damping={".*_hip_pitch_joint": 2.5, ".*_hip_roll_joint": 2.5, ".*_hip_yaw_joint": 2.5, ".*_knee_joint": 5.0},
            armature={".*_hip_pitch_joint": ARMATURE_7520_14, ".*_hip_roll_joint": ARMATURE_7520_22,
                      ".*_hip_yaw_joint": ARMATURE_7520_14, ".*_knee_joint": ARMATURE_7520_22},
            saturation_effort=180.0,
        ),
        "feet": DCMotorCfg(
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            effort_limit=50.0, effort_limit_sim=50.0, velocity_limit=10.0, velocity_limit_sim=10.0,
            stiffness={".*_ankle_pitch_joint": 20.0, ".*_ankle_roll_joint": 20.0},
            damping={".*_ankle_pitch_joint": 0.2, ".*_ankle_roll_joint": 0.1},
            armature=2.0 * ARMATURE_5020,
            saturation_effort=80.0,
        ),
    }


def get_robot_config():
    cfg = G1_29DOF_CFG.copy()
    cfg.spawn.articulation_props.fix_root_link = False
    cfg.spawn.articulation_props.enabled_self_collisions = True
    acts = dict(cfg.actuators)
    acts.update(agile_leg_actuators())
    cfg.actuators = acts
    return cfg
