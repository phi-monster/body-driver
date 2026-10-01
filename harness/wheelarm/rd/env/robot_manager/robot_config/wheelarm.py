import math

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
import isaaclab.sim as sim_utils

from env.global_configs import ROBOTS_PATH

# body-driver wheelarm rig (大并行 §2 第 39 条,路 8). The base is free (wheels on the floor), not fixed like the arm rigs.
# Rest pose: arm folded forward and down so the wrist eye looks at the floor ahead (shoulder 30°, elbow 90°, wrist 40°, from upright).
REST = {"wheel_left_joint": 0.0, "wheel_right_joint": 0.0, "arm_joint1": 0.0, "arm_joint2": math.radians(30.0),
        "arm_joint3": math.radians(90.0), "arm_joint4": math.radians(40.0), "arm_joint5": 0.0,
        "finger_left_joint": 0.0, "finger_right_joint": 0.0}


def get_robot_config():
    return ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ROBOTS_PATH}/wheelarm/wheelarm.usd",
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False, max_depenetration_velocity=5.0),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=1,
                fix_root_link=False,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos=REST,
            joint_vel={".*": 0.0},
            pos=(0.0, -1.0, 0.08),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        actuators={
            # Wheels take position targets like every other joint (the commanded angle runs ahead of the wheel; how far ahead sets the push).
            # 7.5 kg on two 5 cm wheels: 0.5 m/s² takes 0.19 N·m a wheel, i.e. a target 0.02 rad ahead at 10 N·m/rad.
            "wheels": ImplicitActuatorCfg(
                joint_names_expr=["wheel_.*_joint"], effort_limit_sim=5.0, velocity_limit_sim=20.0, stiffness=10.0, damping=1.0,
            ),
            # Arm: the shoulder carries about 1.3 kg at 0.3 m (4 N·m) ⇒ sags 0.01 rad at 400 N·m/rad.
            "arm": ImplicitActuatorCfg(
                joint_names_expr=["arm_joint[1-5]"], effort_limit_sim=50.0, velocity_limit_sim=3.0, stiffness=400.0, damping=40.0,
            ),
            "fingers": ImplicitActuatorCfg(
                joint_names_expr=["finger_.*_joint"], effort_limit_sim=20.0, velocity_limit_sim=0.2, stiffness=400.0, damping=20.0,
            ),
        },
    )
