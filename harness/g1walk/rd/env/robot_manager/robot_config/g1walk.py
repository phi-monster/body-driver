# body-driver walking humanoid rig (大并行 §2 第 40 条,路 8): Isaac Lab's own G1_29DOF_CFG (Isaac 资产库 Robots/Unitree/G1/g1.usd, root free,
# actuators as Isaac Lab trained its Agile lower-body policy with) — the same body the vendor walking controller was trained on.
from isaaclab_assets.robots.unitree import G1_29DOF_CFG


def get_robot_config():
    cfg = G1_29DOF_CFG.copy()
    cfg.spawn.articulation_props.fix_root_link = False
    return cfg
