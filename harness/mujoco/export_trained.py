# export_trained.py TASK DIR: simulator-side rig code, run once when a box is set up, with the venv Unitree's training
# project needs (mjlab 1.2.0, mujoco 3.5.0, mujoco-warp 3.5.0, warp-lang 1.12.1: /root/mj/train-venv). Writes the scene of
# one of Unitree's mjlab velocity tasks (Unitree-G1-Flat, Unitree-Go2-Flat) as its training builds it: the robot with its
# training's actuators (each joint's position servo, the armature of its motor, its effort limit), its collision geometry
# and contact settings, the terrain, and the training's simulation options (step, integrator, solver), so that the
# exam hall's robot is the one its maker's walking program learned to walk.
import pathlib
import sys

sys.path.insert(0, "/root/mj/unitree_rl_mjlab")
import mujoco
import mjlab.tasks  # noqa: F401  (the registry)
import src.tasks  # noqa: F401
from mjlab.scene import Scene
from mjlab.tasks.registry import load_env_cfg

task, out = sys.argv[1], pathlib.Path(sys.argv[2])
cfg = load_env_cfg(task)
cfg.scene.num_envs = 1
scene = Scene(cfg.scene, device="cpu")
o, c = scene.spec.option, cfg.sim.mujoco
o.timestep = c.timestep
o.integrator = getattr(mujoco.mjtIntegrator, "mjINT_" + c.integrator.upper())
o.cone = getattr(mujoco.mjtCone, "mjCONE_" + c.cone.upper())
o.solver = getattr(mujoco.mjtSolver, "mjSOL_" + c.solver.upper())
o.jacobian = getattr(mujoco.mjtJacobian, "mjJAC_" + c.jacobian.upper())
o.impratio, o.iterations, o.tolerance = c.impratio, c.iterations, c.tolerance
o.ls_iterations, o.ls_tolerance, o.ccd_iterations = c.ls_iterations, c.ls_tolerance, c.ccd_iterations
scene.write(out)
m = mujoco.MjModel.from_xml_path(str(out / "scene.xml"))
print("export_trained: %s to %s: %d joints, %d actuators, %.3f kg, a step of %.4f s, integrator %d, %d solver iterations"
      % (task, out, m.njnt, m.nu, m.body_subtreemass[0], m.opt.timestep, m.opt.integrator, m.opt.iterations))
