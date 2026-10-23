# deploy_params.py RUN TEMPLATE OUT: simulator-side rig code, run with the training venv (/root/mj/train-venv) once a
# walking program has trained. Writes OUT/exported/policy.onnx and OUT/params/deploy.yaml in the form Unitree's deploy
# program reads (deploy/robots/*/config/policy/velocity/v0), so that legged.py runs it as it runs Unitree's own: from
# the run (RUN: its last exported policy, which names its joints and carries their gains, defaults and action scales,
# and params/env.yaml: its step and the joints its actions drive) and from Unitree's deploy.yaml for the same robot
# (TEMPLATE: the SDK's joint order, the speeds the deploy program allows, the observation's terms).
import os
import re
import shutil
import sys

import onnx
import yaml

run, template, out = sys.argv[1], sys.argv[2], sys.argv[3]
meta = {p.key: p.value for p in onnx.load(os.path.join(run, "policy.onnx")).metadata_props}
names = meta["joint_names"].split(",")
floats = lambda key: [float(x) for x in meta[key].split(",")]
# The run's own record, written by mjlab with Python's tags (tuples, slices, its own classes): each tagged value read
# as the plain list, map or text it holds.
class Plain(yaml.SafeLoader):
    pass


def plain(loader, suffix, node):
    if isinstance(node, yaml.SequenceNode):
        return loader.construct_sequence(node, deep=True)
    if isinstance(node, yaml.MappingNode):
        return loader.construct_mapping(node, deep=True)
    return loader.construct_scalar(node)


Plain.add_multi_constructor("tag:yaml.org,2002:python/", plain)
env = yaml.load(open(os.path.join(run, "params", "env.yaml")), Loader=Plain)
t = yaml.safe_load(open(template))

act = env["actions"]["joint_pos"]
patterns = act["actuator_names"]
driven = [k for k, n in enumerate(names) if any(re.fullmatch(p, n) for p in patterns)]   # mjlab: natural order
default = floats("default_joint_pos")
scale = floats("action_scale")
if len(scale) == 1:
    scale = scale * len(driven)   # one scale for every joint (a task's scale given as a number)
if len(scale) != len(driven):
    raise SystemExit("deploy_params: %d action scales for %d driven joints" % (len(scale), len(driven)))

d = {
    "joint_ids_map": t["joint_ids_map"],
    "step_dt": float(env["sim"]["mujoco"]["timestep"]) * int(env["decimation"]),
    "stiffness": floats("joint_stiffness"),
    "damping": floats("joint_damping"),
    "default_joint_pos": default,
    "commands": t["commands"],
    "actions": {"JointPositionAction": {"clip": None, "joint_names": list(patterns), "scale": scale,
                                        "offset": [default[k] for k in driven],
                                        "joint_ids": None if len(driven) == len(names) else driven}},
    "observations": t["observations"],
}
d["observations"]["last_action"]["scale"] = [1.0] * len(driven)
for k in ("joint_pos_rel", "joint_vel_rel"):
    d["observations"][k]["scale"] = [1.0] * len(names)
os.makedirs(os.path.join(out, "exported"), exist_ok=True)
os.makedirs(os.path.join(out, "params"), exist_ok=True)
shutil.copy(os.path.join(run, "policy.onnx"), os.path.join(out, "exported", "policy.onnx"))
with open(os.path.join(out, "params", "deploy.yaml"), "w") as f:
    yaml.safe_dump(d, f, sort_keys=False, default_flow_style=None)
print("deploy_params: %s: %d joints, %d driven (%s), a step of %.3f s" % (out, len(names), len(driven),
      "all" if len(driven) == len(names) else ",".join(names[k] for k in driven), d["step_dt"]))
