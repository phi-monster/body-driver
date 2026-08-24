#!/usr/bin/env python3
"""RoboDojo GPU 物理修补(幂等)。

症状:sim 配置里 device: cuda ⇒ 开场即
      TypeError: can't convert cuda:0 device type tensor to numpy
根因:reward_manager/func_parser.py 直接把 get_instance_pose 返回的张量喂给 np.concatenate,
      而在 GPU 物理下那是 cuda 张量。上游只在 CPU 上跑过(base_env.py:15 DEFAULT_SIM_DEVICE="cpu"),
      所以这条路从没被走过。
用法:cd /root/RoboDojo && python3 <本文件>
"""
import io, sys

P = "env/reward_manager/func_parser.py"
MARK = "GPU 物理修:上游只在 CPU 上跑过"
OLD = "                    pose = np.concatenate([pos, rot])"
NEW = (
    "                    # " + MARK + ",这里拿到的可能是 cuda 张量。\n"
    "                    _cpu = lambda t: np.asarray(t.detach().cpu() if hasattr(t, \"detach\") else t).reshape(-1)\n"
    "                    pose = np.concatenate([_cpu(pos), _cpu(rot)])"
)

s = io.open(P, encoding="utf-8").read()
if MARK in s:
    print("已经打过,跳过"); sys.exit(0)
if OLD not in s:
    print("🔴 上游改过这段,别盲改"); sys.exit(1)
io.open(P, "w", encoding="utf-8").write(s.replace(OLD, NEW, 1))
print("patched")
