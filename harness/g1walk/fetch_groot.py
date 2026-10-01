# -*- coding: utf-8 -*-
"""第 40 条(路 8):把 NVIDIA GR00T-WholeBodyControl 的 Decoupled WBC 下半身策略(GR00T N1.5 / N1.6 用的那一份)取下来,核过 sha256 再放。
出处:github.com/NVlabs/GR00T-WholeBodyControl,decoupled_wbc/sim2mujoco/resources/robots/g1/
  - policy/GR00T-WholeBodyControl-Balance.onnx(站着用:速度命令的模 ≤ 0.05)、policy/GR00T-WholeBodyControl-Walk.onnx(走的时候用);
    输入一串 516 = 86 × 6 帧历史,输出 15 个(两条腿 12 + 腰 3)的原始动作 —— 怎么拼、怎么换成目标见 rd/bd/groot_wbc.py;
  - g1_gear_wbc.yaml(接口的数:各项比例、默认姿势、kp / kd、物理步 0.005 × 4、历史 6 帧、开局的胯高 0.74)、
    g1_gear_wbc.xml(它自己验的 MuJoCo 身体:关节顺序、每个关节的力矩上限、armature)、
    scripts/run_mujoco_gear_wbc.py(它自己的 MuJoCo 跑法:观测怎么拼、两份策略怎么换、力矩怎么算 —— 我们照这一份接);
  - policy/NVIDIA Open Model License(权重的许可:能商用、能再分发,分发时带上这份许可和一句 Notice)。
代码是 Apache-2.0,权重是 NVIDIA Open Model License。
sha256:两份 ONNX 的 = 仓库里 LFS 指针写的 oid;其余几个是 2026-10-01 从 main 取下来时量的。
用法:python fetch_groot.py <放到哪个目录>
"""
import hashlib
import os
import sys
import urllib.request

REPO = "NVlabs/GR00T-WholeBodyControl"
D = "decoupled_wbc/sim2mujoco/resources/robots/g1/"
MEDIA = f"https://media.githubusercontent.com/media/{REPO}/main/"
RAW = f"https://raw.githubusercontent.com/{REPO}/main/"
FILES = [  # (url, 存成的名字, sha256)
    (MEDIA + D + "policy/GR00T-WholeBodyControl-Balance.onnx", "GR00T-WholeBodyControl-Balance.onnx",
     "f645da599d4ca3d29ed273c8f4712620bb680d34977469ca3aeabe5bb9631c18"),
    (MEDIA + D + "policy/GR00T-WholeBodyControl-Walk.onnx", "GR00T-WholeBodyControl-Walk.onnx",
     "7c82255b6905ffcc4468fa7f8ddcf7b70db168cf1042107ccab887cb6a8e5407"),
    (RAW + D + "g1_gear_wbc.yaml", "g1_gear_wbc.yaml",
     "31226a224ca8450e89d9ce17d5cb31c052192a9cbfedc8d145f1cb95627ac7a2"),
    (RAW + D + "g1_gear_wbc.xml", "g1_gear_wbc.xml",
     "3b0b5a1c8299fda85cf328cc2a8df53ccc765ce37f20030175295049947b1a19"),
    (RAW + "decoupled_wbc/sim2mujoco/scripts/run_mujoco_gear_wbc.py", "run_mujoco_gear_wbc.py",
     "b78dfb546ee250116b3853f96a12f82174aca248808da33caf199ec8e42f82fd"),
    (RAW + D + "policy/NVIDIA%20Open%20Model%20License", "NVIDIA Open Model License",
     "a0c4f6a35dd2c858c86069d84140052831820e78f3074a2fb758bb4156b66994"),
]


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def fetch(dst_dir):
    os.makedirs(dst_dir, exist_ok=True)
    got = {}
    for url, name, want in FILES:
        p = os.path.join(dst_dir, name)
        if not (os.path.exists(p) and want and sha(p) == want):
            tmp = p + ".part"
            with urllib.request.urlopen(url, timeout=120) as r, open(tmp, "wb") as f:
                f.write(r.read())
            h = sha(tmp)
            if want and h != want:
                os.remove(tmp)
                raise SystemExit(f"{name} 的 sha256 对不上:{h} ≠ {want}(没放)")
            os.replace(tmp, p)
        got[name] = sha(p)
        print("%s  %s%s" % (got[name], name, "" if want else "(没登记 sha256:头一回取的,登记上)"))
    return got


if __name__ == "__main__":
    fetch(sys.argv[1] if len(sys.argv) > 1 else ".")
