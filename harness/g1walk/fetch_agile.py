# -*- coding: utf-8 -*-
"""第 40 条(路 8):把 NVIDIA WBC-AGILE 的 G1 下半身策略 velocity_height_g1(Apache-2.0)取下来,核过 sha256 再放。
出处:github.com/nvidia-isaac/WBC-AGILE,agile/data/policy/velocity_height_g1/
  - unitree_g1_velocity_height_history_torchscript.pt:推理只用它(400 → 512 → 256 → 128 → 12 的 MLP,ELU,不带归一化;
    输入是 5 帧历史拼好的观测,输出 12 个腿关节的原始动作 —— 怎么拼、怎么换成目标见 rd/bd/agile_vh.py)
  - leapp/Velocity-Height-G1-History-v0/ 下的 .yaml(接口说明:输入、输出、关节顺序、kp / kd、50 Hz、5 帧历史)、.onnx(同一个策略连
    观测处理、历史、动作换算一起导出的整张图)、_initial_values.safetensors(历史的初值)—— 只拿来逐项核对我们自己拼的那一套(agile_check.py),
    不拿来跑。
sha256 是 2026-10-01 从 main 取下来时量的(TorchScript 的 = 仓库里 LFS 指针写的 oid;ONNX 的 = yaml 里写的 sha256sum)。
用法:python fetch_agile.py <放到哪个目录>
"""
import hashlib
import os
import sys
import urllib.request

MEDIA = "https://media.githubusercontent.com/media/nvidia-isaac/WBC-AGILE/main/agile/data/policy/velocity_height_g1/"
RAW = "https://raw.githubusercontent.com/nvidia-isaac/WBC-AGILE/main/agile/data/policy/velocity_height_g1/"
LEAPP = "leapp/Velocity-Height-G1-History-v0/"
FILES = [  # (url, 存成的名字, sha256)
    (MEDIA + "unitree_g1_velocity_height_history_torchscript.pt", "unitree_g1_velocity_height_history_torchscript.pt",
     "240a5ce0b121837eba2f886a523d284a2a263dceb78d3132639eaf74ad7650f2"),
    (RAW + LEAPP + "Velocity-Height-G1-History-v0.yaml", "Velocity-Height-G1-History-v0.yaml",
     "9cec22be77deb10a756b89de6b6a456e4d2ec5539b37cce87f0a5baaadcb3e58"),
    (MEDIA + LEAPP + "Velocity-Height-G1-History-v0.onnx", "Velocity-Height-G1-History-v0.onnx",
     "f9a8dfcb8cb8bb2816b92cf893650c53dea30bb041f4919a85f1cf73acbb29fc"),
    (RAW + LEAPP + "Velocity-Height-G1-History-v0_initial_values.safetensors", "Velocity-Height-G1-History-v0_initial_values.safetensors",
     "28ee6046fc0be3bd25fb4988e623543ea84a50519ac3db91b921dec492311af5"),
]


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def fetch(dst_dir, names=None):
    os.makedirs(dst_dir, exist_ok=True)
    for url, name, want in FILES:
        if names is not None and name not in names:
            continue
        p = os.path.join(dst_dir, name)
        if os.path.exists(p) and sha(p) == want:
            print("已经在、核过:", p)
            continue
        tmp = p + ".part"
        with urllib.request.urlopen(url, timeout=120) as r, open(tmp, "wb") as f:
            f.write(r.read())
        got = sha(tmp)
        if got != want:
            os.remove(tmp)
            raise SystemExit(f"{name} 的 sha256 对不上:{got} ≠ {want}(没放)")
        os.replace(tmp, p)
        print("取下来、核过:", p)


if __name__ == "__main__":
    fetch(sys.argv[1] if len(sys.argv) > 1 else ".")
