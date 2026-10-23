#!/bin/bash
# setup.sh: everything the MuJoCo exam hall needs on a box, in /root/mj, with this repository at /root/bd (simulator-side
# rig code; run once, as root, under nohup: the Go2's training alone keeps one GPU busy for hours). The versions are
# the ones the hall was built and measured with (10-23); each project is checked out at the commit used then.
#   1. The hall's venv (/root/mj/venv): MuJoCo and what robot.py needs; the Menagerie's robot models.
#   2. The G1's walking: NVIDIA's decoupled whole-body controller (GR00T-WholeBodyControl): its two released policies and
#      the meshes of its G1 (git-lfs).
#   3. The Go2's walking: Unitree's mjlab training project in its own venv (/root/mj/train-venv, its pinned mjlab 1.2.0
#      and mujoco-warp 3.5.0, with the MuJoCo and Warp of their release: 3.5.0 and 1.12.1; newer ones break mjlab), the
#      Go2 as its training builds it (export_trained.py), its policy trained with Unitree's recipe (Unitree-Go2-Flat:
#      4096 robots, 10001 iterations, one GPU) and written as Unitree's deploy program reads it (deploy_params.py).
set -e
mkdir -p /root/mj && cd /root/mj
HALL=/root/bd/harness/mujoco

checkout() {   # REPO URL COMMIT: a shallow clone at that commit
  [ -d "$1" ] || git clone -q --depth 1 "$2" "$1"
  git -C "$1" fetch -q --depth 1 origin "$3" && git -C "$1" -c advice.detachedHead=false checkout -q "$3"
}

python3 -m venv venv
venv/bin/pip install --quiet mujoco==3.16.0 numpy==2.5.4 msgpack==1.2.3 websocket-client==1.9.2 websockets==17.3 \
  pillow==13.0.0 onnxruntime==1.31.0 PyYAML==6.0.3
checkout menagerie https://github.com/google-deepmind/mujoco_menagerie.git cda5236f98383f6fbaa4820c6b18dba892cd087f

GIT_LFS_SKIP_SMUDGE=1 checkout GR00T-WholeBodyControl https://github.com/NVlabs/GR00T-WholeBodyControl.git \
  45c03facbf848e0c743091b7d9c17952096ff908
(cd GR00T-WholeBodyControl && git lfs pull --include="decoupled_wbc/sim2mujoco/resources/robots/g1/policy/*.onnx,decoupled_wbc/sim2mujoco/resources/robots/g1/meshes/*")

checkout unitree_rl_mjlab https://github.com/unitreerobotics/unitree_rl_mjlab.git 1425b15f73bd4095f0df53709d7c389c3eb9e790
python3 -m venv train-venv
train-venv/bin/pip install --quiet --upgrade pip wheel
(cd unitree_rl_mjlab && ../train-venv/bin/pip install --quiet -e .)
train-venv/bin/pip install --quiet mujoco==3.5.0 warp-lang==1.12.1 scipy==1.18.1
train-venv/bin/python $HALL/export_trained.py Unitree-Go2-Flat /root/mj/go2_trained
mkdir -p train_logs
(cd unitree_rl_mjlab && CUDA_VISIBLE_DEVICES=0 ../train-venv/bin/python scripts/train.py Unitree-Go2-Flat \
  --env.scene.num-envs 4096 --agent.logger tensorboard > /root/mj/train_logs/go2_flat.log 2>&1)
RUN=$(ls -d unitree_rl_mjlab/logs/rsl_rl/go2_velocity/*/ | tail -1)
train-venv/bin/python $HALL/deploy_params.py "$RUN" \
  unitree_rl_mjlab/deploy/robots/go2/config/policy/velocity/v0/params/deploy.yaml /root/mj/firmware/go2
echo "setup: done $(date +%FT%T)"
