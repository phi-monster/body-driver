# What lived only on the vast box (saved 10-12, before the box was reclaimed)

- `l3_link/`: RoboDojo's `XPolicyLab/policy/l3_link/deploy.py` as we ran it (the BD_NO_VIDEO / BD_TIMING / BD_RENDERS /
  BD_SWITCH switches, the scrambler hook), its `.bak_*` history, `deploy.yml`, and `bd_scramble.py` (also in
  `harness/robodojo_scramble/`). Installed over the stock file at that path on a RoboDojo box.
- `env_cfg/`: the `arx_x5*.yml` configurations (`arx_x5.yml` official; `_fabric` the development one, see the plan's
  sim beat cost notes; `_gpu`, `_fast`, `_still`, `_still2`, `_nohead`, `_dev`) and the camera configurations, for
  `/root/RoboDojo/env_cfg/`.
- `instruments/`: `run.sh` and the scripts of the instrument service's directory (the RoMa / SAM 2.1 service of
  docs/instrument-service.md); the weights and the sam2 clone are public downloads.
- `work_core/`: the box-side helper scripts (launching fakes, status, watchdogs, gprof replays).
- `harness/run/qnew.sh qrec.sh qscore.sh fake.sh fakes.sh` were already here and match the box's copies.

The recordings and the owner's keep list went to the private Hugging Face dataset `phi-monster/box-salvage-2026-10`.
