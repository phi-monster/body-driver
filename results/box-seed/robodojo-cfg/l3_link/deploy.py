import os, sys, numpy as np
from . import bd_scramble as _scr   # BD_SCRAMBLE (owner 10-10): what the driver is given is scrambled, see bd_scramble.py

# 单目深度 2026-09-26 删了(见 _fill_depth);BL_MDE_OFF 不再有用。
_MDE_OFF = os.environ.get("BL_MDE_OFF", "0") != "0"
_MDE_DUMP = [0]

# 🔴 真机配置:只有一部手机(头顶)+ 一只腕部 RGB 相机,没有任何深度。
#   BL_NO_DEPTH=1 ⇒ 每台相机的 depth 通道整个删掉(不是清零,是不存在),驱动认到的就是纯 RGB 相机;
#   BL_DROP_CAMS=cam_left_wrist,... ⇒ 这些相机从观测里删掉(真机左腕没有相机)。
_NO_DEPTH = os.environ.get("BL_NO_DEPTH", "0") != "0"
_DROP = [c for c in os.environ.get("BL_DROP_CAMS", "").split(",") if c]
_SAID = [0]

def _strip(obs):
    vis = obs.get("vision") if isinstance(obs, dict) else None
    if not isinstance(vis, dict):
        return obs
    for c in _DROP:
        vis.pop(c, None)
    if _NO_DEPTH:
        for k, c in vis.items():
            if isinstance(c, dict):
                c.pop("depth", None)
    if _SAID[0] < 1:
        print("[相机] 交给驱动的相机:", list(vis.keys()), "| 深度:", "无" if _NO_DEPTH else "有", flush=True)
        _SAID[0] += 1
    return obs

def _fill_depth(obs):
    # 2026-09-26:单目深度删了(09-12 试过:米数不可信、手上相机里连远近都是反的;驱动也不读身体给的深度)。
    # 这里只剩按开关删掉深度通道 / 相机。
    if _NO_DEPTH or _DROP:
        obs = _strip(obs)
    return obs


# BD_TIMING=1 (10-09, development only): where a beat of the simulator goes. Each part is wrapped
# with a clock and summed; every 200 beats one line on the simulator log gives the milliseconds a
# beat of each. Nothing changes what the simulator does.
import time as _time
_TIMING = os.environ.get("BD_TIMING", "0") != "0"
_T = {}
_C = {}
_BEATS = [0]

def _timed(owner, name, label):
    f = getattr(owner, name)
    def wrapped(*a, **k):
        t0 = _time.perf_counter()
        try:
            return f(*a, **k)
        finally:
            _T[label] = _T.get(label, 0.0) + (_time.perf_counter() - t0)
            _C[label] = _C.get(label, 0) + 1
    setattr(owner, name, wrapped)

def _instrument(env, client):
    s = getattr(env, "sim", None)
    parts = [(env, "take_action", "action"), (env, "step", "step"), (env, "sim_step", "sim_step"),
             (env, "get_obs", "obs"), (env, "render", "render_all"), (env, "_stream_vision", "video"),
             (env, "is_episode_end", "episode_end")]
    _call = client.call
    def _timed_call(*a, **k):
        t0 = _time.perf_counter()
        try:
            return _call(*a, **k)
        finally:
            label = "call_" + str(k.get("func_name", a[0] if a else "?"))
            _T[label] = _T.get(label, 0.0) + (_time.perf_counter() - t0)
            _C[label] = _C.get(label, 0) + 1
    client.call = _timed_call
    if s is not None:
        parts += [(getattr(s, "scene", None), "write_data_to_sim", "write"), (getattr(s, "sim", None), "step", "physics"),
                  (getattr(s, "sim", None), "render", "render"), (getattr(s, "scene", None), "update", "scene_update")]
    for o in ("obs_manager", "reward_manager"):
        m = getattr(env, o, None)
        if m is not None:
            parts.append((m, "get_obs" if o == "obs_manager" else "step", o))
    cm = getattr(getattr(env, "robot_manager", None), "control_manager", None)
    if cm is not None:
        parts += [(cm, "push", "control_push"), (cm, "pop", "control_pop")]
    for owner, name, label in parts:
        try:
            _timed(owner, name, label)
        except Exception as e:
            print("[timing] cannot time", label, repr(e), flush=True)

def _beat_done():
    _BEATS[0] += 1
    n = _BEATS[0]
    if n % 200 == 0:
        print("[timing] beats %d: " % n + ", ".join("%s %.1f ms (%.1f calls)" % (k, 1000.0 * v / n, _C[k] / n)
                                                   for k, v in sorted(_T.items(), key=lambda kv: -kv[1])), flush=True)

# BD_NO_VIDEO=1 (10-09, development only): no video of the run is written; nothing the robot or
# the driver sees changes (the frames went only to the ffmpeg streams).
_NO_VIDEO = os.environ.get("BD_NO_VIDEO", "0") != "0"

# BD_RENDERS=k (10-09, development only): each observation renders k frames instead of one, the
# scene unchanged between them (no physics step), so the renderer's temporal accumulation (DLAA, the
# denoisers) converges k frames a beat, as a camera's picture is whole the frame it is taken. The
# rig renders only when an observation is taken (eval_env.get_obs_batch; sim_step is called with
# render=False, so render_interval changes nothing). Exam runs leave it unset.
_RENDERS = max(1, int(os.environ.get("BD_RENDERS", "1")))

# BD_SWITCH=seconds (10-09, development only): the interpreter's thread switch interval. Each call to
# the driver hands the GIL from this thread to the websocket client's event-loop thread and back,
# and at the default 5 ms each handoff can wait out the interval: a get_action of a few bytes took
# 18.6 ms a beat (A51). Nothing the simulator computes changes.
if os.environ.get("BD_SWITCH"):
    sys.setswitchinterval(float(os.environ["BD_SWITCH"]))

def eval_one_episode(TASK_ENV, model_client):
    if _NO_VIDEO:
        TASK_ENV._stream_vision = lambda env_idx, frame: None
    if _RENDERS > 1 and not getattr(TASK_ENV, "_bd_renders", False):
        _render_once = TASK_ENV.render
        def _render_k(*a, **k):
            for _ in range(_RENDERS - 1):
                _render_once(*a, **k)
            return _render_once(*a, **k)
        TASK_ENV.render = _render_k
        TASK_ENV._bd_renders = True
    if _TIMING and not getattr(TASK_ENV, "_bd_timed", False):
        _instrument(TASK_ENV, model_client)
        TASK_ENV._bd_timed = True
    model_client.call(func_name="reset")

    while not TASK_ENV.is_episode_end():
        obs = _scr.scramble_obs(_fill_depth(TASK_ENV.get_obs()))
        model_client.call(func_name="update_obs", obs=obs)
        actions = model_client.call(func_name="get_action")

        if actions is None:
            break
        # 单个动作 ⇒ 包一层;动作块 ⇒ 原样。判据是"第一个元素是不是序列",不假设维度。
        if len(actions) == 0:
            break
        first = actions[0]
        if not hasattr(first, "__len__"):
            actions = [actions]

        for action_idx, action in enumerate(actions):
            TASK_ENV.take_action(_scr.unscramble_action(action))
            if _TIMING:
                _beat_done()
            if TASK_ENV.is_episode_end() or action_idx + 1 == len(actions):
                break
            obs = _scr.scramble_obs(_fill_depth(TASK_ENV.get_obs()))
            model_client.call(func_name="update_obs", obs=obs)


def eval_one_episode_batch(TASK_ENV, model_client):
    raise NotImplementedError("l3_link 只跑单环境(deploy.yml 里 eval_batch: false)")
