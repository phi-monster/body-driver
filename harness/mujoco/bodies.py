# bodies.py: simulator-side rig code (not the driver). The real robots of the MuJoCo Menagerie the driver is tested
# on, each given as its maker's interface gives it: joints that go to a position they are sent (the model's own servos,
# or a position loop of the rig's around a model's torque motors, as a humanoid's or a quadruped's controller board
# runs it), a wheeled base and a drone at velocity setpoints with their own loops inside. Each has cameras where such a
# robot carries them and one camera standing in the room, and the truth a scorer judges the driver against: each
# group's role and each camera's mount. Nothing here reaches the driver but what robot.py sends: readings and pictures.
import math
import numpy as np
import mujoco

MENAGERIE = "/root/mj/menagerie"


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def look_axes(forward, up=(0.0, 0.0, 1.0)):
    """MuJoCo's xyaxes for a camera looking along forward, its picture's up towards up (a camera looks along its -z)."""
    f = unit(forward)
    right = np.cross(f, np.asarray(up, float))
    if np.linalg.norm(right) < 1e-6:
        right = np.cross(f, np.array([1.0, 0.0, 0.0]))
    right = unit(right)
    upv = np.cross(right, f)
    return [float(x) for x in list(right) + list(upv)]


def find_body(spec, name):
    for b in spec.bodies:
        if b.name == name:
            return b
    raise KeyError("no body " + name)


def world_camera(spec, rng, center, size):
    """One camera standing in the room, somewhere around the robot at a distance and height drawn from the seed,
    looking at the robot's middle (drawn a little off it)."""
    az = rng.uniform(-math.pi, math.pi)
    d = size * rng.uniform(1.4, 2.2)
    h = size * rng.uniform(0.5, 1.2)
    c = np.asarray(center, float)
    pos = c + np.array([d * math.cos(az), d * math.sin(az), h])
    target = c + rng.uniform(-0.1, 0.1, 3) * size
    spec.worldbody.add_camera(name="room", pos=[float(x) for x in pos], xyaxes=look_axes(target - pos),
                              fovy=float(rng.uniform(50.0, 70.0)))
    return {"name": "room", "mount": "WORLD_FIXED", "by": None}


def body_camera(spec, body, name, pos, forward, up, rng, mount, by):
    """A camera fixed on one of the robot's bodies, its place and direction in that body's frame, a little drawn."""
    b = find_body(spec, body)
    p = np.asarray(pos, float) + rng.uniform(-0.01, 0.01, 3)
    f = unit(forward) + rng.uniform(-0.08, 0.08, 3)
    b.add_camera(name=name, pos=[float(x) for x in p], xyaxes=look_axes(f, up), fovy=float(rng.uniform(55.0, 80.0)))
    return {"name": name, "mount": mount, "by": by}


def lights(spec, rng):
    """A light from above, drawn a little off straight down, so every scene is lit as a room is (the Stretch scene had
    none and its cameras saw black)."""
    d = unit([rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), -1.0])
    light = spec.worldbody.add_light()
    light.name = "room_light"
    light.pos = [0.0, 0.0, 4.0]
    light.dir = [float(x) for x in d]
    if hasattr(mujoco, "mjtLightType"):
        light.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    else:
        light.directional = True
    light.diffuse = [0.7, 0.7, 0.7]
    light.specular = [0.2, 0.2, 0.2]
    light.castshadow = True
    spec.visual.headlight.ambient = [0.3, 0.3, 0.3]
    spec.visual.headlight.diffuse = [0.4, 0.4, 0.4]


def room(spec):
    """A floor of squares and a sky, as the Menagerie's other scenes have them, where a scene has none (Stretch's
    floor is plain black, its sky none: its cameras saw black)."""
    sky = spec.add_texture()
    sky.name, sky.type, sky.builtin = "bd_sky", mujoco.mjtTexture.mjTEXTURE_SKYBOX, mujoco.mjtBuiltin.mjBUILTIN_GRADIENT
    sky.rgb1, sky.rgb2, sky.width, sky.height = [0.3, 0.5, 0.7], [0.0, 0.0, 0.0], 512, 3072
    grid = spec.add_texture()
    grid.name, grid.type, grid.builtin = "bd_grid", mujoco.mjtTexture.mjTEXTURE_2D, mujoco.mjtBuiltin.mjBUILTIN_CHECKER
    grid.rgb1, grid.rgb2, grid.width, grid.height = [0.2, 0.3, 0.4], [0.1, 0.2, 0.3], 300, 300
    grid.mark, grid.markrgb = mujoco.mjtMark.mjMARK_EDGE, [0.8, 0.8, 0.8]
    mat = spec.add_material()
    mat.name, mat.texuniform, mat.texrepeat, mat.reflectance = "bd_grid", True, [5.0, 5.0], 0.2
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "bd_grid"
    for g in spec.geoms:
        if g.type == mujoco.mjtGeom.mjGEOM_PLANE:
            g.material = "bd_grid"


def props(spec, rng, center, size, count):
    """Boxes of drawn colours and sizes on the floor around the robot: things for its cameras to see move."""
    c = np.asarray(center, float)
    for i in range(count):
        r = size * rng.uniform(0.35, 1.0)
        a = rng.uniform(-math.pi, math.pi)
        s = size * rng.uniform(0.03, 0.09)
        b = spec.worldbody.add_body(name="prop%d" % i, pos=[float(c[0] + r * math.cos(a)), float(c[1] + r * math.sin(a)), float(s)])
        b.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[float(s), float(s * rng.uniform(0.5, 1.5)), float(s)],
                   rgba=[float(x) for x in list(rng.uniform(0.05, 0.95, 3)) + [1.0]])
    lights(spec, rng)


def servo(name, role, actuators, ticks=None):
    return {"name": name, "role": role, "kind": "servo", "actuators": actuators, "ticks": ticks}


def pd(name, role, actuators, kp, kd):
    return {"name": name, "role": role, "kind": "pd", "actuators": actuators, "kp": kp, "kd": kd, "ticks": None}


def kind_panda(rng):
    spec = mujoco.MjSpec.from_file(MENAGERIE + "/franka_emika_panda/scene.xml")
    center, size = [0.3, 0.0, 0.4], 0.8
    groups = [servo("arm1_joints", "ARM", ["actuator%d" % i for i in range(1, 8)]),
              servo("arm1_closer", "CLOSER", ["actuator8"])]
    cams = [body_camera(spec, "hand", "arm1_wrist", [0.09, 0.0, 0.0], [-0.45, 0.0, 1.0], [1.0, 0.0, 0.0], rng,
                        "ARM_CARRIED", "arm1_joints"),
            world_camera(spec, rng, center, size)]
    props(spec, rng, center, size, 8)
    return {"spec": spec, "groups": groups, "cameras": cams, "key": "home", "fix": None}


def kind_ur5e(rng):
    spec = mujoco.MjSpec.from_file(MENAGERIE + "/universal_robots_ur5e/scene.xml")
    center, size = [0.3, 0.0, 0.4], 0.8
    groups = [servo("arm1_joints", "ARM", ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"])]
    cams = [body_camera(spec, "wrist_3_link", "arm1_wrist", [0.05, 0.1, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], rng,
                        "ARM_CARRIED", "arm1_joints"),
            world_camera(spec, rng, center, size)]
    props(spec, rng, center, size, 8)
    return {"spec": spec, "groups": groups, "cameras": cams, "key": None, "fix": None,
            "init": {"shoulder_lift_joint": -1.57, "elbow_joint": 1.57, "wrist_1_joint": -1.57, "wrist_2_joint": -1.57}}


def kind_so101(rng):
    spec = mujoco.MjSpec.from_file(MENAGERIE + "/robotstudio_so101/scene.xml")
    center, size = [0.15, 0.0, 0.15], 0.4
    ticks = 4096   # a turn of the Feetech servo the SO-101 is built of, read and commanded as whole ticks
    groups = [servo("arm1_joints", "ARM", ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"], ticks),
              servo("arm1_closer", "CLOSER", ["gripper"], ticks)]
    cams = [{"name": "wrist_cam", "mount": "ARM_CARRIED", "by": "arm1_joints"},   # the maker's own wrist camera
            world_camera(spec, rng, center, size)]
    props(spec, rng, center, size, 8)
    return {"spec": spec, "groups": groups, "cameras": cams, "key": None, "fix": None}


def kind_aloha(rng):
    spec = mujoco.MjSpec.from_file(MENAGERIE + "/aloha/scene.xml")
    center, size = [0.0, 0.0, 0.2], 0.8
    groups = []
    for side, n in (("left", "arm1"), ("right", "arm2")):
        groups.append(servo(n + "_joints", "ARM", [side + "/" + j for j in
                                                   ("waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate")]))
        groups.append(servo(n + "_closer", "CLOSER", [side + "/gripper"]))
    cams = [{"name": "wrist_cam_left", "mount": "ARM_CARRIED", "by": "arm1_joints"},
            {"name": "wrist_cam_right", "mount": "ARM_CARRIED", "by": "arm2_joints"},
            {"name": "overhead_cam", "mount": "WORLD_FIXED", "by": None}]   # the maker's three cameras
    props(spec, rng, center, size, 6)
    return {"spec": spec, "groups": groups, "cameras": cams, "key": None, "fix": None}


def kind_stretch(rng):
    """Hello Robot's Stretch 3: a two-wheeled base at wheel velocities, a lift and a telescoping arm with a wrist of three
    joints, a gripper, a head that pans and tilts; the maker's cameras on the gripper (D405) and on the head (D435i)."""
    spec = mujoco.MjSpec.from_file(MENAGERIE + "/hello_robot_stretch_3/scene.xml")
    room(spec)
    center, size = [0.0, 0.0, 0.6], 1.2
    groups = [servo("base", "CARRIER", ["left_wheel_vel", "right_wheel_vel"]),
              servo("arm1_joints", "ARM", ["lift", "arm", "wrist_yaw", "wrist_pitch", "wrist_roll"]),
              servo("arm1_closer", "CLOSER", ["gripper"]),
              servo("head", "ARM", ["head_pan", "head_tilt"])]
    cams = [{"name": "d405_rgb", "mount": "ARM_CARRIED", "by": "arm1_joints"},
            {"name": "d435i_camera_rgb", "mount": "ARM_CARRIED", "by": "head"},
            world_camera(spec, rng, center, size)]
    props(spec, rng, center, size, 8)
    return {"spec": spec, "groups": groups, "cameras": cams, "key": None, "fix": None,
            "init": {"joint_head_tilt": -0.6, "joint_lift": 0.5}}


def kind_x2(rng):
    spec = mujoco.MjSpec.from_file(MENAGERIE + "/skydio_x2/scene.xml")
    center, size = [0.0, 0.0, 0.3], 1.5
    groups = [{"name": "motors", "role": "CARRIER", "kind": "flight", "actuators": ["thrust1", "thrust2", "thrust3", "thrust4"],
               "ticks": None}]
    cams = [body_camera(spec, "x2", "body_cam", [0.12, 0.0, 0.0], [1.0, 0.0, -0.35], [0.0, 0.0, 1.0], rng,
                        "CARRIER_CARRIED", "motors"),
            world_camera(spec, rng, center, size)]
    props(spec, rng, center, size, 10)
    return {"spec": spec, "groups": groups, "cameras": cams, "key": "hover", "fix": None}


def kind_g1(rng):
    spec = mujoco.MjSpec.from_file(MENAGERIE + "/unitree_g1/scene_with_hands.xml")
    center, size = [0.0, 0.0, 0.8], 1.4
    leg = ["hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll"]
    arm = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"]
    hand = ["thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1"]
    groups = [servo("leg_left", "PART", ["left_%s_joint" % j for j in leg]),
              servo("leg_right", "PART", ["right_%s_joint" % j for j in leg]),
              servo("waist", "ARM", ["waist_%s_joint" % j for j in ("yaw", "roll", "pitch")]),
              servo("arm1_joints", "ARM", ["left_%s_joint" % j for j in arm]),
              servo("arm1_closer", "CLOSER", ["left_hand_%s_joint" % j for j in hand]),
              servo("arm2_joints", "ARM", ["right_%s_joint" % j for j in arm]),
              servo("arm2_closer", "CLOSER", ["right_hand_%s_joint" % j for j in hand])]
    cams = [body_camera(spec, "torso_link", "head_cam", [0.06, 0.0, 0.42], [1.0, 0.0, -0.5], [0.0, 0.0, 1.0], rng,
                        "ARM_CARRIED", "waist"),
            body_camera(spec, "left_wrist_yaw_link", "arm1_wrist", [0.0, 0.04, 0.04], [1.0, -0.2, -0.3], [0.0, 0.0, 1.0],
                        rng, "ARM_CARRIED", "arm1_joints"),
            body_camera(spec, "right_wrist_yaw_link", "arm2_wrist", [0.0, -0.04, 0.04], [1.0, 0.2, -0.3], [0.0, 0.0, 1.0],
                        rng, "ARM_CARRIED", "arm2_joints"),
            world_camera(spec, rng, center, size)]
    props(spec, rng, [0.5, 0.0, 0.0], size, 10)
    # On a stand: the pelvis held where it stands (no free joint), as a humanoid is first booted in a lab.
    return {"spec": spec, "groups": groups, "cameras": cams, "key": None, "fix": "pelvis"}


def kind_rby1(rng):
    spec = mujoco.MjSpec.from_file(MENAGERIE + "/rainbow_robotics_rby1/scene_rby1m_1.2.xml")
    # Only its wheels touch the floor, as on the real robot: the base's own collision hull reaches below the wheels'
    # bottoms, and the floor pushing it up turned the robot over within two seconds (both bases, 10-23).
    base = find_body(spec, "base")
    for gm in base.geoms:
        gm.contype, gm.conaffinity = 0, 0
    center, size = [0.0, 0.0, 0.8], 1.4
    groups = [servo("base", "CARRIER", ["front_right_wheel_act", "front_left_wheel_act", "rear_right_wheel_act",
                                        "rear_left_wheel_act"]),
              servo("torso", "ARM", ["link%d_act" % i for i in range(1, 7)]),
              servo("arm1_joints", "ARM", ["left_arm_%d_act" % i for i in range(1, 8)]),
              servo("arm1_closer", "CLOSER", ["left_finger_act"]),
              servo("arm2_joints", "ARM", ["right_arm_%d_act" % i for i in range(1, 8)]),
              servo("arm2_closer", "CLOSER", ["right_finger_act"]),
              servo("head", "ARM", ["head_0_act", "head_1_act"])]
    cams = [body_camera(spec, "link_head_2", "head_cam", [0.08, 0.0, 0.05], [1.0, 0.0, -0.4], [0.0, 0.0, 1.0], rng,
                        "ARM_CARRIED", "head"),
            body_camera(spec, "link_left_arm_6", "arm1_wrist", [0.07, 0.0, 0.02], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0], rng,
                        "ARM_CARRIED", "arm1_joints"),
            body_camera(spec, "link_right_arm_6", "arm2_wrist", [0.07, 0.0, 0.02], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0], rng,
                        "ARM_CARRIED", "arm2_joints"),
            world_camera(spec, rng, center, size)]
    props(spec, rng, [0.6, 0.0, 0.0], size, 10)
    return {"spec": spec, "groups": groups, "cameras": cams, "key": None, "fix": None,
            "init": {"right_arm_3": -1.6, "left_arm_3": -1.6, "head_1": 0.5}}


def kind_shadow(rng):
    """A UR5e with a Shadow hand on its flange: the hand's twenty actuators (its wrist's two among them) one group, as
    its maker's interface gives them; the arm's camera behind the hand, the whole hand a part of its picture."""
    spec = mujoco.MjSpec.from_file(MENAGERIE + "/universal_robots_ur5e/scene.xml")
    hand = mujoco.MjSpec.from_file(MENAGERIE + "/shadow_hand/right_hand.xml")
    site = next(s for s in spec.sites if s.name == "attachment_site")
    spec.attach(hand, site=site, prefix="hand/")
    center, size = [0.3, 0.0, 0.4], 0.9
    groups = [servo("arm1_joints", "ARM", ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]),
              servo("arm1_closer", "CLOSER", ["hand/rh_A_" + a for a in
                                              ("WRJ2", "WRJ1", "THJ5", "THJ4", "THJ3", "THJ2", "THJ1", "FFJ4", "FFJ3",
                                               "FFJ0", "MFJ4", "MFJ3", "MFJ0", "RFJ4", "RFJ3", "RFJ0", "LFJ5", "LFJ4",
                                               "LFJ3", "LFJ0")])]
    cams = [body_camera(spec, "wrist_3_link", "arm1_wrist", [0.0, 0.06, 0.09], [0.0, 1.0, -0.7], [0.0, 0.0, 1.0], rng,
                        "ARM_CARRIED", "arm1_joints"),
            world_camera(spec, rng, center, size)]
    props(spec, rng, center, size, 8)
    spec.option.impratio = 10.0   # the hand's own solver settings (its model asks for them)
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    return {"spec": spec, "groups": groups, "cameras": cams, "key": None, "fix": None,
            "init": {"shoulder_lift_joint": -1.57, "elbow_joint": 1.57, "wrist_1_joint": -1.57, "wrist_2_joint": -1.57}}


def kind_go2(rng):
    spec = mujoco.MjSpec.from_file(MENAGERIE + "/unitree_go2/scene.xml")
    center, size = [0.0, 0.0, 0.3], 1.0
    # Stance gains that hold its weight (at 40 and 1 it sank 5 cm in two seconds and never stopped creeping; at 150 and 8
    # it settles within the minute, 10-23).
    groups = [pd("leg_" + l, "CARRIER", [l + "_hip", l + "_thigh", l + "_calf"], 150.0, 8.0) for l in ("FL", "FR", "RL", "RR")]
    cams = [body_camera(spec, "base", "head_cam", [0.4, 0.0, 0.06], [1.0, 0.0, -0.25], [0.0, 0.0, 1.0], rng,
                        "CARRIER_CARRIED", "leg_FL"),
            world_camera(spec, rng, center, size)]
    props(spec, rng, center, size, 10)
    return {"spec": spec, "groups": groups, "cameras": cams, "key": "home", "fix": None}


KINDS = {"panda": kind_panda, "ur5e": kind_ur5e, "so101": kind_so101, "aloha": kind_aloha, "stretch": kind_stretch,
         "x2": kind_x2, "go2": kind_go2, "g1": kind_g1, "rby1": kind_rby1,
         "shadow": kind_shadow}


# ---------------------------------------------------------------------------------------------------------------------
# What each robot's sensors really are (owner 10-23: a simulator's readings and pictures are flawless, a robot's are not;
# the flaws are copied from the hardware's papers, and where a paper gives no number, drawn per robot over the range of
# such products). Each camera is one of three classes, each group's channels read at an encoder's resolution.
#
# Cameras. The small sensors these robots carry publish no full well or read noise (their makers keep the full
# datasheets under NDA: OmniVision's OV2740 and OV9782 briefs, Sony's IMX219 excerpt); the range is that of small-pixel
# CMOS (1.1 to 3 um pixels, a few thousand to about thirteen thousand electrons at full scale, read noise of a few).
# Latency: no maker publishes one either; measured, a USB camera's frames come two to four frames late at 30 a second
# (Luxonis' glass-to-glass rig: 54 ms, peaks 60 to 70, for a 52 fps MJPEG stream; OBS: one or two frames of buffering),
# a RealSense's color at about 40 to 100 ms, and a camera on a video link (H.264) later still.
CAMERA_CLASSES = {
    # A USB camera streaming MJPEG (a webcam on a wrist, in a room).
    "webcam": {"full_well": (3000.0, 10000.0), "read_noise": (1.5, 4.0), "jpeg": (70, 90), "latency": (0.06, 0.13)},
    # A depth camera's color stream, raw over USB 3 (Intel RealSense D405, D435).
    "depth_rgb": {"full_well": (5000.0, 13000.0), "read_noise": (1.5, 4.0), "jpeg": None, "latency": (0.04, 0.10)},
    # A camera on a video link, its codec's loss stood for by JPEG's (a drone's, a quadruped's front camera).
    "stream": {"full_well": (3000.0, 10000.0), "read_noise": (1.5, 4.0), "jpeg": (55, 80), "latency": (0.10, 0.25)},
}

# Encoders: counts a turn as bits (a hinge's step is a turn over 2**bits), a slide's step in meters, a velocity read
# from counts over a beat; a jitter of a fraction of a count before the reading is rounded to its step. From the makers'
# papers: Feetech STS3215 (the SO-101) and Dynamixel X (ALOHA's arms and grippers, Stretch's wrist, gripper and head)
# 4096 counts a turn; Franka's arm 14 bits (a lab's equipment page). Unpublished (Unitree, Universal Robots, Rainbow,
# Hello Robot's lift and telescope, the wheels): drawn over the range of such encoders. Shadow's hand reads its joints by
# Hall effect sensors, known for a jitter of several counts.
JITTER = (0.2, 0.6)
SENSORS = {
    "panda": {"cameras": {"arm1_wrist": "depth_rgb", "room": "webcam"},
              "groups": {"arm1_joints": {"bits": (14, 14)}, "arm1_closer": {"step": 0.5}}},
    "ur5e": {"cameras": {"arm1_wrist": "webcam", "room": "webcam"},
             "groups": {"arm1_joints": {"bits": (16, 20)}}},
    "so101": {"cameras": {"wrist_cam": "webcam", "room": "webcam"},
              "groups": {"arm1_joints": {"bits": (12, 12)}, "arm1_closer": {"bits": (12, 12)}}},
    "aloha": {"cameras": {"wrist_cam_left": "webcam", "wrist_cam_right": "webcam", "overhead_cam": "webcam"},
              "groups": {"arm1_joints": {"bits": (12, 12)}, "arm2_joints": {"bits": (12, 12)},
                         "arm1_closer": {"step": 3.0e-5}, "arm2_closer": {"step": 3.0e-5}}},
    "stretch": {"cameras": {"d405_rgb": "depth_rgb", "d435i_camera_rgb": "depth_rgb", "room": "webcam"},
                "groups": {"base": {"bits": (12, 16), "velocity": True},
                           "arm1_joints": {"bits": (12, 12), "slide_step": (1.0e-5, 1.0e-4)},
                           "arm1_closer": {"step": 1.0e-5}, "head": {"bits": (12, 12)}}},
    "x2": {"cameras": {"body_cam": "stream", "room": "webcam"},
           "groups": {"motors": {"estimate": (0.01, 0.05)}}},
    "go2": {"cameras": {"head_cam": "stream", "room": "webcam"},
            "groups": {"leg_FL": {"bits": (14, 16)}, "leg_FR": {"bits": (14, 16)}, "leg_RL": {"bits": (14, 16)},
                       "leg_RR": {"bits": (14, 16)}}},
    "g1": {"cameras": {"head_cam": "depth_rgb", "arm1_wrist": "webcam", "arm2_wrist": "webcam", "room": "webcam"},
           "groups": {"leg_left": {"bits": (14, 18)}, "leg_right": {"bits": (14, 18)}, "waist": {"bits": (14, 18)},
                      "arm1_joints": {"bits": (14, 18)}, "arm2_joints": {"bits": (14, 18)},
                      "arm1_closer": {"bits": (12, 14)}, "arm2_closer": {"bits": (12, 14)}}},
    "rby1": {"cameras": {"head_cam": "depth_rgb", "arm1_wrist": "webcam", "arm2_wrist": "webcam", "room": "webcam"},
             "groups": {"base": {"bits": (12, 16), "velocity": True}, "torso": {"bits": (14, 20)},
                        "arm1_joints": {"bits": (14, 20)}, "arm2_joints": {"bits": (14, 20)},
                        "arm1_closer": {"step": 1.0e-5}, "arm2_closer": {"step": 1.0e-5}, "head": {"bits": (14, 20)}}},
    "shadow": {"cameras": {"arm1_wrist": "webcam", "room": "webcam"},
               "groups": {"arm1_joints": {"bits": (16, 20)}, "arm1_closer": {"bits": (12, 12), "jitter": (1.0, 3.0)}}},
}


def draw_sensors(kind, built, rng):
    """Each camera's and each group's sensor drawn for this robot from its class and paper (SENSORS), written on the
    camera and the group: a camera's full well, read noise, JPEG quality and latency; a group's bits, slide step,
    fixed step (a gripper read in its command's own units), velocity, estimate noise and jitter."""
    table = SENSORS[kind]
    for c in built["cameras"]:
        cls = CAMERA_CLASSES[table["cameras"][c["name"]]]
        c["sensor"] = {"class": table["cameras"][c["name"]],
                       "full_well": float(rng.uniform(*cls["full_well"])), "read_noise": float(rng.uniform(*cls["read_noise"])),
                       "jpeg": int(rng.integers(cls["jpeg"][0], cls["jpeg"][1] + 1)) if cls["jpeg"] else None,
                       "latency": float(rng.uniform(*cls["latency"]))}
    for g in built["groups"]:
        spec = table["groups"][g["name"]]
        s = {"jitter": float(rng.uniform(*spec.get("jitter", JITTER)))}
        if "bits" in spec:
            s["bits"] = int(rng.integers(spec["bits"][0], spec["bits"][1] + 1))
        if "slide_step" in spec:
            s["slide_step"] = float(math.exp(rng.uniform(math.log(spec["slide_step"][0]), math.log(spec["slide_step"][1]))))
        if "step" in spec:
            s["step"] = float(spec["step"])
        if spec.get("velocity"):
            s["velocity"] = True
        if "estimate" in spec:
            s["estimate"] = float(rng.uniform(*spec["estimate"]))
            # A flight controller's estimate (a filter over its IMU and its camera or GPS) errs by much the same over a
            # fraction of a second, not afresh every beat.
            s["estimate_time"] = float(rng.uniform(0.2, 1.0))
        g["sensor"] = s
