# describe.py MODEL...: simulator-side rig code. For each Menagerie model: its actuators (name, what they drive, their
# control range, position servo or not), its bodies as a tree, and the cameras it already has.
import sys
import mujoco
for path in sys.argv[1:]:
    m = mujoco.MjModel.from_xml_path(path)
    print("=== %s: %d bodies, %d joints, %d actuators, %d cameras, timestep %.4f" % (path.split("/")[-2], m.nbody, m.njnt, m.nu, m.ncam, m.opt.timestep))
    for i in range(m.nu):
        a = m.actuator(i)
        tt = m.actuator_trntype[i]; tid = m.actuator_trnid[i, 0]
        tgt = (m.joint(tid).name if tt == mujoco.mjtTrn.mjTRN_JOINT else m.tendon(tid).name if tt == mujoco.mjtTrn.mjTRN_TENDON
               else m.site(tid).name if tt == mujoco.mjtTrn.mjTRN_SITE else m.body(tid).name if tt == mujoco.mjtTrn.mjTRN_BODY else "?")
        kind = "position" if m.actuator_biastype[i] == mujoco.mjtBias.mjBIAS_AFFINE and m.actuator_biasprm[i, 1] != 0 else "force/velocity"
        print("  act %-28s -> %-6s %-26s ctrl %s %s" % (a.name, str(mujoco.mjtTrn(tt)).split("_")[-1], tgt, m.actuator_ctrlrange[i].round(3).tolist(), kind))
    for b in range(1, m.nbody):
        j = [m.joint(k).name + ":" + str(mujoco.mjtJoint(m.jnt_type[k])).split("_")[-1] for k in range(m.njnt) if m.jnt_bodyid[k] == b]
        print("  body %-28s parent %-24s joints %s" % (m.body(b).name, m.body(m.body_parentid[b]).name, " ".join(j)))
    for c in range(m.ncam):
        print("  cam %s on %s" % (m.camera(c).name, m.body(m.cam_bodyid[c]).name))
