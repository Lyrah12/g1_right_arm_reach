#!/usr/bin/env python3
"""
finger_tremor_harness.py — reproduce thumb_yaw idle drift + finger trembling WITHOUT ROS.

Mimics the real stack on the deployed scene:
  - 200 Hz physics (bridge physics_rate)
  - Finger impedance control at 50 Hz (hand_controller stepFingerControl):
        tau = clamp(kp*(des - q) - kd*dq, -tau_max, +tau_max), des = home pose
    Applied to the 6 active finger motors. Between 50 Hz updates the ctrl is
    held constant (zero-order hold) — exactly what the bridge does.
  - Right arm + wrist motors: gravity-hold (qfrc_bias at zero vel), refreshed at 20 Hz
    (bridge _refresh_gravity_hold behavior).
  - Position servos (legs/waist/left arm): ctrl = init qpos (hold).

Usage:
  python3 finger_tremor_harness.py [--dur 60] [--kp 5.0] [--kd 0.3] [--armature 0.0005]
                                   [--rate 50] [--out /tmp/finger_tremor.npz]
  --armature overrides the hand-joint armature in the model (A/B fix test).
  --rate overrides the finger control rate in Hz.
"""

import argparse
import os
import sys

import numpy as np
import mujoco

SCENE = "/home/lyrah/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml"

FINGER_NAMES = [
    "R_thumb_proximal_yaw_joint",
    "R_thumb_proximal_pitch_joint",
    "R_index_proximal_joint",
    "R_middle_proximal_joint",
    "R_ring_proximal_joint",
    "R_pinky_proximal_joint",
]
DES = [0.25, 0.25, 0.8, 0.8, 0.8, 0.8]

HAND_ALL_NAMES = [
    "R_thumb_proximal_yaw_joint", "R_thumb_proximal_pitch_joint",
    "R_thumb_intermediate_joint", "R_thumb_distal_joint",
    "R_index_proximal_joint", "R_index_intermediate_joint",
    "R_middle_proximal_joint", "R_middle_intermediate_joint",
    "R_ring_proximal_joint", "R_ring_intermediate_joint",
    "R_pinky_proximal_joint", "R_pinky_intermediate_joint",
]

ARM_WRIST_NAMES = [
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dur", type=float, default=60.0)
    ap.add_argument("--kp", type=float, default=5.0)
    ap.add_argument("--kd", type=float, default=0.3)
    ap.add_argument("--tau_max", type=float, default=1.5)
    ap.add_argument("--armature", type=float, default=0.0,
                    help="override hand-joint armature; 0 = use native model value")
    ap.add_argument("--rate", type=float, default=50.0)   # finger control rate (Hz)
    ap.add_argument("--scene", default=SCENE)
    ap.add_argument("--out", default="/tmp/finger_tremor.npz")
    ap.add_argument("--physics_rate", type=float, default=200.0)  # bridge physics_rate
    args = ap.parse_args()

    if not os.path.exists(args.scene):
        print(f"[FATAL] scene not found: {args.scene}", file=sys.stderr)
        sys.exit(1)

    model = mujoco.MjModel.from_xml_path(args.scene)
    data = mujoco.MjData(model)

    # Bridge overrides the scene timestep with 1/physics_rate (default 200 Hz).
    model.opt.timestep = 1.0 / args.physics_rate
    physics_rate = 1.0 / model.opt.timestep
    control_interval = max(1, int(round(physics_rate / args.rate)))
    print(f"model: nq={model.nq} nv={model.nv} nu={model.nu} "
          f"timestep={model.opt.timestep:.4f} ({physics_rate:.0f} Hz)")

    # Native hand-joint armature (diagnostic — confirms the deployed model value)
    native_arm = {}
    for jn in HAND_ALL_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid >= 0:
            native_arm[jn] = float(model.dof_armature[model.jnt_dofadr[jid]])
    uniq = sorted(set(native_arm.values()))
    print(f"native hand armature: {uniq}")

    # ------------------------------------------------------------------
    # Optional armature override (A/B fix test); 0 = use native model value
    # ------------------------------------------------------------------
    if args.armature > 0:
        n_replaced = 0
        for jn in HAND_ALL_NAMES:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid >= 0:
                model.dof_armature[model.jnt_dofadr[jid]] = args.armature
                n_replaced += 1
        print(f"armature override: {n_replaced} hand joints -> {args.armature}")

    # ------------------------------------------------------------------
    # Name -> qpos/dof/actuator maps
    # ------------------------------------------------------------------
    def qpos_addr(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return model.jnt_qposadr[jid], model.jnt_dofadr[jid]

    finger_q = [qpos_addr(n) for n in FINGER_NAMES]          # [(qpos, dof)]
    arm_wrist_dof = [qpos_addr(n)[1] for n in ARM_WRIST_NAMES]

    # actuator -> joint name, and joint name -> list of motor actuator ids
    act_joint = {}
    for a in range(model.nu):
        jid = model.actuator_trnid[a][0]
        jn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid) if jid >= 0 else None
        act_joint[a] = jn
    joint_acts = {}
    for a, jn in act_joint.items():
        joint_acts.setdefault(jn, []).append(a)
    print(f"nactuators={model.nu}")

    motor_acts = [a for a in range(model.nu)
                  if model.actuator_biastype[a] == 0]       # pure <motor>
    pos_acts = [a for a in range(model.nu)
                if model.actuator_biastype[a] != 0]         # <position> servo
    print(f"motors={len(motor_acts)} position-servos={len(pos_acts)}")

    # Base joint type
    base_free = False
    for jid in range(model.njnt):
        jn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        if jn and "floating" in jn and model.jnt_type[jid] == 0:  # mjJNT_FREE
            base_free = True
    print(f"base joint free={base_free}")

    # ------------------------------------------------------------------
    # Home hand pose (bridge hand_ready_pose)
    # ------------------------------------------------------------------
    home = {
        "R_thumb_proximal_yaw_joint": 0.25,
        "R_thumb_proximal_pitch_joint": 0.25,
        "R_thumb_intermediate_joint": 0.40,
        "R_thumb_distal_joint": 0.60,
        "R_index_proximal_joint": 0.8, "R_index_intermediate_joint": 0.8,
        "R_middle_proximal_joint": 0.8, "R_middle_intermediate_joint": 0.8,
        "R_ring_proximal_joint": 0.8, "R_ring_intermediate_joint": 0.8,
        "R_pinky_proximal_joint": 0.8, "R_pinky_intermediate_joint": 0.8,
    }
    for n, v in home.items():
        qa, _ = qpos_addr(n)
        data.qpos[qa] = v
    mujoco.mj_forward(model, data)

    # ------------------------------------------------------------------
    # Initialize actuators like the bridge: position servos -> init qpos,
    # motors -> 0, then gravity-hold for arm/wrist.
    # ------------------------------------------------------------------
    init_pos = {}
    for jn, acts in joint_acts.items():
        qa, da = qpos_addr(jn)
        init_pos[jn] = float(data.qpos[qa])
        for a in acts:
            if a in motor_acts:
                data.ctrl[a] = 0.0
            else:
                data.ctrl[a] = float(data.qpos[qa])

    def refresh_gravity_hold():
        """gravity-hold torque for arm/wrist/finger motor joints (zero-vel qfrc_bias)."""
        qvel_save = data.qvel.copy()
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        hold = {}
        for jn, da in zip(ARM_WRIST_NAMES, arm_wrist_dof):
            hold[jn] = float(data.qfrc_bias[da])
        data.qvel[:] = qvel_save
        return hold

    grav = refresh_gravity_hold()

    # finger motor actuator ids (by name)
    finger_motor_acts = []
    for n in FINGER_NAMES:
        acts = joint_acts.get(n, [])
        m = [a for a in acts if a in motor_acts]
        finger_motor_acts.append(m[0] if m else -1)

    arm_wrist_motor_acts = []
    for n in ARM_WRIST_NAMES:
        acts = joint_acts.get(n, [])
        m = [a for a in acts if a in motor_acts]
        arm_wrist_motor_acts.append(m[0] if m else -1)

    finger_ctrl = np.zeros(6)

    # ------------------------------------------------------------------
    # Run
    # ------------------------------------------------------------------
    n_steps = int(args.dur * physics_rate)
    grav_refresh_interval = max(1, int(physics_rate / 20.0))
    dt = model.opt.timestep

    # sampling: log every control interval
    rec = {n: [] for n in HAND_ALL_NAMES}
    rec_dq = {n: [] for n in HAND_ALL_NAMES}
    rec_tau = {n: [] for n in FINGER_NAMES}
    rec_apply = {n: [] for n in FINGER_NAMES}
    t_rec = []

    for s in range(n_steps):
        if s % control_interval == 0:
            # ---- 50 Hz finger impedance control ----
            for i, (qa, da) in enumerate(finger_q):
                q = float(data.qpos[qa])
                dq = float(data.qvel[da])
                e = DES[i] - q
                tau = args.kp * e + args.kd * (-dq)
                tau = max(-args.tau_max, min(args.tau_max, tau))
                finger_ctrl[i] = tau
        if s % grav_refresh_interval == 0:
            grav = refresh_gravity_hold()
        # ---- apply ctrl ----
        for i, a in enumerate(finger_motor_acts):
            if a >= 0:
                data.ctrl[a] = finger_ctrl[i]
        for i, a in enumerate(arm_wrist_motor_acts):
            if a >= 0 and ARM_WRIST_NAMES[i] in grav:
                data.ctrl[a] = grav[ARM_WRIST_NAMES[i]]
        mujoco.mj_step(model, data)
        if s % control_interval == 0:
            for n in HAND_ALL_NAMES:
                qa, da = qpos_addr(n)
                rec[n].append(float(data.qpos[qa]))
                rec_dq[n].append(float(data.qvel[da]))
            for i, n in enumerate(FINGER_NAMES):
                rec_tau[n].append(finger_ctrl[i])
                a = finger_motor_acts[i]
                rec_apply[n].append(float(data.actuator_force[a]) if a >= 0 else 0.0)
            t_rec.append(s * dt)

    # ------------------------------------------------------------------
    # Report
    # ------------------------------------------------------------------
    t = np.array(t_rec)
    np.savez(args.out,
             t=t,
             **{f"q_{n}": np.array(rec[n]) for n in HAND_ALL_NAMES},
             **{f"dq_{n}": np.array(rec_dq[n]) for n in HAND_ALL_NAMES},
             **{f"tau_{n}": np.array(rec_tau[n]) for n in FINGER_NAMES},
             **{f"apply_{n}": np.array(rec_apply[n]) for n in FINGER_NAMES})
    print(f"saved -> {args.out}")

    def trend(q):
        """linear trend (rad/s) of a logged position series."""
        return float(np.polyfit(t, q, 1)[0])

    def tremble(q):
        """trembling amplitude: std of position after removing linear trend (rad)."""
        q = np.asarray(q, dtype=float)
        qfit = np.polyval(np.polyfit(t, q, 1), t)
        return float(np.std(q - qfit))

    print(f"\n=== {args.dur}s, kp={args.kp}, kd={args.kd}, "
          f"armature={args.armature}, rate={args.rate} Hz ===")
    print(f"{'joint':34s} {'start':>8s} {'end':>8s} {'trend rad/s':>12s} "
          f"{'tremble rad':>11s} {'tau_end':>8s}")
    for i, n in enumerate(FINGER_NAMES):
        q = rec[n]
        print(f"{n:34s} {q[0]:8.3f} {q[-1]:8.3f} {trend(q):12.4f} "
              f"{tremble(q):11.4f} {rec_tau[n][-1]:8.3f}")
    # slave joints (coupled) — check tracking
    for n in ["R_thumb_intermediate_joint", "R_thumb_distal_joint"]:
        q = rec[n]
        print(f"{n:34s} {q[0]:8.3f} {q[-1]:8.3f} {trend(q):12.4f} {tremble(q):11.4f}")

    # thumb_yaw detail: report max excursion & whether it hit the limit
    ty = np.array(rec["R_thumb_proximal_yaw_joint"])
    print(f"\nthumb_yaw: min={ty.min():.3f} max={ty.max():.3f} "
          f"final={ty[-1]:.3f} | hit limit 1.3: {'YES' if ty.max() > 1.25 else 'no'}")


if __name__ == "__main__":
    main()
