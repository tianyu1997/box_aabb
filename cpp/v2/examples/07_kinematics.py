#!/usr/bin/env python3
"""
Example 7: Robot Kinematics — FK & Interval FK
=================================================

Demonstrates:
  - Loading a robot and inspecting DH parameters
  - Computing forward kinematics (scalar)
  - Interval forward kinematics
  - Envelope computation
  - Interval arithmetic utilities

Usage:
    python 07_kinematics.py
"""
import numpy as np

try:
    import pysbf2
except ImportError:
    raise ImportError("pysbf2 not found. Build with -DSBF_WITH_PYTHON=ON.")


def main():
    # ─── 1. Load Robot & Inspect ──────────────────────────────────────────
    print("=== Robot Model ===")
    robot = pysbf2.Robot.from_json("../../configs/panda.json")
    print(f"Name:        {robot.name()}")
    print(f"Joints:      {robot.n_joints()}")
    print(f"Links:       {robot.n_links()}")
    print(f"Transforms:  {robot.n_transforms()}")
    print(f"Has tool:    {robot.has_tool()}")
    print(f"EE spheres:  {robot.n_ee_spheres()}")
    print(f"Fingerprint: {robot.fingerprint()[:32]}...")

    # DH parameters
    print(f"\nDH Parameters:")
    print(f"  {'Joint':>5}  {'alpha':>8}  {'a':>8}  {'d':>8}  "
          f"{'theta':>8}  {'type':>4}")
    for i, dh in enumerate(robot.dh_params()):
        jtype = "R" if dh.joint_type == 0 else "P"
        print(f"  {i:5d}  {dh.alpha:8.4f}  {dh.a:8.4f}  {dh.d:8.4f}  "
              f"{dh.theta:8.4f}  {jtype:>4}")

    # Joint limits
    print(f"\nJoint Limits:")
    for i, iv in enumerate(robot.joint_limits().limits):
        print(f"  Joint {i}: [{iv.lo:+.4f}, {iv.hi:+.4f}] "
              f"(range={iv.width():.4f})")

    # ─── 2. Scalar Forward Kinematics ────────────────────────────────────
    print("\n=== Scalar FK ===")
    q_home = np.zeros(robot.n_joints())
    q_test = np.array([0.5, -0.3, 0.2, -1.5, 0.1, 1.2, -0.3])

    for name, q in [("home", q_home), ("test", q_test)]:
        positions = robot.fk_link_positions(q)
        print(f"\nConfig '{name}':")
        for j in range(positions.shape[0]):
            p = positions[j, :]
            print(f"  Link {j}: [{p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f}]")

    # Full transforms
    transforms = robot.fk_transforms(q_test)
    print(f"\nFull transforms at 'test': {len(transforms)} matrices")
    # Show end-effector transform
    T_ee = transforms[-1]
    print(f"End-effector pose:")
    print(f"  Position: [{T_ee[0,3]:.4f}, {T_ee[1,3]:.4f}, {T_ee[2,3]:.4f}]")
    print(f"  Rotation (col 0): [{T_ee[0,0]:.4f}, {T_ee[1,0]:.4f}, {T_ee[2,0]:.4f}]")

    # ─── 3. Free FK function ─────────────────────────────────────────────
    print("\n=== Free FK Functions ===")
    positions2 = pysbf2.fk_link_positions(robot, q_test)
    transforms2 = pysbf2.fk_transforms(robot, q_test)
    print(f"fk_link_positions: {len(positions2)} positions")
    print(f"fk_transforms:     {len(transforms2)} matrices")

    # Single DH transform
    dh0 = robot.dh_params()[0]
    T_single = pysbf2.dh_transform(dh0.alpha, dh0.a, dh0.d, dh0.theta)
    print(f"\nSingle DH transform (joint 0):")
    for row in range(4):
        print(f"  [{T_single[row,0]:+.4f} {T_single[row,1]:+.4f} "
              f"{T_single[row,2]:+.4f} {T_single[row,3]:+.4f}]")

    # ─── 4. Interval Arithmetic ──────────────────────────────────────────
    print("\n=== Interval Arithmetic ===")

    # Basic operations
    a = pysbf2.Interval(-1.0, 1.0)
    b = pysbf2.Interval(0.5, 2.0)
    print(f"a = {a}")
    print(f"b = {b}")
    print(f"a + b = {a + b}")
    print(f"a - b = {a - b}")
    print(f"a * b = {a * b}")
    print(f"a * 3.0 = {a * 3.0}")
    print(f"hull(a, b) = {a.hull(b)}")
    print(f"intersect(a, b) = {a.intersect(b)}")

    # Interval trig
    sin_iv = pysbf2.I_sin(-0.5, 0.5)
    cos_iv = pysbf2.I_cos(-0.5, 0.5)
    print(f"\nI_sin([-0.5, 0.5]) = {sin_iv}")
    print(f"I_cos([-0.5, 0.5]) = {cos_iv}")

    # Verify enclosure
    n_samples = 10000
    thetas = np.linspace(-0.5, 0.5, n_samples)
    actual_sin = np.sin(thetas)
    actual_cos = np.cos(thetas)
    print(f"  sin range: [{actual_sin.min():.6f}, {actual_sin.max():.6f}]")
    print(f"  cos range: [{actual_cos.min():.6f}, {actual_cos.max():.6f}]")
    assert sin_iv.lo <= actual_sin.min() and sin_iv.hi >= actual_sin.max(), \
        "I_sin does not enclose actual range!"
    assert cos_iv.lo <= actual_cos.min() and cos_iv.hi >= actual_cos.max(), \
        "I_cos does not enclose actual range!"
    print("  Enclosure verified!")

    # ─── 5. Interval FK ──────────────────────────────────────────────────
    print("\n=== Interval FK ===")

    # Create intervals around home config
    delta = 0.1
    intervals = [pysbf2.Interval(q_home[i] - delta, q_home[i] + delta)
                 for i in range(robot.n_joints())]
    print(f"Box size: ±{delta} around home")

    fk_state = pysbf2.compute_fk_full(robot, intervals)
    print(f"FKState: valid={fk_state.valid}, "
          f"n_tf={fk_state.n_tf}, n_jm={fk_state.n_jm}")

    # Incremental update: narrow joint 3
    intervals2 = list(intervals)
    intervals2[3] = pysbf2.Interval(-0.05, 0.05)  # narrower
    fk_inc = pysbf2.compute_fk_incremental(fk_state, robot, intervals2, 3)
    print(f"Incremental (split dim 3): valid={fk_inc.valid}")

    # ─── 6. Envelope Computer (LEGACY) ───────────────────────────────────
    # NOTE: IntervalFKEnvelopeComputer is a legacy API retained for backward
    # compatibility.  New C++ code should use FrameStore + collision_policy.h.
    print("\n=== Envelope Computer (Legacy API) ===")
    env_comp = pysbf2.IntervalFKEnvelopeComputer(robot)
    print(f"Total AABB slots: {env_comp.n_total_aabb_slots()}")

    # Compute envelope
    env = env_comp.compute_envelope(intervals)
    print(f"Envelope: valid={env.valid}, "
          f"link_slots={env.n_link_slots}, "
          f"ee_slots={env.n_ee_slots}")

    # Incremental
    env2 = env_comp.compute_envelope_incremental(
        env.fk_state, intervals2, changed_dim=3)
    print(f"Incremental envelope: valid={env2.valid}")

    # ─── 7. Config Containment ────────────────────────────────────────────
    print("\n=== Configuration Queries ===")
    limits = robot.joint_limits()
    print(f"Home in limits: {limits.contains(q_home)}")
    print(f"Test in limits: {limits.contains(q_test)}")

    # Out of bounds
    q_bad = np.ones(robot.n_joints()) * 10.0
    print(f"Out-of-range in limits: {limits.contains(q_bad)}")
    q_clamped = limits.clamp(q_bad)
    print(f"Clamped: {limits.contains(q_clamped)}")
    print(f"Clamped values: {q_clamped}")


if __name__ == "__main__":
    main()
