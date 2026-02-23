"""Diagnose path quality: compare raw solver output vs post-repair."""
import sys, numpy as np, copy
sys.path.insert(0, 'src')
sys.path.insert(0, '.')
from aabb.robot import load_robot
from planner.pipeline import (
    set_verbose, _greedy_shortcut, run_method_with_bridge,
    _solve_method_dijkstra, grow_and_prepare, PandaGCSConfig
)
from scripts.exp2_method_comparison import (
    build_panda_5obs_scene, _path_length, _compute_period,
    SBF_CFG_PANDA, SBF_CFG_2DOF, build_random_2d_scene
)
from forest.collision import CollisionChecker
set_verbose(False)

seed = 7

def test_env(robot, scene, q_start, q_goal, sbf_cfg, label):
    period = _compute_period(robot.joint_limits)
    print(f"\n{'='*60}")
    print(f"{label}  (period={period})")
    print(f"{'='*60}")

    cfg = PandaGCSConfig()
    for k, v in sbf_cfg.items():
        if hasattr(cfg, k):
            setattr(cfg, k, v)
    cfg.seed = seed

    # Build forest
    prep = grow_and_prepare(robot, copy.deepcopy(scene), cfg, q_start, q_goal, robot.n_joints)

    # Get raw solver output
    raw = run_method_with_bridge(
        _solve_method_dijkstra, "Dijkstra",
        prep, cfg, q_start, q_goal, robot.n_joints, seed=seed)
    if not raw or not raw.get('success'):
        print("  Solver FAILED")
        return

    wps_raw = raw['waypoints']
    path_raw = np.array(wps_raw, dtype=np.float64)
    pl_raw_geo = _path_length(path_raw, period)
    pl_raw_euc = _path_length(path_raw, None)
    print(f"  Raw solver:     geo={pl_raw_geo:.4f}  euc={pl_raw_euc:.4f}  nwp={len(wps_raw)}")
    print(f"  Raw solver cost (from pipeline) = {raw['cost']:.4f}")

    checker = CollisionChecker(robot=robot, scene=scene)

    # Test greedy shortcut with period (geodesic)
    wps_list = [path_raw[i].copy() for i in range(len(path_raw))]
    sc_geo = _greedy_shortcut(wps_list, checker, 0.03, period=period)
    pl_sc_geo_geo = _path_length(np.array(sc_geo), period)
    pl_sc_geo_euc = _path_length(np.array(sc_geo), None)
    print(f"  Shortcut(geo):  geo={pl_sc_geo_geo:.4f}  euc={pl_sc_geo_euc:.4f}  nwp={len(sc_geo)}")

    # Test greedy shortcut without period (euclidean)
    wps_list2 = [path_raw[i].copy() for i in range(len(path_raw))]
    sc_euc = _greedy_shortcut(wps_list2, checker, 0.05)
    pl_sc_euc_geo = _path_length(np.array(sc_euc), period)
    pl_sc_euc_euc = _path_length(np.array(sc_euc), None)
    print(f"  Shortcut(euc):  geo={pl_sc_euc_geo:.4f}  euc={pl_sc_euc_euc:.4f}  nwp={len(sc_euc)}")

    # What does _repair_path_geodesic/euclidean actually produce?
    from baselines import SBFAdapter
    ada = SBFAdapter(method='dijkstra')
    ada_cfg = dict(sbf_cfg)
    ada_cfg['seed'] = seed
    ada.setup(robot, copy.deepcopy(scene), ada_cfg)
    # Manually set prep to reuse same forest
    ada._prep = prep
    ada._checker = checker

    if period is not None:
        repaired, rep_cost = ada._repair_path_geodesic(path_raw.copy())
    else:
        repaired, rep_cost = ada._repair_path_euclidean(path_raw.copy())

    pl_rep_geo = _path_length(repaired, period)
    pl_rep_euc = _path_length(repaired, None)
    print(f"  Repair output:  geo={pl_rep_geo:.4f}  euc={pl_rep_euc:.4f}  nwp={len(repaired)}  cost={rep_cost:.4f}")

    # Check: is repair improving or degrading?
    metric = pl_rep_geo if period else pl_rep_euc
    raw_metric = pl_raw_geo if period else pl_raw_euc
    delta = metric - raw_metric
    print(f"  Delta (repair - raw): {delta:+.4f}  {'WORSE' if delta > 0.01 else 'BETTER' if delta < -0.01 else 'SAME'}")


# --- 2DOF ---
robot2 = load_robot('2dof_planar')
qs2 = np.array([2.5, -2.5])
qg2 = np.array([-2.5, 2.5])
rng2 = np.random.default_rng(seed)
scene2 = build_random_2d_scene(robot2, qs2, qg2, rng2)
test_env(robot2, scene2, qs2, qg2, SBF_CFG_2DOF, "2DOF")

# --- Panda ---
robot_p = load_robot('panda')
qs_p = np.array([0.5, -1.2, 0.5, -2.5, 0.5, 0.8, 1.5])
qg_p = np.array([-2.0, 1.2, -1.8, -0.5, -2.0, 3.0, -1.8])
scene_p = build_panda_5obs_scene(robot_p, qs_p, qg_p, scene_seed=1000+seed)
test_env(robot_p, scene_p, qs_p, qg_p, SBF_CFG_PANDA, "Panda 7DOF")
