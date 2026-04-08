from pathlib import Path
import sys


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))
    from viz.growth_viz import run_cli

    run_cli(
        run_name="exp_2dof_growth_viz",
        description="Run the v4 2DOF growth visualization experiment using LectAligned + BEST_TIGHTEN.",
    )


if __name__ == "__main__":
    main()