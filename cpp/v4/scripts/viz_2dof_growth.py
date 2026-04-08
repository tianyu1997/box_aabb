from pathlib import Path
import sys


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))
    from viz.growth_viz import run_cli

    run_cli(
        run_name="viz_2dof_growth",
        description="Render 2DOF dual-panel growth and execution GIFs with the v4 grower.",
    )


if __name__ == "__main__":
    main()