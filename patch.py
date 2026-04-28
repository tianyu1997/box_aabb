import sys
from pathlib import Path

content = Path("cpp/v7/experiments/scripts/common.py").read_text()

content = content.replace('BUILD_RELEASE = ROOT / "build-release"\nBUILD_DEBUG = ROOT / "build"', 'BUILD_V6 = ROOT.parent / "v6" / "build"\nBUILD_RELEASE = ROOT / "build-release"\nBUILD_DEBUG = ROOT / "build"')

content = content.replace("""    if requested_build_dir is not None:
        candidate_dirs.append(_normalize_build_dir(requested_build_dir))
    else:
        if BUILD_RELEASE.is_dir():
            candidate_dirs.append(BUILD_RELEASE)
        if BUILD_DEBUG.is_dir():
            candidate_dirs.append(BUILD_DEBUG)""", """    if requested_build_dir is not None:
        candidate_dirs.append(_normalize_build_dir(requested_build_dir))
    else:
        # Enforce usage of v6/build tree to avoid mixing build products
        if BUILD_V6.is_dir():
            candidate_dirs.append(BUILD_V6)
        else:
            raise FileNotFoundError(f"Authoritative v6 build directory not found at {BUILD_V6}")""")

Path("cpp/v7/experiments/scripts/common.py").write_text(content)
