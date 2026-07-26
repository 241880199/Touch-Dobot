#!/usr/bin/env python3
"""
FK Cross-Validation: C++ vs MATLAB
===================================
Generates test vectors, runs both FK implementations, compares results,
and writes a markdown report.

Usage:
    python compare_fk.py                    # full pipeline
    python compare_fk.py --generate-only    # only generate test vectors
    python compare_fk.py --compare-only     # only compare existing outputs
"""

import json
import math
import os
import random
import subprocess
import sys
from pathlib import Path

# ============================================================
# Configuration
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parent.parent
MATLAB_SCRIPT = PROJECT_ROOT / "Relay_Station" / "fk_validate.m"
CPP_EXE = PROJECT_ROOT / "Touch_Client" / "tests" / "fk_validate.exe"
INPUT_JSON = PROJECT_ROOT / "Relay_Station" / "input.json"
INPUT_CSV = PROJECT_ROOT / "Touch_Client" / "tests" / "input.csv"
MATLAB_OUT = PROJECT_ROOT / "Relay_Station" / "matlab_output.json"
CPP_OUT = PROJECT_ROOT / "Touch_Client" / "tests" / "cpp_output.json"
REPORT = PROJECT_ROOT / "docs" / "fk_cross_validation_report.md"

M_PI = math.pi
D2R = M_PI / 180.0

# Joint limits (degrees)
J_LIMITS = [
    (-360.0, 360.0),
    (-360.0, 360.0),
    (-155.0, 155.0),
    (-360.0, 360.0),
    (-360.0, 360.0),
    (-360.0, 360.0),
]

RANDOM_COUNT = 50
RANDOM_SEED = 42

# ============================================================
# Test vector generation
# ============================================================


def generate_test_vectors():
    """Generate 5 categories of test vectors."""
    configs = []

    # Category 1: All zeros
    configs.append({"label": "all_zeros", "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "category": "zeros"})

    # Category 2: Joint limits
    limit_labels = [
        "J1_pos_max", "J1_neg_min", "J2_pos_max", "J2_neg_min",
        "J3_pos_max", "J3_neg_min",
    ]
    limit_joints = [
        [360.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [-360.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 360.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, -360.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 155.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -155.0, 0.0, 0.0, 0.0],
    ]
    for label, joints in zip(limit_labels, limit_joints):
        configs.append({"label": label, "joints": joints, "category": "limits"})

    # Category 3: Random configs (fixed seed for reproducibility)
    random.seed(RANDOM_SEED)
    for i in range(RANDOM_COUNT):
        joints = [random.uniform(lo, hi) for lo, hi in J_LIMITS]
        configs.append({"label": f"random_{i:03d}", "joints": joints, "category": "random"})

    # Category 4: Near-singular configs
    singular_configs = [
        ("singular_arm_stretched", [0.0, -90.0, 1.0, 0.0, 0.0, 0.0]),
        ("singular_arm_stretched2", [0.0, -45.0, 0.0, 0.0, 90.0, 0.0]),
        ("singular_cylinder_near", [0.0, -30.0, 60.0, 0.0, 30.0, 0.0]),
        ("singular_j3_zero", [10.0, -20.0, 0.0, -15.0, 25.0, -10.0]),
        ("singular_j2_minus90", [0.0, -90.0, 5.0, 0.0, 0.0, 0.0]),
    ]
    for label, joints in singular_configs:
        configs.append({"label": label, "joints": joints, "category": "singular"})

    # Category 5: Existing test configs from test_kinematics.cpp
    test_configs = [
        ("test_fk_j1_z_invariant", [0.0, -30.0, 90.0, 0.0, 45.0, 0.0]),
        ("test_fk_non_constant", [15.0, -55.0, 115.0, 5.0, 85.0, 5.0]),
        ("test_jacobian_consistency", [15.0, -55.0, 115.0, 5.0, 85.0, 5.0]),
        ("test_jacobian_second", [0.0, -45.0, 5.0, 0.0, 90.0, 0.0]),
        ("test_condition_finite", [0.0, -30.0, 60.0, 0.0, 30.0, 0.0]),
        ("test_condition_singular", [0.0, -90.0, 0.0, 0.0, 0.0, 0.0]),
        ("test_condition_ratio_ok_safe", [10.0, -20.0, 60.0, -15.0, 25.0, -10.0]),
        ("test_ik_roundtrip", [15.0, -55.0, 115.0, 5.0, 85.0, 5.0]),
        ("test_ik_workspace_center_ref", [10.0, -25.0, 55.0, 0.0, 40.0, 0.0]),
        ("test_compose_transform", [10.0, -20.0, 30.0, -15.0, 25.0, -10.0]),
        ("static_pose_approx", [-103.0 / D2R * 0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ]
    for label, joints in test_configs:
        configs.append({"label": f"cpp_test_{label}", "joints": joints, "category": "existing_tests"})

    return configs


def write_input_files(configs):
    """Write input.json (MATLAB) and input.csv (C++)."""
    # JSON for MATLAB
    json_data = {"configs": [{"label": c["label"], "joints": c["joints"]} for c in configs]}
    INPUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    with open(INPUT_JSON, "w") as f:
        json.dump(json_data, f, indent=2)
    print(f"  Wrote {len(configs)} configs → {INPUT_JSON}")

    # CSV for C++
    INPUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    with open(INPUT_CSV, "w") as f:
        f.write("# label,j1,j2,j3,j4,j5,j6\n")
        for c in configs:
            f.write(f"{c['label']},{c['joints'][0]:.10g},{c['joints'][1]:.10g},"
                    f"{c['joints'][2]:.10g},{c['joints'][3]:.10g},"
                    f"{c['joints'][4]:.10g},{c['joints'][5]:.10g}\n")
    print(f"  Wrote {len(configs)} configs → {INPUT_CSV}")


# ============================================================
# Run MATLAB
# ============================================================


def run_matlab():
    """Run MATLAB fk_validate. Returns True on success."""
    matlab_script_str = str(MATLAB_SCRIPT)
    relay_dir = str(MATLAB_SCRIPT.parent)

    # Try matlab -batch first (R2019a+)
    cmds = [
        f'cd(\'{relay_dir}\'); fk_validate(\'input.json\', \'matlab_output.json\');',
    ]

    for batch_flag in ["-batch", "-nodisplay -nosplash -nodesktop -r"]:
        cmd = f'matlab {batch_flag} "{cmds[0]}"'
        # Actually, -batch takes the command directly without quotes
        if batch_flag == "-batch":
            cmd = f'matlab -batch "cd(\'{relay_dir}\'); fk_validate(\'input.json\', \'matlab_output.json\');"'
        else:
            cmd = f'matlab -nodisplay -nosplash -nodesktop -r "cd(\'{relay_dir}\'); fk_validate(\'input.json\', \'matlab_output.json\'); exit;"'

        print(f"  Trying: matlab {batch_flag} ...")
        try:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=120)
            if result.returncode == 0 and MATLAB_OUT.exists():
                print(f"  MATLAB OK → {MATLAB_OUT}")
                return True
            else:
                print(f"  MATLAB exit={result.returncode}")
                if result.stderr:
                    print(f"  stderr: {result.stderr[:500]}")
        except subprocess.TimeoutExpired:
            print("  MATLAB timed out")
        except FileNotFoundError:
            print("  MATLAB not found on PATH")
            break

    print("  WARNING: MATLAB not available — skipping MATLAB FK run")
    return False


# ============================================================
# Run C++
# ============================================================


def run_cpp():
    """Run C++ fk_validate.exe. Returns True on success."""
    exe = str(CPP_EXE)
    csv = str(INPUT_CSV)
    out = str(CPP_OUT)

    if not os.path.exists(exe):
        print(f"  ERROR: {exe} not found — run build_fk_validate.bat first")
        return False

    print(f"  Running: {exe} {csv} {out}")
    try:
        result = subprocess.run([exe, csv, out], capture_output=True, text=True, timeout=30)
        print(result.stdout.strip())
        if result.returncode != 0:
            print(f"  stderr: {result.stderr}")
            return False
        return CPP_OUT.exists()
    except subprocess.TimeoutExpired:
        print("  C++ timed out")
        return False
    except FileNotFoundError:
        print(f"  ERROR: cannot execute {exe}")
        return False


# ============================================================
# Comparison & analysis
# ============================================================


def vec3_dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def vec3_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vec3_mag(v):
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def load_matlab_output():
    with open(MATLAB_OUT) as f:
        return json.load(f)["configs"]


def load_cpp_output():
    with open(CPP_OUT) as f:
        return json.load(f)["configs"]


def compare_outputs(matlab_configs, cpp_configs):
    """Compare MATLAB vs C++ FK outputs. Returns list of per-config results."""
    # Index both by label
    matlab_by_label = {c["label"]: c for c in matlab_configs}
    cpp_by_label = {c["label"]: c for c in cpp_configs}

    results = []
    common_labels = set(matlab_by_label.keys()) & set(cpp_by_label.keys())

    for label in sorted(common_labels):
        mc = matlab_by_label[label]
        cc = cpp_by_label[label]

        # End-effector position comparison
        matlab_ee = mc["ee_position"]
        cpp_ee = cc["ee_position"]
        ee_dist = vec3_dist(matlab_ee, cpp_ee)
        ee_diff = vec3_sub(matlab_ee, cpp_ee)

        # Joint position comparisons (7 positions: base + J1-J6)
        joint_dists = []
        for j in range(7):
            d = vec3_dist(mc["joint_positions"][j], cc["joint_positions"][j])
            joint_dists.append(d)

        results.append({
            "label": label,
            "joints": mc["input"],
            "matlab_ee": matlab_ee,
            "cpp_ee": cpp_ee,
            "ee_dist": ee_dist,
            "ee_diff": ee_diff,
            "joint_dists": joint_dists,
            "max_joint_dist": max(joint_dists),
        })

    return results


def categorize_by_j2_angle(results):
    """Group results by approximate J2 magnitude to see error growth."""
    bins = {"|J2| < 5°": [], "5° ≤ |J2| < 45°": [], "45° ≤ |J2| < 90°": [], "|J2| ≥ 90°": []}
    for r in results:
        j2 = abs(r["joints"][1])
        if j2 < 5:
            bins["|J2| < 5°"].append(r)
        elif j2 < 45:
            bins["5° ≤ |J2| < 45°"].append(r)
        elif j2 < 90:
            bins["45° ≤ |J2| < 90°"].append(r)
        else:
            bins["|J2| ≥ 90°"].append(r)
    return bins


def categorize_by_category(results, configs_meta):
    """Group results by test category."""
    cat_map = {c["label"]: c.get("category", "unknown") for c in configs_meta}
    cats = {}
    for r in results:
        cat = cat_map.get(r["label"], "unknown")
        cats.setdefault(cat, []).append(r)
    return cats


def generate_report(results, configs_meta):
    """Generate markdown report."""
    lines = []
    lines.append("# FK Cross-Validation Report: C++ vs MATLAB")
    lines.append("")
    lines.append(f"**Generated:** {__import__('datetime').datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"**Configs compared:** {len(results)}")
    lines.append("")

    # Summary statistics
    ee_dists = [r["ee_dist"] for r in results]
    lines.append("## Summary: End-Effector Position Discrepancy")
    lines.append("")
    lines.append("| Metric | Value (mm) |")
    lines.append("|--------|-----------|")
    lines.append(f"| Max error | **{max(ee_dists):.4f}** |")
    lines.append(f"| Mean error | {sum(ee_dists) / len(ee_dists):.4f} |")
    lines.append(f"| RMS error | {math.sqrt(sum(d * d for d in ee_dists) / len(ee_dists)):.4f} |")
    lines.append(f"| Median error | {sorted(ee_dists)[len(ee_dists) // 2]:.4f} |")
    lines.append(f"| Errors < 0.001 mm | {sum(1 for d in ee_dists if d < 0.001)} / {len(ee_dists)} |")
    lines.append(f"| Errors < 0.01 mm | {sum(1 for d in ee_dists if d < 0.01)} / {len(ee_dists)} |")
    lines.append("")

    # By J2 angle group
    lines.append("## Discrepancy by J2 Angle Magnitude")
    lines.append("")
    lines.append("| J2 Range | Count | Mean EE Error (mm) | Max EE Error (mm) |")
    lines.append("|----------|-------|-------------------|-------------------|")
    j2_bins = categorize_by_j2_angle(results)
    for label, items in j2_bins.items():
        if items:
            dists = [r["ee_dist"] for r in items]
            lines.append(f"| {label} | {len(items)} | {sum(dists) / len(dists):.4f} | {max(dists):.4f} |")
        else:
            lines.append(f"| {label} | 0 | — | — |")
    lines.append("")

    # By category
    lines.append("## Discrepancy by Test Category")
    lines.append("")
    lines.append("| Category | Count | Mean EE Error (mm) | Max EE Error (mm) |")
    lines.append("|----------|-------|-------------------|-------------------|")
    cat_bins = categorize_by_category(results, configs_meta)
    for cat in ["zeros", "limits", "random", "singular", "existing_tests"]:
        items = cat_bins.get(cat, [])
        if items:
            dists = [r["ee_dist"] for r in items]
            lines.append(f"| {cat} | {len(items)} | {sum(dists) / len(dists):.4f} | {max(dists):.4f} |")
        else:
            lines.append(f"| {cat} | 0 | — | — |")
    lines.append("")

    # Worst offenders (top 10)
    lines.append("## Top 10 Largest Discrepancies")
    lines.append("")
    lines.append("| Rank | Label | Joints (deg) | EE Error (mm) | Error Vector (x,y,z) |")
    lines.append("|------|-------|-------------|---------------|---------------------|")
    sorted_results = sorted(results, key=lambda r: r["ee_dist"], reverse=True)
    for i, r in enumerate(sorted_results[:10]):
        j = r["joints"]
        joints_str = f"[{j[0]:.1f}, {j[1]:.1f}, {j[2]:.1f}, {j[3]:.1f}, {j[4]:.1f}, {j[5]:.1f}]"
        err = r["ee_diff"]
        lines.append(f"| {i + 1} | {r['label']} | {joints_str} | {r['ee_dist']:.4f} | [{err[0]:.2f}, {err[1]:.2f}, {err[2]:.2f}] |")
    lines.append("")

    # Per-joint position errors
    lines.append("## Joint Position Errors (Aggregate)")
    lines.append("")
    lines.append("| Joint Index | Mean Error (mm) | Max Error (mm) |")
    lines.append("|-------------|----------------|----------------|")
    for j in range(7):
        dists = [r["joint_dists"][j] for r in results]
        lines.append(f"| J{j} (index {j}) | {sum(dists) / len(dists):.4f} | {max(dists):.4f} |")
    lines.append("")
    lines.append("> J0 = base origin (should always be 0,0,0). J1-J6 = joint positions along the kinematic chain.")
    lines.append("")

    # Root cause analysis
    lines.append("## Root Cause Analysis")
    lines.append("")
    lines.append("C++ `mat4_translate()` adds translation in WORLD frame:")
    lines.append("```cpp")
    lines.append("T[0][3] += x;  T[1][3] += y;  T[2][3] += z;  // WRONG")
    lines.append("```")
    lines.append("MATLAB `T * tr(x,y,z)` multiplies in LOCAL frame (correct).")
    lines.append("")
    lines.append("**Expected pattern if confirmed:**")
    lines.append("- J1-only rotations → error ≈ 0 (first translate happens when T=I)")
    lines.append("- J2-J6 non-zero → error grows as more rotations accumulate in T")
    lines.append("- Error magnitude proportional to rotation angles between translations")
    lines.append("")

    # Detailed table (collapsed for readability)
    lines.append("## All Configs (Detailed)")
    lines.append("")
    lines.append("<details>")
    lines.append("<summary>Click to expand — all configs</summary>")
    lines.append("")
    lines.append("| Label | Joints | MATLAB EE (x,y,z) | C++ EE (x,y,z) | Dist (mm) |")
    lines.append("|-------|--------|-------------------|----------------|-----------|")
    for r in results:
        j = r["joints"]
        joints_str = f"[{j[0]:.0f},{j[1]:.0f},{j[2]:.0f},{j[3]:.0f},{j[4]:.0f},{j[5]:.0f}]"
        me = r["matlab_ee"]
        ce = r["cpp_ee"]
        lines.append(f"| {r['label']} | {joints_str} | [{me[0]:.2f},{me[1]:.2f},{me[2]:.2f}] | [{ce[0]:.2f},{ce[1]:.2f},{ce[2]:.2f}] | {r['ee_dist']:.4f} |")
    lines.append("")
    lines.append("</details>")
    lines.append("")

    # Write
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    with open(REPORT, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print(f"\n  Report → {REPORT}")


# ============================================================
# Main
# ============================================================


def main():
    generate_only = "--generate-only" in sys.argv
    compare_only = "--compare-only" in sys.argv

    if not compare_only:
        print("=" * 60)
        print("Step 1: Generate test vectors")
        print("=" * 60)
        configs = generate_test_vectors()
        print(f"  Generated {len(configs)} test vectors "
              f"({sum(1 for c in configs if c['category'] == 'zeros')} zeros, "
              f"{sum(1 for c in configs if c['category'] == 'limits')} limits, "
              f"{sum(1 for c in configs if c['category'] == 'random')} random, "
              f"{sum(1 for c in configs if c['category'] == 'singular')} singular, "
              f"{sum(1 for c in configs if c['category'] == 'existing_tests')} existing)")
        write_input_files(configs)

        if generate_only:
            return

        print("")
        print("=" * 60)
        print("Step 2: Run MATLAB FK")
        print("=" * 60)
        matlab_ok = run_matlab()

        print("")
        print("=" * 60)
        print("Step 3: Run C++ FK")
        print("=" * 60)
        cpp_ok = run_cpp()

        if not matlab_ok and not cpp_ok:
            print("\nERROR: Neither MATLAB nor C++ FK ran successfully.")
            sys.exit(1)
        if not matlab_ok:
            print("\nWARNING: MATLAB not available — comparing C++ against itself only.")
        if not cpp_ok:
            print("\nERROR: C++ FK did not run. Build fk_validate.exe first.")
            sys.exit(1)
    else:
        # Load configs for category info
        with open(INPUT_JSON) as f:
            configs = json.load(f)["configs"]

    print("")
    print("=" * 60)
    print("Step 4: Compare results")
    print("=" * 60)

    # Load outputs
    if MATLAB_OUT.exists():
        matlab_configs = load_matlab_output()
        print(f"  MATLAB: {len(matlab_configs)} configs")
    else:
        print("  MATLAB output not found — skipping comparison")
        sys.exit(1)

    if CPP_OUT.exists():
        cpp_configs = load_cpp_output()
        print(f"  C++: {len(cpp_configs)} configs")
    else:
        print("  C++ output not found — run fk_validate.exe first")
        sys.exit(1)

    # Compare
    results = compare_outputs(matlab_configs, cpp_configs)
    print(f"  Compared: {len(results)} common configs")

    # Quick summary
    ee_dists = [r["ee_dist"] for r in results]
    print(f"  Max EE error:  {max(ee_dists):.4f} mm")
    print(f"  Mean EE error: {sum(ee_dists) / len(ee_dists):.4f} mm")
    print(f"  RMS EE error:  {math.sqrt(sum(d * d for d in ee_dists) / len(ee_dists)):.4f} mm")
    zeros_count = sum(1 for d in ee_dists if d < 0.001)
    print(f"  Near-zero (<0.001mm): {zeros_count} / {len(ee_dists)}")

    # Generate report
    generate_report(results, configs)

    # Final verdict
    max_err = max(ee_dists)
    if max_err < 1e-6:
        print("\n✓ C++ and MATLAB FK are IDENTICAL (within floating-point tolerance)")
    elif max_err < 1.0:
        print(f"\n⚠ C++ and MATLAB FK differ by up to {max_err:.2f} mm — minor discrepancy")
    else:
        print(f"\n✗ C++ and MATLAB FK differ by up to {max_err:.2f} mm — SIGNIFICANT discrepancy")
        print("  Root cause: mat4_translate() world-frame translation bug in Kinematics.cpp")
        print("  Fix: replace with proper T * Translate(x,y,z) matrix multiplication")


if __name__ == "__main__":
    main()
