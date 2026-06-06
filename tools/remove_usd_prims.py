#!/usr/bin/env python3
"""Remove matching prims from a USD stage without opening Isaac Sim GUI.

Run with the Isaac Sim Python environment so the pxr module is available.

Example:
~/venv/issac_env/bin/python \
~/isaac_amr_ws/tools/remove_usd_prims.py \
--input ~/isaac_amr_ws/assets/warehouse_robot_arm_test_1.usd \
--output ~/isaac_amr_ws/assets/warehouse_robot_arm_test_1_no_people.usd \
--match DHGen \
--match DH_Characters \
--match People
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

try:
    from pxr import Usd
    _SIMULATION_APP = None
except ModuleNotFoundError:
    from isaacsim import SimulationApp

    _SIMULATION_APP = SimulationApp({"headless": True})
    from pxr import Usd


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Input USD file")
    parser.add_argument("--output", required=True, help="Output USD file")
    parser.add_argument(
        "--match",
        action="append",
        default=[],
        help="Substring to match against prim path, prim name, reference, or payload asset path",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Only print matching prims; do not remove or save",
    )
    args = parser.parse_args()

    input_path = Path(args.input).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    matches = args.match or ["DHGen", "DH_Characters", "People"]

    if not input_path.exists():
        raise FileNotFoundError(input_path)

    if not args.dry_run:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(input_path, output_path)
        stage_path = output_path
    else:
        stage_path = input_path

    stage = Usd.Stage.Open(str(stage_path))
    if stage is None:
        raise RuntimeError(f"Failed to open USD stage: {stage_path}")

    to_remove = []
    for prim in stage.TraverseAll():
        path_text = str(prim.GetPath())
        name_text = prim.GetName()
        asset_texts = []

        metadata = prim.GetMetadata("references")
        if metadata:
            asset_texts.append(str(metadata))
        payload = prim.GetMetadata("payload")
        if payload:
            asset_texts.append(str(payload))

        haystack = " ".join([path_text, name_text, *asset_texts])
        if any(token in haystack for token in matches):
            to_remove.append(prim.GetPath())

    # Remove parent paths first and skip children that are already covered.
    unique = []
    for path in sorted(to_remove, key=lambda p: len(str(p))):
        if not any(str(path).startswith(str(parent) + "/") for parent in unique):
            unique.append(path)

    print(f"Stage: {stage_path}")
    print(f"Match tokens: {matches}")
    print(f"Matched prims: {len(unique)}")
    for path in unique:
        print(f"  {path}")

    if args.dry_run:
        return 0

    for path in unique:
        stage.RemovePrim(path)

    stage.GetRootLayer().Save()
    print(f"Saved: {output_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        if _SIMULATION_APP is not None:
            _SIMULATION_APP.close()
