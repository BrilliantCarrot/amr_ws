#!/usr/bin/env python3
"""
Dynamic obstacle safety evaluator for the Isaac AMR scenario.

This script turns one dynamic-obstacle test run into README-friendly numbers:
minimum clearance, collision/safety-violation ratios, CBF intervention ratio,
FrontSafety stop count, command smoothness, and control latency.

코드 사용 방법:
기존 mpc 노드까지의 ros2 launch scenarios isaac_navigation_bringup_lidar_only.launch.py를 입력하여 실행한 다음, 밑의 코드를 입력하여 bag으로 기록
    ros2 bag record /cmd_vel /metrics/min_obstacle_distance /metrics/control_latency_ms -o ~/isaac_amr_ws/bags/dynamic_obstacle_cbf_test

시뮬레이션 종료 후 밑에를 실행
    python3 ~/isaac_amr_ws/tools/dynamic_obstacle_safety_eval.py \
    --bag ~/isaac_amr_ws/bags/dynamic_obstacle_cbf_test \
    --log ~/result.txt \
    --out-prefix ~/isaac_amr_ws/tools/dynamic_obstacle_cbf_eval_avoidance_only \
    --safety-threshold 0.20 \
    --w-limit 0.42 \
    --start-sec 2 \
    --end-sec 25

출력 파일:
- tools/dynamic_obstacle_cbf_eval.json
- tools/dynamic_obstacle_cbf_eval.md

뽑히는 핵심 지표:
- 충돌 샘플 수
- 최소 장애물 이격거리
- 0.20m 미만 접근 비율
- CBF 개입 비율
- FrontSafety 정지 횟수
- 정지 비율
- 최대 각속도
- MPC solve time 평균/p99
- e2e latency 평균/p99    
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sqlite3
import struct
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import numpy as np
except Exception as exc:  # pragma: no cover - dependency check path
    raise RuntimeError("numpy가 필요합니다. ROS2 Python 환경에서 실행하세요.") from exc

try:
    from rclpy.serialization import deserialize_message
    from amr_msgs.msg import MinObstacleDistance
except Exception:
    deserialize_message = None
    MinObstacleDistance = None


RawRows = List[Tuple[float, bytes]]
NumberSeries = List[Tuple[float, float]]
CmdSeries = List[Tuple[float, float, float]]
LatencySeries = List[Tuple[float, float, float]]


CBF_RESULT_RE = re.compile(
    r"\[CBF result\].*?ok=(?P<ok>true|false).*?"
    r"u_nom=\((?P<v_nom>[-0-9.]+),\s*(?P<w_nom>[-0-9.]+)\).*?"
    r"u_safe=\((?P<v_safe>[-0-9.]+),\s*(?P<w_safe>[-0-9.]+)\)"
)
CBF_PRE_RE = re.compile(
    r"\[CBF dbg pre\]\s*N=(?P<N>\d+)\s+"
    r"clr=(?P<clr>[-0-9.eE+]+)\s+"
    r"fwd=(?P<fwd>[-0-9.eE+]+).*?"
    r"lhs_nom=(?P<lhs>[-0-9.eE+]+)\s+"
    r"rhs=(?P<rhs>[-0-9.eE+]+)"
)
FRONT_SAFETY_RE = re.compile(
    r"\[FrontSafety\]\s+found=(?P<found>[01])\s+"
    r"clear=(?P<clear>[-0-9.eE+]+)\s+"
    r"filtered=(?P<filtered>[-0-9.eE+]+)\s+"
    r"stop=(?P<stop>[01])\s+"
    r"v=(?P<v>[-0-9.eE+]+)\s+"
    r"w=(?P<w>[-0-9.eE+]+)"
)
MPC_SUMMARY_RE = re.compile(
    r"\[MPC\]\s+solve=(?P<solve>[-0-9.]+)ms.*?"
    r"e2e=(?P<e2e>[-0-9.]+)ms.*?"
    r"u_nom=\((?P<v_nom>[-0-9.]+),\s*(?P<w_nom>[-0-9.]+)\)\s+"
    r"u_pub=\((?P<v_pub>[-0-9.]+),\s*(?P<w_pub>[-0-9.]+)\)"
)


def expand(path_str: str) -> Path:
    return Path(path_str).expanduser().resolve()


def find_db3_path(bag_path: Path) -> Path:
    if bag_path.is_file() and bag_path.suffix == ".db3":
        return bag_path
    if not bag_path.exists():
        raise FileNotFoundError(f"bag 경로가 없습니다: {bag_path}")
    candidates = sorted(bag_path.glob("*.db3"))
    if not candidates:
        candidate = bag_path / f"{bag_path.name}_0.db3"
        if candidate.exists():
            return candidate
        raise FileNotFoundError(f"db3 파일을 찾지 못했습니다: {bag_path}")
    return candidates[0]


def read_raw_topic(bag_path: Path, topic_name: str) -> RawRows:
    db_path = find_db3_path(bag_path)
    conn = sqlite3.connect(str(db_path))
    try:
        cursor = conn.cursor()
        cursor.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
        row = cursor.fetchone()
        if row is None:
            return []

        topic_id = row[0]
        cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topic_id,),
        )
        return [(ts_ns * 1e-9, bytes(data)) for ts_ns, data in cursor.fetchall()]
    finally:
        conn.close()


def normalize(rows: RawRows) -> RawRows:
    if not rows:
        return []
    t0 = rows[0][0]
    return [(t - t0, data) for t, data in rows]


def summarize(values: Sequence[float]) -> Dict[str, Optional[float]]:
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return {
            "count": 0,
            "mean": None,
            "std": None,
            "min": None,
            "p01": None,
            "p05": None,
            "p50": None,
            "p95": None,
            "p99": None,
            "max": None,
        }
    return {
        "count": int(arr.size),
        "mean": float(np.mean(arr)),
        "std": float(np.std(arr)),
        "min": float(np.min(arr)),
        "p01": float(np.percentile(arr, 1)),
        "p05": float(np.percentile(arr, 5)),
        "p50": float(np.percentile(arr, 50)),
        "p95": float(np.percentile(arr, 95)),
        "p99": float(np.percentile(arr, 99)),
        "max": float(np.max(arr)),
    }


def parse_twist(rows: RawRows) -> CmdSeries:
    result: CmdSeries = []
    for t, data in normalize(rows):
        try:
            vals = struct.unpack_from("<6d", data, 4)
            result.append((t, float(vals[0]), float(vals[5])))
        except Exception:
            continue
    return result


def parse_latency(rows: RawRows) -> LatencySeries:
    result: LatencySeries = []
    for t, data in normalize(rows):
        try:
            vals = struct.unpack_from("<6d", data, len(data) - 48)
            result.append((t, float(vals[0]), float(vals[3])))
        except Exception:
            continue
    return result


def align(offset: int, boundary: int, base: int = 0) -> int:
    remainder = (offset - base) % boundary
    return offset if remainder == 0 else offset + boundary - remainder


def parse_min_distance_cdr(data: bytes) -> Optional[float]:
    """Parse amr_msgs/MinObstacleDistance directly from CDR bytes.

    Layout:
      CDR encapsulation      : 4 bytes
      std_msgs/Header.stamp  : int32 sec + uint32 nanosec
      std_msgs/Header.frame  : uint32 string length + string bytes + padding
      min_distance_m         : float64, 8-byte aligned
      is_critical            : bool
    """
    try:
        offset = 4
        offset += 4  # stamp.sec
        offset += 4  # stamp.nanosec
        frame_len = struct.unpack_from("<I", data, offset)[0]
        offset += 4 + frame_len
        offset = align(offset, 4, base=4)
        offset = align(offset, 8, base=4)
        value = struct.unpack_from("<d", data, offset)[0]
        if math.isfinite(value) and -10.0 < value < 100.0:
            return float(value)
    except Exception:
        pass
    return None


def parse_min_distance(rows: RawRows) -> NumberSeries:
    result: NumberSeries = []
    normalized = normalize(rows)
    if not normalized:
        return result

    for t, data in normalized:
        value = parse_min_distance_cdr(data)
        if deserialize_message is not None and MinObstacleDistance is not None:
            if value is None or abs(value) < 1e-300:
                try:
                    msg = deserialize_message(data, MinObstacleDistance)
                    deserialized = float(msg.min_distance_m)
                    if math.isfinite(deserialized) and -10.0 < deserialized < 100.0:
                        value = deserialized
                except Exception:
                    pass
        if value is not None and math.isfinite(value) and abs(value) < 900.0:
            result.append((t, value))
    return result


def parse_log(log_path: Optional[Path]) -> Dict[str, List[Dict[str, float]]]:
    parsed: Dict[str, List[Dict[str, float]]] = {
        "cbf_result": [],
        "cbf_pre": [],
        "front_safety": [],
        "mpc_summary": [],
    }
    if log_path is None or not log_path.exists():
        return parsed

    with log_path.open("r", encoding="utf-8", errors="ignore") as f:
        for idx, line in enumerate(f):
            m = CBF_RESULT_RE.search(line)
            if m:
                item: Dict[str, float] = {"idx": float(idx)}
                item["ok"] = 1.0 if m.group("ok") == "true" else 0.0
                for key in ("v_nom", "w_nom", "v_safe", "w_safe"):
                    item[key] = float(m.group(key))
                parsed["cbf_result"].append(item)
                continue

            m = CBF_PRE_RE.search(line)
            if m:
                item = {"idx": float(idx)}
                for key in ("N", "clr", "fwd", "lhs", "rhs"):
                    item[key] = float(m.group(key))
                parsed["cbf_pre"].append(item)
                continue

            m = FRONT_SAFETY_RE.search(line)
            if m:
                item = {"idx": float(idx)}
                for key in ("found", "clear", "filtered", "stop", "v", "w"):
                    item[key] = float(m.group(key))
                parsed["front_safety"].append(item)
                continue

            m = MPC_SUMMARY_RE.search(line)
            if m:
                item = {"idx": float(idx)}
                for key in ("solve", "e2e", "v_nom", "w_nom", "v_pub", "w_pub"):
                    item[key] = float(m.group(key))
                parsed["mpc_summary"].append(item)

    return parsed


def calc_clearance_metrics(
    dist: NumberSeries,
    thresholds: Sequence[float],
    collision_threshold: float,
) -> Dict[str, Any]:
    values = np.asarray([x[1] for x in dist], dtype=float)
    result: Dict[str, Any] = {
        "stats": summarize(values.tolist()),
        "collision_threshold_m": collision_threshold,
        "collision_count": 0,
        "collision_ratio": None,
        "thresholds": {},
    }
    if values.size == 0:
        return result

    result["collision_count"] = int(np.sum(values <= collision_threshold))
    result["collision_ratio"] = float(np.mean(values <= collision_threshold))
    for threshold in thresholds:
        result["thresholds"][f"{threshold:.2f}"] = {
            "count_below": int(np.sum(values < threshold)),
            "ratio_below": float(np.mean(values < threshold)),
        }
    return result


def calc_cmd_metrics(cmd: CmdSeries, w_limit: float) -> Dict[str, Any]:
    if len(cmd) < 2:
        return {}

    t = np.asarray([x[0] for x in cmd], dtype=float)
    v = np.asarray([x[1] for x in cmd], dtype=float)
    w = np.asarray([x[2] for x in cmd], dtype=float)
    dv = np.diff(v)
    dw = np.diff(w)
    duration = float(t[-1] - t[0])

    active_w = w[np.abs(w) > 0.02]
    sign_flips = 0
    if active_w.size >= 2:
        sign_flips = int(np.sum(np.sign(active_w[1:]) != np.sign(active_w[:-1])))

    return {
        "duration_s": duration,
        "sample_count": int(len(cmd)),
        "v": summarize(v.tolist()),
        "w": summarize(w.tolist()),
        "abs_w": summarize(np.abs(w).tolist()),
        "delta_v": summarize(dv.tolist()),
        "delta_w": summarize(dw.tolist()),
        "stop_ratio_abs_v_lt_0p01": float(np.mean(np.abs(v) < 0.01)),
        "slow_ratio_abs_v_lt_0p03": float(np.mean(np.abs(v) < 0.03)),
        "w_clip_ratio": float(np.mean(np.abs(w) >= 0.999 * w_limit)),
        "w_sign_flip_count": sign_flips,
        "w_sign_flip_rate_per_s": float(sign_flips / duration) if duration > 1e-9 else None,
    }


def calc_latency_metrics(lat: LatencySeries) -> Dict[str, Any]:
    if not lat:
        return {}
    solve = [x[1] for x in lat]
    e2e = [x[2] for x in lat]
    return {
        "solve_ms": summarize(solve),
        "e2e_ms": summarize(e2e),
        "e2e_over_50ms_count": int(np.sum(np.asarray(e2e) > 50.0)),
        "e2e_over_50ms_ratio": float(np.mean(np.asarray(e2e) > 50.0)),
    }


def calc_log_metrics(parsed_log: Dict[str, List[Dict[str, float]]]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}

    cbf = parsed_log["cbf_result"]
    if cbf:
        dv = np.asarray([abs(x["v_safe"] - x["v_nom"]) for x in cbf], dtype=float)
        dw = np.asarray([abs(x["w_safe"] - x["w_nom"]) for x in cbf], dtype=float)
        result["cbf"] = {
            "sample_count": int(len(cbf)),
            "ok_ratio": float(np.mean([x["ok"] for x in cbf])),
            "intervention_ratio": float(np.mean((dv > 0.01) | (dw > 0.03))),
            "strong_turn_intervention_ratio": float(np.mean(dw > 0.20)),
            "abs_delta_v": summarize(dv.tolist()),
            "abs_delta_w": summarize(dw.tolist()),
        }

    pre = parsed_log["cbf_pre"]
    if pre:
        clr = np.asarray([x["clr"] for x in pre], dtype=float)
        lhs = np.asarray([x["lhs"] for x in pre], dtype=float)
        rhs = np.asarray([x["rhs"] for x in pre], dtype=float)
        result["cbf_constraint"] = {
            "sample_count": int(len(pre)),
            "clearance": summarize(clr.tolist()),
            "nominal_violation_ratio_lhs_lt_rhs": float(np.mean(lhs < rhs)),
            "very_close_ratio_clr_lt_0": float(np.mean(clr < 0.0)),
        }

    front = parsed_log["front_safety"]
    if front:
        found = np.asarray([x["found"] for x in front], dtype=float)
        stop = np.asarray([x["stop"] for x in front], dtype=float)
        filtered = np.asarray(
            [x["filtered"] for x in front if abs(x["filtered"]) < 900.0],
            dtype=float,
        )
        result["front_safety"] = {
            "sample_count": int(len(front)),
            "found_ratio": float(np.mean(found > 0.5)),
            "stop_ratio": float(np.mean(stop > 0.5)),
            "stop_count": int(np.sum(stop > 0.5)),
            "filtered_clearance": summarize(filtered.tolist()),
        }

    mpc = parsed_log["mpc_summary"]
    if mpc:
      dv = np.asarray([abs(x["v_pub"] - x["v_nom"]) for x in mpc], dtype=float)
      dw = np.asarray([abs(x["w_pub"] - x["w_nom"]) for x in mpc], dtype=float)
      result["mpc_command"] = {
          "sample_count": int(len(mpc)),
          "intervention_ratio": float(np.mean((dv > 0.01) | (dw > 0.03))),
          "abs_delta_v_pub_nom": summarize(dv.tolist()),
          "abs_delta_w_pub_nom": summarize(dw.tolist()),
      }

    front_stop_ratio = result.get("front_safety", {}).get("stop_ratio")
    mpc_intervention_ratio = result.get("mpc_command", {}).get("intervention_ratio")
    cbf_intervention_ratio = result.get("cbf", {}).get("intervention_ratio")
    candidates = [
        x for x in (front_stop_ratio, mpc_intervention_ratio, cbf_intervention_ratio)
        if x is not None
    ]
    if candidates:
        result["reactive_safety"] = {
            "intervention_ratio": float(max(candidates)),
            "front_stop_count": int(result.get("front_safety", {}).get("stop_count", 0)),
            "source": "max(front_safety_stop, mpc_cmd_delta, cbf_cmd_delta)",
        }

    return result


def fmt(value: Optional[float], unit: str = "", digits: int = 2) -> str:
    if value is None:
        return "N/A"
    return f"{value:.{digits}f}{unit}"


def pct(value: Optional[float], digits: int = 2) -> str:
    if value is None:
        return "N/A"
    return f"{value * 100.0:.{digits}f}%"


def make_verdict(summary: Dict[str, Any], safety_threshold: float) -> List[str]:
    verdicts: List[str] = []
    clearance = summary["clearance"]
    stats = clearance["stats"]

    min_clearance = stats.get("min")
    collision_count = clearance.get("collision_count")
    if min_clearance is None:
        verdicts.append("CHECK: /metrics/min_obstacle_distance 데이터가 없어 안전거리 검증을 완료하지 못함")
    elif collision_count == 0:
        verdicts.append(f"PASS: 충돌 샘플 0개, 최소 이격거리 {min_clearance:.3f}m 유지")
    else:
        verdicts.append(f"FAIL: 충돌/접촉 샘플 {collision_count}개 발생, 최소 이격거리 {min_clearance:.3f}m")

    threshold_key = f"{safety_threshold:.2f}"
    threshold = clearance["thresholds"].get(threshold_key)
    if threshold:
        ratio = threshold["ratio_below"]
        if ratio <= 0.01:
            verdicts.append(f"PASS: safety threshold {safety_threshold:.2f}m 미만 비율 {ratio * 100.0:.2f}%")
        else:
            verdicts.append(f"CHECK: safety threshold {safety_threshold:.2f}m 미만 비율 {ratio * 100.0:.2f}%")

    latency = summary.get("latency", {})
    e2e_p99 = latency.get("e2e_ms", {}).get("p99") if latency else None
    if e2e_p99 is not None:
        if e2e_p99 < 50.0:
            verdicts.append(f"PASS: e2e latency p99 {e2e_p99:.2f}ms < 50ms")
        else:
            verdicts.append(f"CHECK: e2e latency p99 {e2e_p99:.2f}ms")

    reactive = summary.get("log", {}).get("reactive_safety", {})
    intervention = reactive.get("intervention_ratio") if reactive else None
    if intervention is not None:
        verdicts.append(f"INFO: reactive safety intervention ratio {intervention * 100.0:.2f}%")

    return verdicts


def make_markdown(summary: Dict[str, Any], safety_threshold: float) -> str:
    clearance = summary["clearance"]
    clearance_stats = clearance["stats"]
    threshold = clearance["thresholds"].get(f"{safety_threshold:.2f}", {})
    cmd = summary.get("cmd_vel", {})
    latency = summary.get("latency", {})
    log = summary.get("log", {})
    cbf = log.get("cbf", {})
    front = log.get("front_safety", {})
    reactive = log.get("reactive_safety", {})

    rows = [
        ("Collision count", str(clearance.get("collision_count", "N/A"))),
        ("Minimum obstacle clearance", fmt(clearance_stats.get("min"), " m", 3)),
        (f"Clearance < {safety_threshold:.2f} m ratio", pct(threshold.get("ratio_below"))),
        ("Reactive safety intervention ratio", pct(reactive.get("intervention_ratio"))),
        ("CBF command correction ratio", pct(cbf.get("intervention_ratio"))),
        ("CBF ok ratio", pct(cbf.get("ok_ratio"))),
        ("FrontSafety stop count", str(front.get("stop_count", "N/A"))),
        ("Stop ratio |v| < 0.01 m/s", pct(cmd.get("stop_ratio_abs_v_lt_0p01"))),
        ("Max angular velocity", fmt(cmd.get("abs_w", {}).get("max"), " rad/s", 3)),
        (
            "MPC solve time avg / p99",
            f"{fmt(latency.get('solve_ms', {}).get('mean'), ' ms')} / "
            f"{fmt(latency.get('solve_ms', {}).get('p99'), ' ms')}",
        ),
        (
            "E2E latency avg / p99",
            f"{fmt(latency.get('e2e_ms', {}).get('mean'), ' ms')} / "
            f"{fmt(latency.get('e2e_ms', {}).get('p99'), ' ms')}",
        ),
    ]

    lines = [
        "| Metric | Result |",
        "|---|---:|",
    ]
    lines.extend(f"| {name} | {value} |" for name, value in rows)
    lines.append("")
    lines.append("Verdict:")
    lines.extend(f"- {item}" for item in summary["verdicts"])
    lines.append("")
    return "\n".join(lines)


def print_summary(summary: Dict[str, Any], safety_threshold: float) -> None:
    print("\n" + "=" * 72)
    print("Dynamic Obstacle CBF Safety Evaluation")
    print("=" * 72)
    print(make_markdown(summary, safety_threshold))


def parse_thresholds(raw: str) -> List[float]:
    return [float(x.strip()) for x in raw.split(",") if x.strip()]


def crop_series(series: Iterable[Tuple[float, Any]], start: float, end: Optional[float]) -> List[Tuple[float, Any]]:
    cropped: List[Tuple[float, Any]] = []
    for t, value in series:
        if t < start:
            continue
        if end is not None and t > end:
            continue
        cropped.append((t - start, value))
    return cropped


def crop_cmd(cmd: CmdSeries, start: float, end: Optional[float]) -> CmdSeries:
    return [(t, v, w) for t, (v, w) in crop_series(((t, (v, w)) for t, v, w in cmd), start, end)]


def crop_latency(lat: LatencySeries, start: float, end: Optional[float]) -> LatencySeries:
    return [
        (t, solve, e2e)
        for t, (solve, e2e) in crop_series(((t, (solve, e2e)) for t, solve, e2e in lat), start, end)
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description="동적 장애물 회피 안전성 지표 산출")
    parser.add_argument("--bag", required=True, help="rosbag2 디렉터리 또는 .db3 파일")
    parser.add_argument("--log", default=None, help="선택: mpc_node 터미널 로그 txt")
    parser.add_argument(
        "--out-prefix",
        default="tools/dynamic_obstacle_safety_eval",
        help="출력 파일 prefix (.json, .md 생성)",
    )
    parser.add_argument(
        "--thresholds",
        default="0.00,0.10,0.15,0.20,0.25",
        help="이격거리 임계값 CSV [m]",
    )
    parser.add_argument(
        "--collision-threshold",
        type=float,
        default=0.0,
        help="충돌 판정 기준 [m]. 기본값: 0 이하",
    )
    parser.add_argument(
        "--safety-threshold",
        type=float,
        default=0.20,
        help="README 대표 안전거리 기준 [m]",
    )
    parser.add_argument("--w-limit", type=float, default=0.42, help="각속도 제한 [rad/s]")
    parser.add_argument("--start-sec", type=float, default=0.0, help="분석 시작 시간 [s]")
    parser.add_argument("--end-sec", type=float, default=None, help="분석 종료 시간 [s]")
    args = parser.parse_args()

    bag_path = expand(args.bag)
    log_path = expand(args.log) if args.log else None
    out_prefix = Path(args.out_prefix).expanduser()
    if not out_prefix.is_absolute():
        out_prefix = Path.cwd() / out_prefix
    thresholds = parse_thresholds(args.thresholds)
    if args.safety_threshold not in thresholds:
        thresholds.append(args.safety_threshold)
        thresholds = sorted(set(thresholds))

    cmd = parse_twist(read_raw_topic(bag_path, "/cmd_vel"))
    dist = parse_min_distance(read_raw_topic(bag_path, "/metrics/min_obstacle_distance"))
    lat = parse_latency(read_raw_topic(bag_path, "/metrics/control_latency_ms"))
    if args.start_sec > 0.0 or args.end_sec is not None:
        cmd = crop_cmd(cmd, args.start_sec, args.end_sec)
        dist = crop_series(dist, args.start_sec, args.end_sec)
        lat = crop_latency(lat, args.start_sec, args.end_sec)
    parsed_log = parse_log(log_path)

    summary: Dict[str, Any] = {
        "bag": str(bag_path),
        "log_path": str(log_path) if log_path else None,
        "analysis_window_sec": {
            "start": args.start_sec,
            "end": args.end_sec,
        },
        "clearance": calc_clearance_metrics(dist, thresholds, args.collision_threshold),
        "cmd_vel": calc_cmd_metrics(cmd, args.w_limit),
        "latency": calc_latency_metrics(lat),
        "log": calc_log_metrics(parsed_log),
    }
    summary["verdicts"] = make_verdict(summary, args.safety_threshold)

    out_prefix.parent.mkdir(parents=True, exist_ok=True)
    json_path = out_prefix.with_suffix(".json")
    md_path = out_prefix.with_suffix(".md")
    with json_path.open("w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    with md_path.open("w", encoding="utf-8") as f:
        f.write(make_markdown(summary, args.safety_threshold))

    print_summary(summary, args.safety_threshold)
    print(f"[saved] JSON: {json_path}")
    print(f"[saved] Markdown: {md_path}")

    if deserialize_message is None or MinObstacleDistance is None:
        print(
            "[warn] amr_msgs 역직렬화를 사용할 수 없어 min_distance를 fallback으로 파싱했습니다. "
            "정확도를 위해 source ~/isaac_amr_ws/install/setup.bash 후 다시 실행하세요."
        )


if __name__ == "__main__":
    main()
