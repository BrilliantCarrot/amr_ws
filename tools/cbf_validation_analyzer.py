#!/usr/bin/env python3
"""
CBF 적용 AMR 주행 검증 스크립트

[목적]
  1. 장애물 최소거리 /metrics/min_obstacle_distance 확인함
  2. cmd_vel 선속도/각속도 및 각속도 튐 확인함
  3. control latency /metrics/control_latency_ms 확인함
  4. 선택적으로 터미널 로그를 파싱해서 CBF 개입량 확인함

[사용 예시]
  python3 cbf_validation_analyzer.py \
    --bag ~/amr_ws/bags/cbf_astar_run \
    --log ~/amr_ws/logs/cbf_astar_terminal.txt \
    --out_prefix cbf_astar_eval

[주의]
  - bag 분석은 rosbag2 sqlite3 db3 기준임
  - MinObstacleDistance는 ROS2 deserialize_message로 안전하게 파싱함
  - 터미널 로그는 선택 사항임
"""

import argparse
import json
import math
import re
import sqlite3
import struct
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt

# MinObstacleDistance는 CDR raw 추측 파싱이 아니라 ROS2 역직렬화로 읽음
# 실행 전: source /opt/ros/humble/setup.bash && source ~/amr_ws/install/setup.bash
try:
    from rclpy.serialization import deserialize_message
    from amr_msgs.msg import MinObstacleDistance
except Exception:
    deserialize_message = None
    MinObstacleDistance = None


# ============================================================
# 기본 유틸
# ============================================================

NumberSeries = List[Tuple[float, float]]
CmdSeries = List[Tuple[float, float, float]]
LatencySeries = List[Tuple[float, float, float]]


def expand_path(path_str: Optional[str]) -> Optional[Path]:
    """~ 포함 경로를 Path로 변환함"""
    if not path_str:
        return None
    return Path(path_str).expanduser().resolve()


def find_db3_path(bag_path: Path) -> Path:
    """rosbag2 디렉터리에서 db3 파일 찾음"""
    if bag_path.is_file() and bag_path.suffix == ".db3":
        return bag_path

    if not bag_path.exists():
        raise FileNotFoundError(f"bag 경로가 없음: {bag_path}")

    candidates = sorted(bag_path.glob("*.db3"))
    if not candidates:
        # 기존 스크립트 방식도 한 번 시도함
        candidate = bag_path / f"{bag_path.name}_0.db3"
        if candidate.exists():
            return candidate
        raise FileNotFoundError(f"db3 파일을 찾지 못함: {bag_path}")
    return candidates[0]


def read_raw_topic(bag_path: Path, topic_name: str) -> List[Tuple[float, bytes]]:
    """SQLite3 rosbag에서 특정 토픽 raw CDR bytes 읽어옴"""
    db_path = find_db3_path(bag_path)
    conn = sqlite3.connect(str(db_path))
    cursor = conn.cursor()

    cursor.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
    row = cursor.fetchone()
    if row is None:
        conn.close()
        return []

    topic_id = row[0]
    cursor.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (topic_id,),
    )
    rows = cursor.fetchall()
    conn.close()

    return [(ts_ns * 1e-9, bytes(data)) for ts_ns, data in rows]


def normalize_rows(rows: List[Tuple[float, bytes]]) -> List[Tuple[float, bytes]]:
    """시간을 첫 샘플 기준 0초로 맞춤"""
    if not rows:
        return []
    t0 = rows[0][0]
    return [(t - t0, data) for t, data in rows]


def safe_percentile(values: np.ndarray, p: float) -> Optional[float]:
    """빈 배열에 안전한 percentile 계산함"""
    if values.size == 0:
        return None
    return float(np.percentile(values, p))


def summarize_array(values: List[float]) -> Dict[str, Optional[float]]:
    """기본 통계 계산함"""
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
        "p01": safe_percentile(arr, 1),
        "p05": safe_percentile(arr, 5),
        "p50": safe_percentile(arr, 50),
        "p95": safe_percentile(arr, 95),
        "p99": safe_percentile(arr, 99),
        "max": float(np.max(arr)),
    }


# ============================================================
# CDR 파싱
# ============================================================


def parse_twist(rows: List[Tuple[float, bytes]]) -> CmdSeries:
    """geometry_msgs/Twist CDR → (t, v, omega)"""
    result: CmdSeries = []
    for t, data in normalize_rows(rows):
        try:
            # CDR encapsulation 4 bytes 이후 float64 6개임
            vals = struct.unpack_from("<6d", data, 4)
            result.append((t, vals[0], vals[5]))
        except Exception:
            continue
    return result


def parse_latency(rows: List[Tuple[float, bytes]]) -> LatencySeries:
    """ControlLatency CDR → (t, solve_ms, e2e_ms)"""
    result: LatencySeries = []
    for t, data in normalize_rows(rows):
        try:
            # 기존 compare_trajectory.py 방식: 마지막 6개 double 중 0, 3 사용함
            vals = struct.unpack_from("<6d", data, len(data) - 48)
            result.append((t, vals[0], vals[3]))
        except Exception:
            continue
    return result


def parse_pose_rmse(rows: List[Tuple[float, bytes]]) -> NumberSeries:
    """PoseRmse CDR → (t, rmse_total)"""
    result: NumberSeries = []
    for t, data in normalize_rows(rows):
        try:
            vals = struct.unpack_from("<4d", data, len(data) - 32)
            result.append((t, vals[3]))
        except Exception:
            continue
    return result


def parse_min_obstacle_distance(rows: List[Tuple[float, bytes]]) -> NumberSeries:
    """MinObstacleDistance CDR → (t, min_distance_m)

    monte_carlo.py처럼 amr_msgs.msg.MinObstacleDistance의
    msg.min_distance_m 필드를 직접 사용함.
    raw struct.unpack으로 필드 위치를 추측하지 않음.
    """
    result: NumberSeries = []

    if not rows:
        return result

    if deserialize_message is None or MinObstacleDistance is None:
        raise RuntimeError(
            "MinObstacleDistance 역직렬화 실패: ROS2 환경을 source 했는지 확인하세요.\n"
            "예: source /opt/ros/humble/setup.bash && source ~/amr_ws/install/setup.bash"
        )

    t0 = rows[0][0]
    for ts, data in rows:
        try:
            msg = deserialize_message(data, MinObstacleDistance)
            val = float(msg.min_distance_m)

            # monte_carlo.py의 collision 판정은 음수 clearance도 의미 있게 사용함.
            # 따라서 0 이상 필터를 걸지 않고, 미측정 sentinel만 제외함.
            if math.isfinite(val) and abs(val) < 900.0:
                result.append((ts - t0, val))
        except Exception:
            continue

    return result


# ============================================================
# 터미널 로그 파싱
# ============================================================

CBF_RESULT_RE = re.compile(
    r"\[CBF result\].*?ok=(?P<ok>true|false).*?"
    r"u_nom=\((?P<v_nom>[-+0-9.eE]+),\s*(?P<w_nom>[-+0-9.eE]+)\)\s*"
    r"u_safe=\((?P<v_safe>[-+0-9.eE]+),\s*(?P<w_safe>[-+0-9.eE]+)\)"
)

MPC_SUMMARY_RE = re.compile(
    r"\[MPC\].*?solve=(?P<solve>[-+0-9.eE]+)ms.*?"
    r"e2e=(?P<e2e>[-+0-9.eE]+)ms.*?"
    r"u_nom=\((?P<v_nom>[-+0-9.eE]+),\s*(?P<w_nom>[-+0-9.eE]+)\)\s*"
    r"u_pub=\((?P<v_pub>[-+0-9.eE]+),\s*(?P<w_pub>[-+0-9.eE]+)\)"
)

CBF_PRE_RE = re.compile(
    r"\[CBF dbg pre\]\s*N=(?P<N>\d+)\s+"
    r"clr=(?P<clr>[-+0-9.eE]+)\s+"
    r"fwd=(?P<fwd>[-+0-9.eE]+)\s+"
    r"h=(?P<h>[-+0-9.eE]+)\s+"
    r"av=(?P<av>[-+0-9.eE]+)\s+"
    r"aw=(?P<aw>[-+0-9.eE]+)\s+"
    r"lhs_nom=(?P<lhs>[-+0-9.eE]+)\s+"
    r"rhs=(?P<rhs>[-+0-9.eE]+)\s+"
    r"v_nom=(?P<v_nom>[-+0-9.eE]+)\s+"
    r"w_nom=(?P<w_nom>[-+0-9.eE]+)"
)

MISSION_DONE_RE = re.compile(r"미션 완료|목표 지점 도달|도킹 완료|Mission complete|goal reached", re.IGNORECASE)


def parse_log_file(log_path: Optional[Path]) -> Dict[str, List[Dict[str, float]]]:
    """터미널 로그에서 CBF/MPC 관련 수치 파싱함"""
    parsed = {
        "cbf_result": [],
        "mpc_summary": [],
        "cbf_pre": [],
        "mission_done": [],
    }
    if not log_path:
        return parsed
    if not log_path.exists():
        raise FileNotFoundError(f"로그 파일이 없음: {log_path}")

    with log_path.open("r", encoding="utf-8", errors="ignore") as f:
        for line_idx, line in enumerate(f):
            m = CBF_RESULT_RE.search(line)
            if m:
                item = {"idx": line_idx, "ok": 1.0 if m.group("ok") == "true" else 0.0}
                for key in ["v_nom", "w_nom", "v_safe", "w_safe"]:
                    item[key] = float(m.group(key))
                parsed["cbf_result"].append(item)
                continue

            m = MPC_SUMMARY_RE.search(line)
            if m:
                item = {"idx": line_idx}
                for key in ["solve", "e2e", "v_nom", "w_nom", "v_pub", "w_pub"]:
                    item[key] = float(m.group(key))
                parsed["mpc_summary"].append(item)
                continue

            m = CBF_PRE_RE.search(line)
            if m:
                item = {"idx": line_idx}
                item["N"] = float(m.group("N"))
                for key in ["clr", "fwd", "h", "av", "aw", "lhs", "rhs", "v_nom", "w_nom"]:
                    item[key] = float(m.group(key))
                parsed["cbf_pre"].append(item)
                continue

            if MISSION_DONE_RE.search(line):
                parsed["mission_done"].append({"idx": float(line_idx)})

    return parsed


# ============================================================
# 지표 계산
# ============================================================


def calc_min_distance_metrics(dist: NumberSeries, thresholds: List[float]) -> Dict[str, object]:
    """장애물 최소거리 지표 계산함"""
    values = [x[1] for x in dist]
    stats = summarize_array(values)
    result: Dict[str, object] = {"stats": stats, "thresholds": {}}
    if not values:
        return result

    arr = np.asarray(values, dtype=float)
    for th in thresholds:
        result["thresholds"][str(th)] = {
            "samples_below": int(np.sum(arr < th)),
            "ratio_below": float(np.sum(arr < th) / arr.size),
        }
    return result


def calc_cmd_metrics(cmd: CmdSeries, w_limit: float) -> Dict[str, object]:
    """cmd_vel 기반 제어 품질 지표 계산함"""
    if len(cmd) < 2:
        return {}

    t = np.asarray([x[0] for x in cmd], dtype=float)
    v = np.asarray([x[1] for x in cmd], dtype=float)
    w = np.asarray([x[2] for x in cmd], dtype=float)
    dv = np.diff(v)
    dw = np.diff(w)
    dt = np.diff(t)
    valid_dt = dt[dt > 1e-9]
    duration = float(t[-1] - t[0]) if len(t) >= 2 else 0.0

    # 부호 반전은 작은 각속도 구간 제외하고 계산함
    w_deadband = 0.02
    w_active = w[np.abs(w) > w_deadband]
    sign_flips = 0
    if w_active.size >= 2:
        sign_flips = int(np.sum(np.sign(w_active[1:]) != np.sign(w_active[:-1])))

    return {
        "duration_s": duration,
        "sample_count": int(len(cmd)),
        "dt_mean_s": float(np.mean(valid_dt)) if valid_dt.size else None,
        "v": summarize_array(v.tolist()),
        "w": summarize_array(w.tolist()),
        "abs_w": summarize_array(np.abs(w).tolist()),
        "dv": summarize_array(dv.tolist()),
        "dw": summarize_array(dw.tolist()),
        "stop_ratio_v_lt_0p01": float(np.mean(np.abs(v) < 0.01)),
        "slow_ratio_v_lt_0p03": float(np.mean(np.abs(v) < 0.03)),
        "clip_ratio_abs_w_ge_limit": float(np.mean(np.abs(w) >= 0.999 * w_limit)),
        "w_sign_flip_count": sign_flips,
        "w_sign_flip_rate_per_s": float(sign_flips / duration) if duration > 1e-9 else None,
    }


def calc_latency_metrics(lat: LatencySeries) -> Dict[str, object]:
    """latency 지표 계산함"""
    if not lat:
        return {}
    solve = [x[1] for x in lat]
    e2e = [x[2] for x in lat]
    return {
        "solve_ms": summarize_array(solve),
        "e2e_ms": summarize_array(e2e),
        "e2e_over_20ms_ratio": float(np.mean(np.asarray(e2e) > 20.0)),
        "solve_over_5ms_ratio": float(np.mean(np.asarray(solve) > 5.0)),
    }


def calc_rmse_metrics(rmse: NumberSeries) -> Dict[str, object]:
    """tracking RMSE 지표 계산함"""
    if not rmse:
        return {}
    values = [x[1] for x in rmse]
    return {"rmse_total_m": summarize_array(values)}


def calc_log_cbf_metrics(parsed_log: Dict[str, List[Dict[str, float]]]) -> Dict[str, object]:
    """터미널 로그 기반 CBF 개입 지표 계산함"""
    result: Dict[str, object] = {}

    cbf_res = parsed_log.get("cbf_result", [])
    if cbf_res:
        dv = np.asarray([abs(x["v_safe"] - x["v_nom"]) for x in cbf_res], dtype=float)
        dw = np.asarray([abs(x["w_safe"] - x["w_nom"]) for x in cbf_res], dtype=float)
        result["cbf_result"] = {
            "count": int(len(cbf_res)),
            "ok_ratio": float(np.mean([x["ok"] for x in cbf_res])),
            "abs_delta_v": summarize_array(dv.tolist()),
            "abs_delta_w": summarize_array(dw.tolist()),
            "intervention_ratio_delta_v_gt_0p01_or_delta_w_gt_0p03": float(
                np.mean((dv > 0.01) | (dw > 0.03))
            ),
            "strong_intervention_ratio_delta_w_gt_0p2": float(np.mean(dw > 0.2)),
            "v_safe_above_v_nom_ratio": float(
                np.mean([x["v_safe"] > x["v_nom"] + 1e-6 for x in cbf_res])
            ),
        }

    mpc = parsed_log.get("mpc_summary", [])
    if mpc:
        dv = np.asarray([abs(x["v_pub"] - x["v_nom"]) for x in mpc], dtype=float)
        dw = np.asarray([abs(x["w_pub"] - x["w_nom"]) for x in mpc], dtype=float)
        result["mpc_summary"] = {
            "count": int(len(mpc)),
            "abs_delta_v_pub_nom": summarize_array(dv.tolist()),
            "abs_delta_w_pub_nom": summarize_array(dw.tolist()),
            "intervention_ratio_delta_v_gt_0p01_or_delta_w_gt_0p03": float(
                np.mean((dv > 0.01) | (dw > 0.03))
            ),
        }

    pre = parsed_log.get("cbf_pre", [])
    if pre:
        clr = np.asarray([x["clr"] for x in pre], dtype=float)
        fwd = np.asarray([x["fwd"] for x in pre], dtype=float)
        lhs = np.asarray([x["lhs"] for x in pre], dtype=float)
        rhs = np.asarray([x["rhs"] for x in pre], dtype=float)
        N = np.asarray([x["N"] for x in pre], dtype=float)
        result["cbf_pre"] = {
            "count": int(len(pre)),
            "active_obstacle_N": summarize_array(N.tolist()),
            "clearance": summarize_array(clr.tolist()),
            "forward_projection": summarize_array(fwd.tolist()),
            "nominal_violation_ratio_lhs_lt_rhs": float(np.mean(lhs < rhs)),
            "behind_active_ratio_fwd_lt_0": float(np.mean(fwd < 0.0)),
            "very_close_ratio_clearance_lt_0": float(np.mean(clr < 0.0)),
        }

    result["mission_done_detected_in_log"] = bool(parsed_log.get("mission_done"))
    return result


# ============================================================
# 판정 로직
# ============================================================


def make_verdict(summary: Dict[str, object], safety_threshold: float, w_limit: float) -> List[str]:
    """사람이 읽기 쉬운 판정 문장 생성함"""
    verdicts: List[str] = []

    dist_metrics = summary.get("min_obstacle_distance", {})
    dist_stats = dist_metrics.get("stats", {}) if isinstance(dist_metrics, dict) else {}
    min_dist = dist_stats.get("min") if isinstance(dist_stats, dict) else None
    if min_dist is not None:
        if min_dist >= safety_threshold:
            verdicts.append(f"PASS: 최소 장애물 거리 {min_dist:.3f} m >= 기준 {safety_threshold:.3f} m")
        else:
            verdicts.append(f"WARN: 최소 장애물 거리 {min_dist:.3f} m < 기준 {safety_threshold:.3f} m")
    else:
        verdicts.append("INFO: /metrics/min_obstacle_distance 데이터 없음")

    cmd = summary.get("cmd_vel", {})
    if isinstance(cmd, dict) and cmd:
        stop_ratio = cmd.get("stop_ratio_v_lt_0p01")
        slow_ratio = cmd.get("slow_ratio_v_lt_0p03")
        clip_ratio = cmd.get("clip_ratio_abs_w_ge_limit")
        dw_stats = cmd.get("dw", {})
        max_dw = dw_stats.get("max") if isinstance(dw_stats, dict) else None
        flip_rate = cmd.get("w_sign_flip_rate_per_s")

        if stop_ratio is not None:
            if stop_ratio < 0.02:
                verdicts.append(f"PASS: 정지 비율(|v|<0.01) {stop_ratio*100:.1f}%로 낮음")
            else:
                verdicts.append(f"WARN: 정지 비율(|v|<0.01) {stop_ratio*100:.1f}%가 큼")
        if slow_ratio is not None and slow_ratio > 0.2:
            verdicts.append(f"CHECK: 저속 비율(|v|<0.03) {slow_ratio*100:.1f}% — 경로/CBF 개입 구간 확인 필요")
        if clip_ratio is not None:
            if clip_ratio < 0.01:
                verdicts.append(f"PASS: |ω| 한계 {w_limit:.2f} rad/s 클리핑 비율 {clip_ratio*100:.1f}%")
            else:
                verdicts.append(f"WARN: |ω| 클리핑 비율 {clip_ratio*100:.1f}%")
        if max_dw is not None and max_dw > 0.4:
            verdicts.append(f"WARN: 최대 Δω={max_dw:.3f} rad/s 로 큼. 끊김 가능성 있음")
        if flip_rate is not None and flip_rate > 1.0:
            verdicts.append(f"WARN: 각속도 부호 전환율 {flip_rate:.2f}/s 로 큼. 와리가리 가능성 있음")

    lat = summary.get("latency", {})
    if isinstance(lat, dict) and lat:
        e2e = lat.get("e2e_ms", {})
        e2e_p99 = e2e.get("p99") if isinstance(e2e, dict) else None
        if e2e_p99 is not None:
            if e2e_p99 <= 25.0:
                verdicts.append(f"PASS: e2e latency p99={e2e_p99:.2f} ms")
            else:
                verdicts.append(f"WARN: e2e latency p99={e2e_p99:.2f} ms, 50Hz 기준 부담 있음")

    cbf = summary.get("cbf_log", {})
    if isinstance(cbf, dict) and cbf:
        cbf_result = cbf.get("cbf_result", {})
        if isinstance(cbf_result, dict) and cbf_result:
            intervention = cbf_result.get("intervention_ratio_delta_v_gt_0p01_or_delta_w_gt_0p03")
            strong = cbf_result.get("strong_intervention_ratio_delta_w_gt_0p2")
            if intervention is not None:
                if intervention < 0.4:
                    verdicts.append(f"PASS: CBF 개입 비율 {intervention*100:.1f}% — 대부분 nominal 유지")
                else:
                    verdicts.append(f"CHECK: CBF 개입 비율 {intervention*100:.1f}% — 필터가 자주 개입함")
            if strong is not None and strong > 0.2:
                verdicts.append(f"WARN: 강한 CBF 각속도 보정(|Δω|>0.2) {strong*100:.1f}%")

        cbf_pre = cbf.get("cbf_pre", {})
        if isinstance(cbf_pre, dict) and cbf_pre:
            behind = cbf_pre.get("behind_active_ratio_fwd_lt_0")
            if behind is not None and behind > 0.5:
                verdicts.append(f"CHECK: CBF active 중 fwd<0 비율 {behind*100:.1f}% — 뒤쪽 장애물이 오래 남는지 확인")

        if cbf.get("mission_done_detected_in_log"):
            verdicts.append("PASS: 터미널 로그에서 목표 도달/미션 완료 감지됨")

    return verdicts


# ============================================================
# 출력/그래프
# ============================================================


def print_summary(summary: Dict[str, object], verdicts: List[str]) -> None:
    """터미널 요약 출력함"""
    print("\n" + "=" * 78)
    print("  CBF 주행 검증 결과")
    print("=" * 78)

    def stat_line(name: str, stats: Dict[str, object], unit: str = "") -> None:
        if not stats or stats.get("count", 0) == 0:
            print(f"  {name:<26}: N/A")
            return
        print(
            f"  {name:<26}: "
            f"mean={stats.get('mean'):.4f}{unit}, "
            f"min={stats.get('min'):.4f}{unit}, "
            f"p05={stats.get('p05'):.4f}{unit}, "
            f"p95={stats.get('p95'):.4f}{unit}, "
            f"max={stats.get('max'):.4f}{unit}"
        )

    dist = summary.get("min_obstacle_distance", {})
    if isinstance(dist, dict):
        stat_line("min obstacle distance", dist.get("stats", {}), "m")

    cmd = summary.get("cmd_vel", {})
    if isinstance(cmd, dict) and cmd:
        print("-" * 78)
        print(f"  duration              : {cmd.get('duration_s', 0.0):.2f}s")
        stat_line("linear v", cmd.get("v", {}), "m/s")
        stat_line("angular w", cmd.get("w", {}), "rad/s")
        stat_line("delta v", cmd.get("dv", {}), "m/s")
        stat_line("delta w", cmd.get("dw", {}), "rad/s")
        print(f"  stop ratio |v|<0.01  : {cmd.get('stop_ratio_v_lt_0p01', 0.0)*100:.2f}%")
        print(f"  slow ratio |v|<0.03  : {cmd.get('slow_ratio_v_lt_0p03', 0.0)*100:.2f}%")
        print(f"  omega sign flips      : {cmd.get('w_sign_flip_count', 0)}")

    lat = summary.get("latency", {})
    if isinstance(lat, dict) and lat:
        print("-" * 78)
        stat_line("solve latency", lat.get("solve_ms", {}), "ms")
        stat_line("e2e latency", lat.get("e2e_ms", {}), "ms")

    cbf = summary.get("cbf_log", {})
    if isinstance(cbf, dict) and cbf:
        print("-" * 78)
        res = cbf.get("cbf_result", {})
        pre = cbf.get("cbf_pre", {})
        if isinstance(res, dict) and res:
            print(f"  CBF result samples    : {res.get('count', 0)}")
            print(f"  CBF ok ratio          : {res.get('ok_ratio', 0.0)*100:.2f}%")
            print(f"  CBF intervention ratio: {res.get('intervention_ratio_delta_v_gt_0p01_or_delta_w_gt_0p03', 0.0)*100:.2f}%")
            stat_line("CBF |delta v|", res.get("abs_delta_v", {}), "m/s")
            stat_line("CBF |delta w|", res.get("abs_delta_w", {}), "rad/s")
        if isinstance(pre, dict) and pre:
            stat_line("CBF clearance", pre.get("clearance", {}), "m")
            stat_line("CBF fwd", pre.get("forward_projection", {}), "m")
            print(f"  nominal violation ratio: {pre.get('nominal_violation_ratio_lhs_lt_rhs', 0.0)*100:.2f}%")
            print(f"  active fwd<0 ratio     : {pre.get('behind_active_ratio_fwd_lt_0', 0.0)*100:.2f}%")

    print("-" * 78)
    print("  판정")
    for v in verdicts:
        print(f"  - {v}")
    print("=" * 78 + "\n")


def plot_results(
    dist: NumberSeries,
    cmd: CmdSeries,
    lat: LatencySeries,
    parsed_log: Dict[str, List[Dict[str, float]]],
    out_png: Path,
    safety_threshold: float,
    w_limit: float,
) -> None:
    """검증 그래프 저장함"""
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle("CBF Validation Report", fontsize=14, fontweight="bold")

    # 1. 장애물 최소거리
    ax = axes[0, 0]
    if dist:
        t = [x[0] for x in dist]
        y = [x[1] for x in dist]
        ax.plot(t, y, linewidth=1.1, label="min obstacle distance")
        ax.axhline(safety_threshold, linestyle="--", linewidth=1.0, label="safety threshold")
        ax.set_title("Minimum obstacle distance")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("distance [m]")
        ax.legend(fontsize=8)
    else:
        ax.text(0.5, 0.5, "No /metrics/min_obstacle_distance", ha="center", va="center")
    ax.grid(True, alpha=0.3)

    # 2. cmd_vel v
    ax = axes[0, 1]
    if cmd:
        t = [x[0] for x in cmd]
        v = [x[1] for x in cmd]
        ax.plot(t, v, linewidth=1.1, label="v")
        ax.axhline(0.01, linestyle="--", linewidth=0.8, label="stop threshold")
        ax.set_title("Linear velocity")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("v [m/s]")
        ax.legend(fontsize=8)
    else:
        ax.text(0.5, 0.5, "No /cmd_vel", ha="center", va="center")
    ax.grid(True, alpha=0.3)

    # 3. cmd_vel omega
    ax = axes[1, 0]
    if cmd:
        t = [x[0] for x in cmd]
        w = [x[2] for x in cmd]
        ax.plot(t, w, linewidth=1.1, label="omega")
        ax.axhline(w_limit, linestyle="--", linewidth=0.8)
        ax.axhline(-w_limit, linestyle="--", linewidth=0.8)
        ax.set_title("Angular velocity")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("omega [rad/s]")
        ax.legend(fontsize=8)
    else:
        ax.text(0.5, 0.5, "No /cmd_vel", ha="center", va="center")
    ax.grid(True, alpha=0.3)

    # 4. delta omega
    ax = axes[1, 1]
    if cmd and len(cmd) >= 2:
        t = np.asarray([x[0] for x in cmd])[1:]
        w = np.asarray([x[2] for x in cmd])
        dw = np.diff(w)
        ax.plot(t, dw, linewidth=0.9, label="delta omega")
        ax.set_title("Angular velocity jump")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("delta omega [rad/s]")
        ax.legend(fontsize=8)
    else:
        ax.text(0.5, 0.5, "No enough /cmd_vel", ha="center", va="center")
    ax.grid(True, alpha=0.3)

    # 5. latency
    ax = axes[2, 0]
    if lat:
        t = [x[0] for x in lat]
        solve = [x[1] for x in lat]
        e2e = [x[2] for x in lat]
        ax.plot(t, solve, linewidth=0.9, label="solve ms")
        ax.plot(t, e2e, linewidth=0.9, label="e2e ms")
        ax.axhline(20.0, linestyle="--", linewidth=0.8, label="20 ms")
        ax.set_title("Latency")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("ms")
        ax.legend(fontsize=8)
    else:
        ax.text(0.5, 0.5, "No /metrics/control_latency_ms", ha="center", va="center")
    ax.grid(True, alpha=0.3)

    # 6. CBF intervention from log
    ax = axes[2, 1]
    cbf_res = parsed_log.get("cbf_result", [])
    if cbf_res:
        idx = np.arange(len(cbf_res))
        dv = [abs(x["v_safe"] - x["v_nom"]) for x in cbf_res]
        dw = [abs(x["w_safe"] - x["w_nom"]) for x in cbf_res]
        ax.plot(idx, dv, linewidth=0.9, label="|delta v|")
        ax.plot(idx, dw, linewidth=0.9, label="|delta omega|")
        ax.axhline(0.03, linestyle="--", linewidth=0.8, label="small intervention")
        ax.set_title("CBF intervention from terminal log")
        ax.set_xlabel("CBF result sample index")
        ax.set_ylabel("absolute difference")
        ax.legend(fontsize=8)
    else:
        ax.text(0.5, 0.5, "No CBF result log\n(pass --log to analyze)", ha="center", va="center")
    ax.grid(True, alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    plt.close(fig)


# ============================================================
# 메인
# ============================================================


def main() -> None:
    parser = argparse.ArgumentParser(description="CBF 적용 AMR 주행 검증")
    parser.add_argument("--bag", required=True, help="rosbag2 디렉터리 또는 db3 파일 경로")
    parser.add_argument("--log", default=None, help="선택: mpc_node 터미널 출력 저장 txt")
    parser.add_argument("--out_prefix", default="cbf_validation", help="출력 파일 prefix")
    parser.add_argument("--safety_threshold", type=float, default=0.20, help="장애물 최소거리 PASS 기준 [m]")
    parser.add_argument("--distance_thresholds", default="0.10,0.15,0.20,0.25", help="거리 임계값 CSV [m]")
    parser.add_argument("--w_limit", type=float, default=1.0, help="각속도 제한 [rad/s]")
    args = parser.parse_args()

    bag_path = expand_path(args.bag)
    log_path = expand_path(args.log)
    out_prefix = Path(args.out_prefix).expanduser()
    thresholds = [float(x.strip()) for x in args.distance_thresholds.split(",") if x.strip()]

    print("\n[CBF 검증 시작]")
    print(f"  bag: {bag_path}")
    if log_path:
        print(f"  log: {log_path}")

    # bag 토픽 로드함
    cmd = parse_twist(read_raw_topic(bag_path, "/cmd_vel"))
    lat = parse_latency(read_raw_topic(bag_path, "/metrics/control_latency_ms"))
    dist = parse_min_obstacle_distance(read_raw_topic(bag_path, "/metrics/min_obstacle_distance"))
    rmse = parse_pose_rmse(read_raw_topic(bag_path, "/metrics/tracking_rmse"))

    # 로그 로드함
    parsed_log = parse_log_file(log_path)

    summary: Dict[str, object] = {
        "bag": str(bag_path),
        "log": str(log_path) if log_path else None,
        "min_obstacle_distance": calc_min_distance_metrics(dist, thresholds),
        "cmd_vel": calc_cmd_metrics(cmd, args.w_limit),
        "latency": calc_latency_metrics(lat),
        "tracking_rmse": calc_rmse_metrics(rmse),
        "cbf_log": calc_log_cbf_metrics(parsed_log),
    }
    verdicts = make_verdict(summary, args.safety_threshold, args.w_limit)
    summary["verdicts"] = verdicts

    # 출력 저장함
    out_json = out_prefix.with_suffix(".json")
    out_png = out_prefix.with_suffix(".png")
    with out_json.open("w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    plot_results(dist, cmd, lat, parsed_log, out_png, args.safety_threshold, args.w_limit)
    print_summary(summary, verdicts)

    print(f"[저장 완료] JSON: {out_json}")
    print(f"[저장 완료] PNG : {out_png}")


if __name__ == "__main__":
    main()