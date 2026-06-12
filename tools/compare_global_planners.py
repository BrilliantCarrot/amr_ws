#!/usr/bin/env python3
"""
A* vs RRT* global planner comparison from ROS2 bags.

Recommended bag topics:
  /planned_path
  /cmd_vel
  /map_ekf/odom
  /world/simple_room/dynamic_pose/info
  /metrics/tracking_rmse
  /metrics/control_latency_ms
  /metrics/min_obstacle_distance
  /safety/state
  /mpc/mission_phase

Optional log files:
  Pass terminal logs with --astar-log / --rrt-log to extract planner compute time
  from PathPlanner messages such as:
    [PathPlanner] RRT* 완료: raw=... | 계획시간=...

how to use:
  python3 tools/compare_global_planners.py \
  --astar-bag /home/lyj/amr_ws/bags/planner_astar \
  --rrt-bag /home/lyj/amr_ws/bags/planner_rrt_star \
  --astar-log /home/lyj/astar_result.txt \
  --rrt-log /home/lyj/rrt_result.txt \
  --out-prefix /home/lyj/amr_ws/planner_compare    
"""

import argparse
import csv
import math
import os
import re
import sqlite3
import warnings
from pathlib import Path
from statistics import mean

import numpy as np
from rclpy.serialization import deserialize_message

from amr_msgs.msg import ControlLatency, MinObstacleDistance, PoseRmse, SafetyStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


TOPICS = {
    "path": ("/planned_path", NavPath),
    "cmd": ("/cmd_vel", Twist),
    "rmse": ("/metrics/tracking_rmse", PoseRmse),
    "latency": ("/metrics/control_latency_ms", ControlLatency),
    "min_dist": ("/metrics/min_obstacle_distance", MinObstacleDistance),
    "safety": ("/safety/state", SafetyStatus),
    "mission": ("/mpc/mission_phase", String),
    "odom": ("/map_ekf/odom", Odometry),
    "gt": ("/world/simple_room/dynamic_pose/info", TFMessage),
}


def find_db3(bag_path: str) -> Path:
    path = Path(bag_path).expanduser()
    if path.is_file() and path.suffix == ".db3":
        return path
    candidates = sorted(path.glob("*.db3"))
    if not candidates:
        raise FileNotFoundError(f"No .db3 file found in bag path: {bag_path}")
    return candidates[0]


def read_topic(bag_path: str, topic_name: str, msg_type):
    db_path = find_db3(bag_path)
    conn = sqlite3.connect(str(db_path))
    cur = conn.cursor()
    cur.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
    row = cur.fetchone()
    if row is None:
        conn.close()
        return []

    topic_id = row[0]
    cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (topic_id,),
    )
    rows = cur.fetchall()
    conn.close()

    out = []
    for ts_ns, data in rows:
        try:
            out.append((ts_ns * 1e-9, deserialize_message(data, msg_type)))
        except Exception:
            continue
    return out


def normalize(data, t0):
    return [(t - t0, msg) for t, msg in data]


def choose_t0(raw, preferred_keys=None):
    keys = preferred_keys or raw.keys()
    times = []
    for key in keys:
        times.extend([t for t, _ in raw.get(key, [])])
    if not times:
        for value in raw.values():
            times.extend([t for t, _ in value])
    return min(times) if times else 0.0


def path_length(path_msg: NavPath) -> float:
    pts = path_msg.poses
    if len(pts) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        ax = a.pose.position.x
        ay = a.pose.position.y
        bx = b.pose.position.x
        by = b.pose.position.y
        total += math.hypot(bx - ax, by - ay)
    return total


def path_direct_distance(path_msg: NavPath) -> float:
    pts = path_msg.poses
    if len(pts) < 2:
        return 0.0
    sx = pts[0].pose.position.x
    sy = pts[0].pose.position.y
    gx = pts[-1].pose.position.x
    gy = pts[-1].pose.position.y
    return math.hypot(gx - sx, gy - sy)


def pct(values, percentile):
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=float), percentile))


def safe_mean(values):
    return float(mean(values)) if values else None


def parse_planner_log(log_path: str, planner_label: str):
    if not log_path:
        return {}
    path = Path(log_path).expanduser()
    if not path.exists():
        return {}

    text = path.read_text(errors="ignore")
    if planner_label == "astar":
        pattern = re.compile(r"A\* 완료: raw=(\d+) pts \| 계획시간=([0-9.]+)ms")
    else:
        pattern = re.compile(r"RRT\* 완료: raw=(\d+) pts \| 계획시간=([0-9.]+)ms")

    raw_counts = []
    plan_ms = []
    for raw, ms in pattern.findall(text):
        raw_counts.append(int(raw))
        plan_ms.append(float(ms))

    if not plan_ms:
        return {}

    return {
        "planner_log_count": len(plan_ms),
        "planner_time_mean_ms": safe_mean(plan_ms),
        "planner_time_p95_ms": pct(plan_ms, 95),
        "planner_time_max_ms": max(plan_ms),
        "planner_raw_pts_mean": safe_mean(raw_counts),
    }


def analyze_bag(label: str, bag_path: str, log_path: str = None):
    raw = {}
    for key, (topic, msg_type) in TOPICS.items():
        data = read_topic(bag_path, topic, msg_type)
        raw[key] = data

    t0 = choose_t0(raw, ["path", "cmd", "mission", "rmse", "latency", "min_dist", "safety"])
    data = {key: normalize(value, t0) for key, value in raw.items()}

    stats = {
        "label": label,
        "bag": bag_path,
    }

    paths = [msg for _, msg in data["path"]]
    if paths:
        lengths = [path_length(p) for p in paths]
        wps = [len(p.poses) for p in paths]
        direct = path_direct_distance(paths[0])
        stats.update({
            "path_publish_count": len(paths),
            "initial_path_length_m": lengths[0],
            "final_path_length_m": lengths[-1],
            "path_length_mean_m": safe_mean(lengths),
            "initial_waypoints": wps[0],
            "final_waypoints": wps[-1],
            "direct_distance_m": direct,
            "initial_directness": (lengths[0] / direct) if direct > 1e-9 else None,
        })
    else:
        stats.update({
            "path_publish_count": 0,
            "initial_path_length_m": None,
            "final_path_length_m": None,
            "path_length_mean_m": None,
            "initial_waypoints": None,
            "final_waypoints": None,
            "direct_distance_m": None,
            "initial_directness": None,
        })

    cmd = data["cmd"]
    if cmd:
        times = [t for t, _ in cmd]
        vs = np.asarray([msg.linear.x for _, msg in cmd], dtype=float)
        ws = np.asarray([msg.angular.z for _, msg in cmd], dtype=float)
        dw = np.diff(ws) if len(ws) >= 2 else np.asarray([], dtype=float)
        duration = max(times) - min(times) if len(times) >= 2 else 0.0
        stats.update({
            "cmd_duration_s": duration,
            "v_mean_mps": float(np.mean(vs)),
            "v_max_mps": float(np.max(vs)),
            "stop_ratio_pct": float(np.mean(np.abs(vs) < 0.01) * 100.0),
            "w_abs_mean_radps": float(np.mean(np.abs(ws))),
            "w_abs_max_radps": float(np.max(np.abs(ws))),
            "w_clip_ratio_pct": float(np.mean(np.abs(ws) >= 0.999) * 100.0),
            "dw_std_radps": float(np.std(dw)) if dw.size else 0.0,
        })

    rmse = [msg.rmse_total for _, msg in data["rmse"]]
    if rmse:
        stats.update({
            "rmse_mean_m": safe_mean(rmse),
            "rmse_max_m": max(rmse),
            "rmse_final_m": rmse[-1],
        })

    lat = data["latency"]
    if lat:
        solve = [msg.latency_ms for _, msg in lat]
        e2e = [msg.e2e_latency_ms for _, msg in lat]
        stats.update({
            "solve_mean_ms": safe_mean(solve),
            "solve_p99_ms": pct(solve, 99),
            "e2e_mean_ms": safe_mean(e2e),
            "e2e_p99_ms": pct(e2e, 99),
        })

    dist = [msg.min_distance_m for _, msg in data["min_dist"] if msg.min_distance_m < 1e8]
    if dist:
        stats.update({
            "min_clearance_m": min(dist),
            "clearance_mean_m": safe_mean(dist),
        })

    safety = data["safety"]
    if safety:
        states = [msg.state_name for _, msg in safety]
        stats.update({
            "safe_stop_samples": sum(1 for s in states if s == "SAFE_STOP"),
            "degraded_samples": sum(1 for s in states if s == "DEGRADED"),
            "manual_samples": sum(1 for s in states if s == "MANUAL_OVERRIDE"),
        })

    mission = data["mission"]
    if mission:
        first_docking = next((t for t, msg in mission if msg.data == "DOCKING"), None)
        first_docked = next((t for t, msg in mission if msg.data == "DOCKED"), None)
        stats.update({
            "time_to_docking_s": first_docking,
            "time_to_docked_s": first_docked,
            "docked": first_docked is not None,
        })

    planner_hint = "astar" if label.lower().startswith("a") else "rrt_star"
    stats.update(parse_planner_log(log_path, planner_hint))
    return stats


FIELDS = [
    ("path_publish_count", "path publications", ""),
    ("initial_path_length_m", "initial path length", "m"),
    ("initial_directness", "initial path/direct distance", ""),
    ("initial_waypoints", "initial waypoints", ""),
    ("planner_time_mean_ms", "planner time mean", "ms"),
    ("planner_time_p95_ms", "planner time p95", "ms"),
    ("planner_time_max_ms", "planner time max", "ms"),
    ("cmd_duration_s", "cmd duration", "s"),
    ("time_to_docking_s", "time to DOCKING", "s"),
    ("time_to_docked_s", "time to DOCKED", "s"),
    ("rmse_mean_m", "tracking RMSE mean", "m"),
    ("rmse_max_m", "tracking RMSE max", "m"),
    ("min_clearance_m", "minimum obstacle clearance", "m"),
    ("v_mean_mps", "mean linear speed", "m/s"),
    ("w_abs_mean_radps", "mean |omega|", "rad/s"),
    ("dw_std_radps", "omega change std", "rad/s"),
    ("w_clip_ratio_pct", "|omega| clip ratio", "%"),
    ("stop_ratio_pct", "stop ratio", "%"),
    ("solve_mean_ms", "control solve mean", "ms"),
    ("e2e_p99_ms", "e2e latency p99", "ms"),
    ("safe_stop_samples", "SAFE_STOP samples", ""),
    ("docked", "docked", ""),
]


def fmt(value):
    if value is None:
        return "N/A"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def print_table(astar, rrt):
    print("\nA* vs RRT* global planner comparison")
    print("=" * 88)
    print(f"{'metric':<32} {'A*':>18} {'RRT*':>18} {'unit':>10}")
    print("-" * 88)
    for key, label, unit in FIELDS:
        print(f"{label:<32} {fmt(astar.get(key)):>18} {fmt(rrt.get(key)):>18} {unit:>10}")
    print("=" * 88)


def write_csv(out_prefix: str, astar, rrt):
    path = Path(out_prefix + ".csv")
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["metric", "astar", "rrt_star", "unit"])
        for key, label, unit in FIELDS:
            writer.writerow([label, astar.get(key), rrt.get(key), unit])
    return path


def write_markdown(out_prefix: str, astar, rrt):
    path = Path(out_prefix + ".md")
    with path.open("w") as f:
        f.write("# A* vs RRT* Global Planner Comparison\n\n")
        f.write("| Metric | A* | RRT* | Unit |\n")
        f.write("|---|---:|---:|---|\n")
        for key, label, unit in FIELDS:
            f.write(f"| {label} | {fmt(astar.get(key))} | {fmt(rrt.get(key))} | {unit} |\n")
    return path


def path_xy(path_msg: NavPath):
    xs = [pose.pose.position.x for pose in path_msg.poses]
    ys = [pose.pose.position.y for pose in path_msg.poses]
    return np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)


def tf_xy(data, child_frame_id: str):
    times = []
    xs = []
    ys = []
    for t, msg in data:
        for tf in msg.transforms:
            if tf.child_frame_id == child_frame_id:
                times.append(t)
                xs.append(tf.transform.translation.x)
                ys.append(tf.transform.translation.y)
                break
    return (
        np.asarray(times, dtype=float),
        np.asarray(xs, dtype=float),
        np.asarray(ys, dtype=float),
    )


def collect_plot_data(label: str, bag_path: str):
    raw = {}
    for key, (topic, msg_type) in TOPICS.items():
        data = read_topic(bag_path, topic, msg_type)
        raw[key] = data

    t0 = choose_t0(raw, ["path", "cmd", "mission", "rmse", "latency", "min_dist", "safety"])
    data = {key: normalize(value, t0) for key, value in raw.items()}

    paths = [msg for _, msg in data["path"]]
    initial_path = path_xy(paths[0]) if paths else (np.asarray([]), np.asarray([]))
    final_path = path_xy(paths[-1]) if paths else (np.asarray([]), np.asarray([]))

    cmd_t = np.asarray([t for t, _ in data["cmd"]], dtype=float)
    v = np.asarray([msg.linear.x for _, msg in data["cmd"]], dtype=float)
    w = np.asarray([msg.angular.z for _, msg in data["cmd"]], dtype=float)

    rmse_t = np.asarray([t for t, _ in data["rmse"]], dtype=float)
    rmse = np.asarray([msg.rmse_total for _, msg in data["rmse"]], dtype=float)

    lat_t = np.asarray([t for t, _ in data["latency"]], dtype=float)
    solve = np.asarray([msg.latency_ms for _, msg in data["latency"]], dtype=float)
    e2e = np.asarray([msg.e2e_latency_ms for _, msg in data["latency"]], dtype=float)

    dist_pairs = [
        (t, msg.min_distance_m)
        for t, msg in data["min_dist"]
        if msg.min_distance_m < 1e8
    ]
    dist_t = np.asarray([t for t, _ in dist_pairs], dtype=float)
    dist = np.asarray([value for _, value in dist_pairs], dtype=float)

    odom_t = np.asarray([t for t, _ in data["odom"]], dtype=float)
    odom_x = np.asarray([msg.pose.pose.position.x for _, msg in data["odom"]], dtype=float)
    odom_y = np.asarray([msg.pose.pose.position.y for _, msg in data["odom"]], dtype=float)

    gt_robot_t, gt_robot_x, gt_robot_y = tf_xy(data["gt"], "amr_robot")
    obs_t, obs_x, obs_y = tf_xy(data["gt"], "dyn_obs_1")

    mission_events = [(t, msg.data) for t, msg in data["mission"]]
    safety_events = [(t, msg.state_name) for t, msg in data["safety"]]

    return {
        "label": label,
        "initial_path": initial_path,
        "final_path": final_path,
        "cmd_t": cmd_t,
        "v": v,
        "w": w,
        "rmse_t": rmse_t,
        "rmse": rmse,
        "lat_t": lat_t,
        "solve": solve,
        "e2e": e2e,
        "dist_t": dist_t,
        "dist": dist,
        "odom_t": odom_t,
        "odom_x": odom_x,
        "odom_y": odom_y,
        "gt_robot_t": gt_robot_t,
        "gt_robot_x": gt_robot_x,
        "gt_robot_y": gt_robot_y,
        "obs_t": obs_t,
        "obs_x": obs_x,
        "obs_y": obs_y,
        "mission_events": mission_events,
        "safety_events": safety_events,
    }


def finite_metric(stats, key):
    value = stats.get(key)
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return float(value)
    return None


def configure_axes(ax, title, xlabel=None, ylabel=None):
    ax.set_title(title)
    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)


def add_legend_if_any(ax, **kwargs):
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(**kwargs)


def plot_kpi_bars(astar_stats, rrt_stats, plot_dir: Path):
    import matplotlib.pyplot as plt

    metrics = [
        ("initial_path_length_m", "Initial path\nlength", "m"),
        ("time_to_docked_s", "Time to\nDOCKED", "s"),
        ("planner_time_mean_ms", "Planner time\nmean", "ms"),
        ("min_clearance_m", "Minimum\nclearance", "m"),
        ("rmse_mean_m", "Tracking RMSE\nmean", "m"),
        ("w_clip_ratio_pct", "|omega| clip\nratio", "%"),
    ]

    names = []
    astar_values = []
    rrt_values = []
    units = []
    for key, name, unit in metrics:
        a = finite_metric(astar_stats, key)
        r = finite_metric(rrt_stats, key)
        if a is None and r is None:
            continue
        names.append(name)
        astar_values.append(np.nan if a is None else a)
        rrt_values.append(np.nan if r is None else r)
        units.append(unit)

    if not names:
        return None

    x = np.arange(len(names))
    width = 0.36
    fig, ax = plt.subplots(figsize=(11, 5.5))
    ax.bar(x - width / 2.0, astar_values, width, label="A*", color="#4C78A8")
    ax.bar(x + width / 2.0, rrt_values, width, label="RRT*", color="#F58518")
    ax.set_xticks(x)
    ax.set_xticklabels([f"{name}\n[{unit}]" for name, unit in zip(names, units)])
    ax.set_title("A* vs RRT* KPI Summary")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()

    out = plot_dir / "planner_kpi_summary.png"
    fig.savefig(out, dpi=180)
    plt.close(fig)
    return out


def plot_trajectory(astar_plot, rrt_plot, plot_dir: Path):
    import matplotlib.pyplot as plt

    has_any = any([
        astar_plot["initial_path"][0].size,
        rrt_plot["initial_path"][0].size,
        astar_plot["odom_x"].size,
        rrt_plot["odom_x"].size,
        astar_plot["gt_robot_x"].size,
        rrt_plot["gt_robot_x"].size,
    ])
    if not has_any:
        return None

    fig, ax = plt.subplots(figsize=(8, 8))

    for plot, color in [(astar_plot, "#4C78A8"), (rrt_plot, "#F58518")]:
        label = plot["label"]
        px, py = plot["initial_path"]
        if px.size:
            ax.plot(px, py, "--", color=color, linewidth=2.0, label=f"{label} initial path")

        if plot["odom_x"].size:
            ax.plot(plot["odom_x"], plot["odom_y"], color=color, linewidth=1.8, label=f"{label} odom")
        elif plot["gt_robot_x"].size:
            ax.plot(plot["gt_robot_x"], plot["gt_robot_y"], color=color, linewidth=1.8, label=f"{label} GT")

    for plot, color in [(astar_plot, "#72B7B2"), (rrt_plot, "#B279A2")]:
        if plot["obs_x"].size:
            ax.plot(
                plot["obs_x"],
                plot["obs_y"],
                color=color,
                linewidth=1.5,
                alpha=0.8,
                label=f"{plot['label']} dynamic obstacle",
            )

    configure_axes(ax, "2D Planned Path and Executed Trajectory", "x [m]", "y [m]")
    ax.axis("equal")
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()

    out = plot_dir / "planner_trajectory_overlay.png"
    fig.savefig(out, dpi=180)
    plt.close(fig)
    return out


def plot_time_series(astar_plot, rrt_plot, plot_dir: Path):
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 2, figsize=(13, 10), sharex=False)
    axes = axes.reshape(3, 2)

    for plot, color in [(astar_plot, "#4C78A8"), (rrt_plot, "#F58518")]:
        label = plot["label"]
        if plot["cmd_t"].size:
            axes[0, 0].plot(plot["cmd_t"], plot["v"], color=color, linewidth=1.2, label=label)
            axes[0, 1].plot(plot["cmd_t"], plot["w"], color=color, linewidth=1.2, label=label)
        if plot["dist_t"].size:
            axes[1, 0].plot(plot["dist_t"], plot["dist"], color=color, linewidth=1.2, label=label)
        if plot["rmse_t"].size:
            axes[1, 1].plot(plot["rmse_t"], plot["rmse"], color=color, linewidth=1.2, label=label)
        if plot["lat_t"].size:
            axes[2, 0].plot(plot["lat_t"], plot["solve"], color=color, linewidth=1.1, label=label)
            axes[2, 1].plot(plot["lat_t"], plot["e2e"], color=color, linewidth=1.1, label=label)

    configure_axes(axes[0, 0], "Linear Velocity Command", "time [s]", "v [m/s]")
    configure_axes(axes[0, 1], "Angular Velocity Command", "time [s]", "omega [rad/s]")
    configure_axes(axes[1, 0], "Minimum Obstacle Clearance", "time [s]", "distance [m]")
    configure_axes(axes[1, 1], "Tracking RMSE", "time [s]", "RMSE [m]")
    configure_axes(axes[2, 0], "Control Solve Time", "time [s]", "latency [ms]")
    configure_axes(axes[2, 1], "End-to-End Latency", "time [s]", "latency [ms]")

    for ax in axes.ravel():
        add_legend_if_any(ax, loc="best", fontsize=9)

    fig.suptitle("A* vs RRT* Time-Series Comparison", fontsize=15)
    fig.tight_layout()

    out = plot_dir / "planner_time_series.png"
    fig.savefig(out, dpi=180)
    plt.close(fig)
    return out


def plot_timeline(astar_plot, rrt_plot, plot_dir: Path):
    import matplotlib.pyplot as plt

    events = []
    for row, plot in enumerate([astar_plot, rrt_plot]):
        for t, phase in plot["mission_events"]:
            if phase in {"TRACKING", "DOCKING", "DOCKED"}:
                events.append((t, row, phase, plot["label"]))
        for t, state in plot["safety_events"]:
            if state != "NORMAL":
                events.append((t, row + 0.25, state, plot["label"]))

    if not events:
        return None

    fig, ax = plt.subplots(figsize=(11, 3.5))
    color_map = {
        "TRACKING": "#4C78A8",
        "DOCKING": "#F58518",
        "DOCKED": "#54A24B",
        "DEGRADED": "#ECA82C",
        "SAFE_STOP": "#E45756",
        "MANUAL_OVERRIDE": "#B279A2",
    }

    for t, y, name, label in events:
        ax.scatter(t, y, s=60, color=color_map.get(name, "#666666"), zorder=3)
        ax.text(t, y + 0.06, name, rotation=35, ha="left", va="bottom", fontsize=8)

    ax.set_yticks([0, 1])
    ax.set_yticklabels(["A*", "RRT*"])
    configure_axes(ax, "Mission and Safety Timeline", "time [s]", "")
    fig.tight_layout()

    out = plot_dir / "planner_timeline.png"
    fig.savefig(out, dpi=180)
    plt.close(fig)
    return out


def write_plots(out_prefix: str, astar_stats, rrt_stats, astar_bag: str, rrt_bag: str):
    mpl_config = Path("/tmp/matplotlib")
    mpl_config.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(mpl_config))
    warnings.filterwarnings("ignore", message="Unable to import Axes3D.*")

    import matplotlib

    matplotlib.use("Agg")

    plot_dir = Path(out_prefix + "_plots")
    plot_dir.mkdir(parents=True, exist_ok=True)

    astar_plot = collect_plot_data("A*", astar_bag)
    rrt_plot = collect_plot_data("RRT*", rrt_bag)

    outputs = []
    for out in [
        plot_kpi_bars(astar_stats, rrt_stats, plot_dir),
        plot_trajectory(astar_plot, rrt_plot, plot_dir),
        plot_time_series(astar_plot, rrt_plot, plot_dir),
        plot_timeline(astar_plot, rrt_plot, plot_dir),
    ]:
        if out is not None:
            outputs.append(out)

    return outputs


def main():
    parser = argparse.ArgumentParser(description="Compare A* and RRT* planner runs from ROS2 bags.")
    parser.add_argument("--astar-bag", required=True, help="A* rosbag directory or .db3 path")
    parser.add_argument("--rrt-bag", required=True, help="RRT* rosbag directory or .db3 path")
    parser.add_argument("--astar-log", default=None, help="Optional A* terminal log/result.txt")
    parser.add_argument("--rrt-log", default=None, help="Optional RRT* terminal log/result.txt")
    parser.add_argument("--out-prefix", default="planner_compare", help="Output prefix for CSV/Markdown")
    parser.add_argument("--no-plots", action="store_true", help="Skip PNG plot generation")
    args = parser.parse_args()

    astar = analyze_bag("A*", args.astar_bag, args.astar_log)
    rrt = analyze_bag("RRT*", args.rrt_bag, args.rrt_log)

    print_table(astar, rrt)
    csv_path = write_csv(args.out_prefix, astar, rrt)
    md_path = write_markdown(args.out_prefix, astar, rrt)
    print(f"\nSaved: {csv_path}")
    print(f"Saved: {md_path}")
    if not args.no_plots:
        try:
            plot_paths = write_plots(args.out_prefix, astar, rrt, args.astar_bag, args.rrt_bag)
            for path in plot_paths:
                print(f"Saved: {path}")
        except ImportError as exc:
            print(f"[WARN] Plot generation skipped. Missing Python package: {exc}")


if __name__ == "__main__":
    main()
