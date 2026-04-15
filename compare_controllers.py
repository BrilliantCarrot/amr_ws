"""
MPC vs TVLQR 제어기 비교 분석 스크립트

[사용법]
  python3 compare_controllers.py <mpc_bag_path> <lqr_bag_path>

[분석 항목]
  1. 경로 추종 오차 (Tracking RMSE) : /metrics/tracking_rmse
  2. 제어 지연 (Latency)            : /metrics/control_latency_ms  vs /metrics/lqr_control_latency_ms
  3. 수렴 속도 (Convergence)        : RMSE가 0.05m 이하로 수렴하는 시간
  4. 진동 (Oscillation)            : cmd_vel 시계열 표준편차
  5. 장애물 최소 거리               : /metrics/min_obstacle_distance (MPC만)

[출력]
  - 터미널: 비교 통계표
  - PNG   : 4개 비교 그래프
"""

import sys
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from rclpy.serialization import deserialize_message
from amr_msgs.msg import PoseRmse, ControlLatency, MinObstacleDistance
from geometry_msgs.msg import Twist

# ============================================================
# [추가된 부분] 한글 폰트 및 마이너스 기호 설정
# ============================================================
plt.rcParams['font.family'] = 'NanumGothic'
plt.rcParams['axes.unicode_minus'] = False
# ============================================================
# ============================================================
# 유틸리티: rosbag SQLite3에서 토픽 데이터 읽기
# ============================================================
def read_topic(bag_path: str, topic_name: str, msg_type):
    """
    rosbag SQLite3에서 특정 토픽의 메시지를 전부 읽어 반환함.
    반환: list of (timestamp_sec, deserialized_msg)
    """
    db_path = f"{bag_path}/{bag_path.split('/')[-1]}_0.db3"
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    cursor.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
    row = cursor.fetchone()
    if row is None:
        print(f"  [경고] '{topic_name}' 토픽이 bag에 없음: {bag_path}")
        conn.close()
        return []
    topic_id = row[0]

    cursor.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (topic_id,),
    )
    rows = cursor.fetchall()
    conn.close()

    result = []
    for ts_ns, data in rows:
        msg = deserialize_message(data, msg_type)
        result.append((ts_ns * 1e-9, msg))  # ns → sec
    return result


def normalize_time(data):
    """타임스탬프를 0 기준으로 정규화 (시작 시각 = 0)"""
    if not data:
        return []
    t0 = data[0][0]
    return [(t - t0, msg) for t, msg in data]


# ============================================================
# 각 지표 데이터 추출
# ============================================================
def extract_rmse(data):
    """(time, rmse_total) 리스트 반환"""
    return [(t, msg.rmse_total) for t, msg in data]


def extract_latency(data):
    """(time, latency_ms) 리스트 반환"""
    return [(t, msg.latency_ms) for t, msg in data]


def extract_e2e(data):
    """(time, e2e_latency_ms) 리스트 반환"""
    return [(t, msg.e2e_latency_ms) for t, msg in data]


def extract_cmdvel(bag_path: str):
    """cmd_vel 토픽에서 (time, v, omega) 리스트 반환"""
    data = read_topic(bag_path, "/cmd_vel", Twist)
    data = normalize_time(data)
    return [(t, msg.linear.x, msg.angular.z) for t, msg in data]


def extract_min_dist(data):
    """(time, min_distance_m) 리스트 반환"""
    return [(t, msg.min_distance_m) for t, msg in data]


# ============================================================
# 통계 계산
# ============================================================
def convergence_time(rmse_data, threshold=0.05):
    """
    RMSE가 threshold 이하로 수렴하는 시각 반환.
    수렴 후 연속 10개 샘플이 threshold 이하여야 수렴으로 판정.
    """
    times  = [d[0] for d in rmse_data]
    values = [d[1] for d in rmse_data]
    window = 10
    for i in range(len(values) - window):
        if all(v < threshold for v in values[i:i + window]):
            return times[i]
    return None  # 수렴 못 함


def oscillation_score(cmdvel_data):
    """
    cmd_vel 시계열 진동 점수 계산.
    방법: v, ω 각각의 표준편차와 연속 변화량(1차 차분) 표준편차의 합.
    클수록 진동이 심함.
    """
    if len(cmdvel_data) < 2:
        return 0.0, 0.0
    vs  = np.array([d[1] for d in cmdvel_data])
    ws  = np.array([d[2] for d in cmdvel_data])
    dv  = np.diff(vs)
    dw  = np.diff(ws)
    return float(np.std(dv)), float(np.std(dw))


# ============================================================
# 터미널 비교표 출력
# ============================================================
def print_comparison(mpc_stats: dict, lqr_stats: dict):
    print("\n" + "=" * 65)
    print("  MPC vs TVLQR 제어기 비교 결과")
    print("=" * 65)
    fmt = "{:<30} {:>14} {:>14}"
    print(fmt.format("항목", "MPC", "TVLQR"))
    print("-" * 65)

    def row(label, mval, lval, unit=""):
        ms = f"{mval:.4f}{unit}" if mval is not None else "N/A"
        ls = f"{lval:.4f}{unit}" if lval is not None else "N/A"
        print(fmt.format(label, ms, ls))

    # Tracking RMSE
    row("Tracking RMSE 평균 [m]",
        mpc_stats.get("rmse_mean"), lqr_stats.get("rmse_mean"))
    row("Tracking RMSE 최대 [m]",
        mpc_stats.get("rmse_max"), lqr_stats.get("rmse_max"))
    row("Tracking RMSE 표준편차 [m]",
        mpc_stats.get("rmse_std"), lqr_stats.get("rmse_std"))

    print("-" * 65)

    # Latency
    row("제어 Latency 평균 [ms]",
        mpc_stats.get("lat_mean"), lqr_stats.get("lat_mean"))
    row("제어 Latency 최대 [ms]",
        mpc_stats.get("lat_max"), lqr_stats.get("lat_max"))
    row("e2e Latency 평균 [ms]",
        mpc_stats.get("e2e_mean"), lqr_stats.get("e2e_mean"))
    row("e2e Latency 99th [ms]",
        mpc_stats.get("e2e_p99"), lqr_stats.get("e2e_p99"))

    print("-" * 65)

    # 수렴 속도
    mt = mpc_stats.get("conv_time")
    lt = lqr_stats.get("conv_time")
    print(fmt.format(
        "수렴 시간 (RMSE<0.05m) [s]",
        f"{mt:.2f}s" if mt else "미수렴",
        f"{lt:.2f}s" if lt else "미수렴"))

    print("-" * 65)

    # 진동
    row("진동(Δv 표준편차)",
        mpc_stats.get("osc_dv"), lqr_stats.get("osc_dv"))
    row("진동(Δω 표준편차)",
        mpc_stats.get("osc_dw"), lqr_stats.get("osc_dw"))

    print("-" * 65)

    # 장애물 최소 거리 (MPC만)
    md = mpc_stats.get("min_dist_min")
    print(fmt.format(
        "장애물 최소 거리 [m]",
        f"{md:.4f}m" if md else "N/A",
        "(글로벌 경로계획 위임)"))

    print("=" * 65 + "\n")


# ============================================================
# 비교 그래프 생성
# ============================================================
def plot_comparison(mpc_data: dict, lqr_data: dict, out_path="mpc_vs_lqr.png"):
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle("MPC vs TVLQR 제어기 비교", fontsize=15, fontweight="bold")
    gs = gridspec.GridSpec(2, 2, hspace=0.4, wspace=0.35)

    # ---- 1. Tracking RMSE 시계열 ----
    ax1 = fig.add_subplot(gs[0, 0])
    if mpc_data.get("rmse"):
        t, v = zip(*mpc_data["rmse"])
        ax1.plot(t, v, label="MPC", color="steelblue", linewidth=1.2)
    if lqr_data.get("rmse"):
        t, v = zip(*lqr_data["rmse"])
        ax1.plot(t, v, label="TVLQR", color="tomato", linewidth=1.2)
    ax1.axhline(0.05, color="gray", linestyle="--", linewidth=0.8, label="수렴 기준 0.05m")
    ax1.set_xlabel("시간 [s]")
    ax1.set_ylabel("Tracking RMSE [m]")
    ax1.set_title("경로 추종 오차 (Tracking RMSE)")
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    # ---- 2. 제어 Latency 시계열 ----
    ax2 = fig.add_subplot(gs[0, 1])
    if mpc_data.get("lat"):
        t, v = zip(*mpc_data["lat"])
        ax2.plot(t, v, label="MPC", color="steelblue", linewidth=1.0, alpha=0.8)
    if lqr_data.get("lat"):
        t, v = zip(*lqr_data["lat"])
        ax2.plot(t, v, label="TVLQR", color="tomato", linewidth=1.0, alpha=0.8)
    ax2.axhline(20.0, color="gray", linestyle="--", linewidth=0.8, label="기준 20ms")
    ax2.set_xlabel("시간 [s]")
    ax2.set_ylabel("Latency [ms]")
    ax2.set_title("제어 Latency (solve time)")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    # ---- 3. cmd_vel 시계열 (선속도 v) ----
    ax3 = fig.add_subplot(gs[1, 0])
    if mpc_data.get("cmdvel"):
        t = [d[0] for d in mpc_data["cmdvel"]]
        v = [d[1] for d in mpc_data["cmdvel"]]
        ax3.plot(t, v, label="MPC v", color="steelblue", linewidth=1.0)
    if lqr_data.get("cmdvel"):
        t = [d[0] for d in lqr_data["cmdvel"]]
        v = [d[1] for d in lqr_data["cmdvel"]]
        ax3.plot(t, v, label="TVLQR v", color="tomato", linewidth=1.0)
    ax3.set_xlabel("시간 [s]")
    ax3.set_ylabel("선속도 v [m/s]")
    ax3.set_title("cmd_vel 시계열 — 선속도 (진동 확인)")
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    # ---- 4. cmd_vel 시계열 (각속도 ω) ----
    ax4 = fig.add_subplot(gs[1, 1])
    if mpc_data.get("cmdvel"):
        t = [d[0] for d in mpc_data["cmdvel"]]
        w = [d[2] for d in mpc_data["cmdvel"]]
        ax4.plot(t, w, label="MPC ω", color="steelblue", linewidth=1.0)
    if lqr_data.get("cmdvel"):
        t = [d[0] for d in lqr_data["cmdvel"]]
        w = [d[2] for d in lqr_data["cmdvel"]]
        ax4.plot(t, w, label="TVLQR ω", color="tomato", linewidth=1.0)
    ax4.set_xlabel("시간 [s]")
    ax4.set_ylabel("각속도 ω [rad/s]")
    ax4.set_title("cmd_vel 시계열 — 각속도 (진동 확인)")
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)

    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"[완료] 그래프 저장: {out_path}")
    plt.show()


# ============================================================
# 메인
# ============================================================
def main():
    if len(sys.argv) < 3:
        print("사용법: python3 compare_controllers.py <mpc_bag_path> <lqr_bag_path>")
        sys.exit(1)

    mpc_bag = sys.argv[1]
    lqr_bag = sys.argv[2]

    print(f"\n[분석 시작]")
    print(f"  MPC bag : {mpc_bag}")
    print(f"  LQR bag : {lqr_bag}")

    # ---- MPC 데이터 읽기 ----
    print("\n[MPC 데이터 읽는 중...]")
    mpc_rmse_raw  = normalize_time(read_topic(mpc_bag, "/metrics/tracking_rmse",        PoseRmse))
    mpc_lat_raw   = normalize_time(read_topic(mpc_bag, "/metrics/control_latency_ms",    ControlLatency))
    mpc_dist_raw  = normalize_time(read_topic(mpc_bag, "/metrics/min_obstacle_distance", MinObstacleDistance))

    mpc_rmse_data = extract_rmse(mpc_rmse_raw)
    mpc_lat_data  = extract_latency(mpc_lat_raw)
    mpc_e2e_data  = extract_e2e(mpc_lat_raw)
    mpc_cv_data   = extract_cmdvel(mpc_bag)
    mpc_dist_data = extract_min_dist(mpc_dist_raw)

    # ---- LQR 데이터 읽기 ----
    print("[LQR 데이터 읽는 중...]")
    lqr_rmse_raw = normalize_time(read_topic(lqr_bag, "/metrics/lqr_tracking_rmse",        PoseRmse))
    lqr_lat_raw  = normalize_time(read_topic(lqr_bag, "/metrics/lqr_control_latency_ms",    ControlLatency))

    lqr_rmse_data = extract_rmse(lqr_rmse_raw)
    lqr_lat_data  = extract_latency(lqr_lat_raw)
    lqr_e2e_data  = extract_e2e(lqr_lat_raw)
    lqr_cv_data   = extract_cmdvel(lqr_bag)

    # ---- 통계 계산 ----
    def stats(rmse_data, lat_data, e2e_data, cv_data, dist_data=None):
        s = {}
        if rmse_data:
            v = [d[1] for d in rmse_data]
            s["rmse_mean"] = float(np.mean(v))
            s["rmse_max"]  = float(np.max(v))
            s["rmse_std"]  = float(np.std(v))
            s["conv_time"] = convergence_time(rmse_data)
        if lat_data:
            v = [d[1] for d in lat_data]
            s["lat_mean"] = float(np.mean(v))
            s["lat_max"]  = float(np.max(v))
        if e2e_data:
            v = [d[1] for d in e2e_data]
            s["e2e_mean"] = float(np.mean(v))
            s["e2e_p99"]  = float(np.percentile(v, 99))
        if cv_data:
            s["osc_dv"], s["osc_dw"] = oscillation_score(cv_data)
        if dist_data:
            v = [d[1] for d in dist_data if d[1] < 900]  # 미측정(999) 제외
            if v:
                s["min_dist_min"] = float(np.min(v))
        return s

    mpc_stats = stats(mpc_rmse_data, mpc_lat_data, mpc_e2e_data, mpc_cv_data, mpc_dist_data)
    lqr_stats = stats(lqr_rmse_data, lqr_lat_data, lqr_e2e_data, lqr_cv_data)

    # ---- 출력 ----
    print_comparison(mpc_stats, lqr_stats)

    # ---- 그래프 ----
    mpc_plot = {"rmse": mpc_rmse_data, "lat": mpc_lat_data, "cmdvel": mpc_cv_data}
    lqr_plot = {"rmse": lqr_rmse_data, "lat": lqr_lat_data, "cmdvel": lqr_cv_data}
    plot_comparison(mpc_plot, lqr_plot)


if __name__ == "__main__":
    main()
