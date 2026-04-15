"""
MPC vs TVLQR 경로별 제어기 비교 분석 스크립트
원형(circle) + 8자(figure8) 동시 비교

[사용법]
  python3 compare_trajectory.py \
    --mpc_circle  ~/amr_ws/bags/mpc_circle  \
    --lqr_circle  ~/amr_ws/bags/lqr_circle  \
    --mpc_figure8 ~/amr_ws/bags/mpc_figure8 \
    --lqr_figure8 ~/amr_ws/bags/lqr_figure8

  # 원형만 비교할 경우 (figure8 생략 가능)
  python3 compare_trajectory.py \
    --mpc_circle ~/amr_ws/bags/mpc_circle \
    --lqr_circle ~/amr_ws/bags/lqr_circle

[분석 지표]
  1. Tracking RMSE        : 경로 추종 오차 (mean / max / std)
  2. RMSE 안정화 시간      : RMSE가 처음 0.25m 이하로 내려오는 시간 (순환 경로 적합)
  3. 제어 Latency         : solve time (mean / max / p99)
  4. e2e Latency          : 센서→제어 전체 지연 (mean / p99)
  5. 각속도 진동 (Δω std) : smooth 제어 여부 — MPC horizon 효과 수치화
  6. 선속도 안정성 (v std) : 속도 일관성
  7. 각속도 클리핑 비율    : |ω|=1.0 클리핑 횟수 비율 (constraint 처리 능력 차이)
  8. RMSE 주기적 변동      : 원형 한 바퀴 단위 RMSE 평균 (순환 경로 수렴 확인)

[출력]
  - 터미널: 경로별 비교 통계표
  - PNG   : 경로별 6개 서브플롯 그래프
"""

import argparse
import sqlite3
import struct
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

plt.rcParams['font.family'] = 'NanumGothic'
plt.rcParams['axes.unicode_minus'] = False


# ============================================================
# 데이터 읽기 (역직렬화 없이 struct로 직접 파싱 — ROS2 불필요)
# ============================================================

def read_raw(bag_path: str, topic_name: str):
    """SQLite3에서 raw bytes 읽기. 반환: [(timestamp_sec, data_bytes)]"""
    db_path = f"{bag_path}/{bag_path.split('/')[-1]}_0.db3"
    try:
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
        row = cursor.fetchone()
        if row is None:
            conn.close()
            return []
        tid = row[0]
        cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (tid,)
        )
        rows = cursor.fetchall()
        conn.close()
        return [(ts * 1e-9, data) for ts, data in rows]
    except Exception as e:
        print(f"  [경고] {topic_name} 읽기 실패: {e}")
        return []


def parse_pose_rmse(rows):
    """PoseRmse CDR → (time, rmse_x, rmse_y, rmse_yaw, rmse_total)"""
    result = []
    t0 = rows[0][0] if rows else 0
    for ts, data in rows:
        try:
            # PoseRmse: header(가변) + 4×float64 (rmse_x, y, yaw, total)
            vals = struct.unpack_from('<4d', data, len(data) - 32)
            result.append((ts - t0, vals[0], vals[1], vals[2], vals[3]))
        except Exception:
            pass
    return result


def parse_latency(rows):
    """ControlLatency CDR → (time, latency_ms, e2e_ms)"""
    result = []
    t0 = rows[0][0] if rows else 0
    for ts, data in rows:
        try:
            # ControlLatency: header + 6×float64
            vals = struct.unpack_from('<6d', data, len(data) - 48)
            result.append((ts - t0, vals[0], vals[3]))  # latency_ms, e2e_ms
        except Exception:
            pass
    return result


def parse_cmdvel(rows):
    """Twist CDR → (time, v, omega)"""
    result = []
    t0 = rows[0][0] if rows else 0
    for ts, data in rows:
        try:
            # Twist: 6×float64 (linear.x,y,z, angular.x,y,z)
            vals = struct.unpack_from('<6d', data, 4)
            result.append((ts - t0, vals[0], vals[5]))
        except Exception:
            pass
    return result


def load_bag(bag_path, prefix=""):
    """bag 경로에서 모든 지표 데이터 로드"""
    if not bag_path:
        return None

    # RMSE 토픽명 결정
    rmse_topic = "/metrics/tracking_rmse" if not prefix else f"/metrics/{prefix}tracking_rmse"
    lat_topic  = "/metrics/control_latency_ms" if not prefix else f"/metrics/{prefix}control_latency_ms"

    rmse_raw = read_raw(bag_path, rmse_topic)
    lat_raw  = read_raw(bag_path, lat_topic)
    cv_raw   = read_raw(bag_path, "/cmd_vel")

    if not rmse_raw:
        # 토픽명 자동 탐지
        for candidate in ["/metrics/tracking_rmse", "/metrics/lqr_tracking_rmse"]:
            rmse_raw = read_raw(bag_path, candidate)
            if rmse_raw:
                break
    if not lat_raw:
        for candidate in ["/metrics/control_latency_ms", "/metrics/lqr_control_latency_ms"]:
            lat_raw = read_raw(bag_path, candidate)
            if lat_raw:
                break

    return {
        "rmse": parse_pose_rmse(rmse_raw),
        "lat":  parse_latency(lat_raw),
        "cv":   parse_cmdvel(cv_raw),
    }


# ============================================================
# 통계 계산
# ============================================================

def calc_stats(data: dict, label: str) -> dict:
    """데이터에서 핵심 통계 계산"""
    s = {"label": label}
    if not data:
        return s

    # --- RMSE ---
    rmse = data.get("rmse", [])
    if rmse:
        vals = np.array([r[4] for r in rmse])   # rmse_total
        times = np.array([r[0] for r in rmse])
        s["rmse_mean"] = float(np.mean(vals))
        s["rmse_max"]  = float(np.max(vals))
        s["rmse_std"]  = float(np.std(vals))

        # RMSE 안정화 시간: 처음 0.25m 이하로 내려오는 시각
        # (순환 경로는 초반에 크고 점점 줄어드는 패턴)
        threshold = 0.25
        window    = 10
        stab_time = None
        for i in range(len(vals) - window):
            if all(v < threshold for v in vals[i:i+window]):
                stab_time = times[i]
                break
        s["stab_time"] = stab_time

        # 주기적 RMSE 변동: 전체를 4구간으로 나눠 구간별 평균
        n = len(vals)
        q = n // 4
        s["rmse_q1"] = float(np.mean(vals[:q]))
        s["rmse_q2"] = float(np.mean(vals[q:2*q]))
        s["rmse_q3"] = float(np.mean(vals[2*q:3*q]))
        s["rmse_q4"] = float(np.mean(vals[3*q:]))

    # --- Latency ---
    lat = data.get("lat", [])
    if lat:
        lat_ms  = np.array([l[1] for l in lat])
        e2e_ms  = np.array([l[2] for l in lat])
        s["lat_mean"] = float(np.mean(lat_ms))
        s["lat_max"]  = float(np.max(lat_ms))
        s["lat_p99"]  = float(np.percentile(lat_ms, 99))
        s["e2e_mean"] = float(np.mean(e2e_ms))
        s["e2e_p99"]  = float(np.percentile(e2e_ms, 99))
        s["spike_pct"] = float(np.sum(lat_ms > 5.0) / len(lat_ms) * 100)

    # --- cmd_vel ---
    cv = data.get("cv", [])
    if cv and len(cv) >= 2:
        vs = np.array([c[1] for c in cv])
        ws = np.array([c[2] for c in cv])
        dv = np.diff(vs)
        dw = np.diff(ws)
        s["v_std"]       = float(np.std(vs))
        s["v_mean"]      = float(np.mean(vs))
        s["osc_dv"]      = float(np.std(dv))
        s["osc_dw"]      = float(np.std(dw))
        # |ω| >= 1.0 클리핑 비율 — LQR constraint 처리 한계 수치화
        s["clip_pct"]    = float(np.sum(np.abs(ws) >= 0.999) / len(ws) * 100)

    return s


# ============================================================
# 터미널 출력
# ============================================================

def print_table(traj_name: str, mpc_s: dict, lqr_s: dict):
    w = 65
    print()
    print("=" * w)
    print(f"  {traj_name} 경로: MPC vs TVLQR 비교")
    print("=" * w)
    fmt = "{:<35} {:>13} {:>13}"
    print(fmt.format("항목", "MPC", "TVLQR"))
    print("-" * w)

    def row(label, mk, lk, unit="", fmt_str=".4f"):
        mv = mpc_s.get(mk)
        lv = lqr_s.get(lk) if lqr_s else None
        ms = f"{mv:{fmt_str}}{unit}" if mv is not None else "N/A"
        ls = f"{lv:{fmt_str}}{unit}" if lv is not None else "N/A"
        print(fmt.format(label, ms, ls))

    def winner(mk, lk, lower_is_better=True):
        """어느 쪽이 우세한지 표시"""
        mv = mpc_s.get(mk)
        lv = lqr_s.get(lk) if lqr_s else None
        if mv is None or lv is None:
            return ""
        if lower_is_better:
            return "← MPC 우세" if mv < lv else "← LQR 우세"
        else:
            return "← MPC 우세" if mv > lv else "← LQR 우세"

    print(fmt.format("【경로 추종 오차 (Tracking RMSE)】", "", ""))
    row("  평균 RMSE [m]",          "rmse_mean", "rmse_mean")
    row("  최대 RMSE [m]",          "rmse_max",  "rmse_max")
    row("  RMSE 표준편차 [m]",      "rmse_std",  "rmse_std")
    print(fmt.format("  RMSE 안정화 시간 [s]",
        f"{mpc_s['stab_time']:.2f}s" if mpc_s.get('stab_time') else "미달",
        f"{lqr_s['stab_time']:.2f}s" if lqr_s and lqr_s.get('stab_time') else "미달"))
    print(fmt.format("  RMSE 구간별 평균 (Q1/Q2/Q3/Q4)",
        "/".join(f"{mpc_s.get(f'rmse_q{i}',0):.3f}" for i in range(1,5)),
        "/".join(f"{lqr_s.get(f'rmse_q{i}',0):.3f}" for i in range(1,5)) if lqr_s else "N/A"))
    print("-" * w)

    print(fmt.format("【제어 Latency】", "", ""))
    row("  solve time 평균 [ms]",   "lat_mean",  "lat_mean")
    row("  solve time 최대 [ms]",   "lat_max",   "lat_max")
    row("  solve time p99 [ms]",    "lat_p99",   "lat_p99")
    row("  e2e latency 평균 [ms]",  "e2e_mean",  "e2e_mean")
    row("  e2e latency p99 [ms]",   "e2e_p99",   "e2e_p99")
    row("  spike(>5ms) 비율 [%]",   "spike_pct", "spike_pct", "%", ".1f")
    print("-" * w)

    print(fmt.format("【진동 / 제어 품질】", "", ""))
    row("  선속도 평균 v [m/s]",    "v_mean",    "v_mean")
    row("  선속도 편차 v_std",      "v_std",     "v_std")
    row("  Δv 표준편차 (진동)",     "osc_dv",    "osc_dv")
    row("  Δω 표준편차 (진동)",     "osc_dw",    "osc_dw",
        " ← " + winner("osc_dw", "osc_dw"))
    row("  |ω|=1 클리핑 비율 [%]", "clip_pct",  "clip_pct", "%", ".1f")
    print("=" * w)

    # 우열 요약
    print()
    print("  【결과 요약】")
    comparisons = [
        ("RMSE 평균",     "rmse_mean", "rmse_mean",  True),
        ("RMSE 안정성",   "rmse_std",  "rmse_std",   True),
        ("solve time",    "lat_mean",  "lat_mean",   True),
        ("e2e latency",   "e2e_mean",  "e2e_mean",   True),
        ("각속도 진동",   "osc_dw",    "osc_dw",     True),
        ("클리핑 비율",   "clip_pct",  "clip_pct",   True),
    ]
    mpc_wins = 0
    lqr_wins = 0
    for name, mk, lk, lib in comparisons:
        mv = mpc_s.get(mk)
        lv = lqr_s.get(lk) if lqr_s else None
        if mv is None or lv is None:
            continue
        if lib:
            if mv < lv:
                print(f"    ✅ MPC 우세: {name} ({mv:.4f} vs {lv:.4f})")
                mpc_wins += 1
            elif lv < mv:
                print(f"    ✅ LQR 우세: {name} ({lv:.4f} vs {mv:.4f})")
                lqr_wins += 1
            else:
                print(f"    — 동등: {name}")
    print(f"\n    MPC 우세 항목: {mpc_wins}개 / LQR 우세 항목: {lqr_wins}개")


# ============================================================
# 그래프
# ============================================================

def plot_trajectory(traj_name: str,
                    mpc_data: dict, lqr_data: dict,
                    ax_rmse, ax_lat, ax_v, ax_w, ax_dw, ax_clip):
    """한 경로에 대한 6개 서브플롯 그리기"""

    color_mpc = "steelblue"
    color_lqr = "tomato"
    alpha = 0.85

    # 1. RMSE 시계열
    if mpc_data and mpc_data.get("rmse"):
        t = [r[0] for r in mpc_data["rmse"]]
        v = [r[4] for r in mpc_data["rmse"]]
        ax_rmse.plot(t, v, label="MPC", color=color_mpc, lw=1.2, alpha=alpha)
    if lqr_data and lqr_data.get("rmse"):
        t = [r[0] for r in lqr_data["rmse"]]
        v = [r[4] for r in lqr_data["rmse"]]
        ax_rmse.plot(t, v, label="TVLQR", color=color_lqr, lw=1.2, alpha=alpha)
    ax_rmse.axhline(0.25, color="gray", ls="--", lw=0.8, label="안정화 기준 0.25m")
    ax_rmse.set_title(f"{traj_name} — Tracking RMSE")
    ax_rmse.set_xlabel("시간 [s]"); ax_rmse.set_ylabel("RMSE [m]")
    ax_rmse.legend(fontsize=7); ax_rmse.grid(True, alpha=0.3)

    # 2. Latency 시계열
    if mpc_data and mpc_data.get("lat"):
        t = [l[0] for l in mpc_data["lat"]]
        v = [l[1] for l in mpc_data["lat"]]
        ax_lat.plot(t, v, label="MPC", color=color_mpc, lw=0.8, alpha=0.6)
    if lqr_data and lqr_data.get("lat"):
        t = [l[0] for l in lqr_data["lat"]]
        v = [l[1] for l in lqr_data["lat"]]
        ax_lat.plot(t, v, label="TVLQR", color=color_lqr, lw=0.8, alpha=0.6)
    ax_lat.axhline(20.0, color="gray", ls="--", lw=0.8, label="제어주기 20ms")
    ax_lat.set_title(f"{traj_name} — solve time Latency")
    ax_lat.set_xlabel("시간 [s]"); ax_lat.set_ylabel("Latency [ms]")
    ax_lat.legend(fontsize=7); ax_lat.grid(True, alpha=0.3)

    # 3. 선속도 v
    if mpc_data and mpc_data.get("cv"):
        t = [c[0] for c in mpc_data["cv"]]
        v = [c[1] for c in mpc_data["cv"]]
        ax_v.plot(t, v, label="MPC", color=color_mpc, lw=1.0, alpha=alpha)
    if lqr_data and lqr_data.get("cv"):
        t = [c[0] for c in lqr_data["cv"]]
        v = [c[1] for c in lqr_data["cv"]]
        ax_v.plot(t, v, label="TVLQR", color=color_lqr, lw=1.0, alpha=alpha)
    ax_v.set_title(f"{traj_name} — 선속도 v")
    ax_v.set_xlabel("시간 [s]"); ax_v.set_ylabel("v [m/s]")
    ax_v.legend(fontsize=7); ax_v.grid(True, alpha=0.3)

    # 4. 각속도 ω
    if mpc_data and mpc_data.get("cv"):
        t = [c[0] for c in mpc_data["cv"]]
        w = [c[2] for c in mpc_data["cv"]]
        ax_w.plot(t, w, label="MPC", color=color_mpc, lw=1.0, alpha=alpha)
    if lqr_data and lqr_data.get("cv"):
        t = [c[0] for c in lqr_data["cv"]]
        w = [c[2] for c in lqr_data["cv"]]
        ax_w.plot(t, w, label="TVLQR", color=color_lqr, lw=1.0, alpha=alpha)
    ax_w.set_title(f"{traj_name} — 각속도 ω (진동)")
    ax_w.set_xlabel("시간 [s]"); ax_w.set_ylabel("ω [rad/s]")
    ax_w.legend(fontsize=7); ax_w.grid(True, alpha=0.3)

    # 5. Δω (각속도 변화량) — horizon 효과 시각화
    for data, label, color in [(mpc_data, "MPC", color_mpc),
                                (lqr_data, "TVLQR", color_lqr)]:
        if data and data.get("cv") and len(data["cv"]) >= 2:
            t  = np.array([c[0] for c in data["cv"]])[1:]
            dw = np.diff([c[2] for c in data["cv"]])
            ax_dw.plot(t, dw, label=label, color=color, lw=0.8, alpha=0.7)
    ax_dw.set_title(f"{traj_name} — Δω (각속도 변화량)")
    ax_dw.set_xlabel("시간 [s]"); ax_dw.set_ylabel("Δω [rad/s]")
    ax_dw.legend(fontsize=7); ax_dw.grid(True, alpha=0.3)

    # 6. |ω| 클리핑 히스토그램 — constraint 처리 능력 비교
    for data, label, color in [(mpc_data, "MPC", color_mpc),
                                (lqr_data, "TVLQR", color_lqr)]:
        if data and data.get("cv"):
            ws = np.abs([c[2] for c in data["cv"]])
            ax_clip.hist(ws, bins=30, alpha=0.5, label=label, color=color, density=True)
    ax_clip.axvline(1.0, color="red", ls="--", lw=1.0, label="|ω|=1 한계")
    ax_clip.set_title(f"{traj_name} — |ω| 분포 (클리핑 확인)")
    ax_clip.set_xlabel("|ω| [rad/s]"); ax_clip.set_ylabel("밀도")
    ax_clip.legend(fontsize=7); ax_clip.grid(True, alpha=0.3)


# ============================================================
# 메인
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="MPC vs TVLQR 경로별 비교 분석")
    parser.add_argument("--mpc_circle",  default=None, help="MPC 원형 bag 경로")
    parser.add_argument("--lqr_circle",  default=None, help="LQR 원형 bag 경로")
    parser.add_argument("--mpc_figure8", default=None, help="MPC 8자 bag 경로")
    parser.add_argument("--lqr_figure8", default=None, help="LQR 8자 bag 경로")
    parser.add_argument("--out", default="compare_trajectory.png",
                        help="출력 PNG 파일명")
    args = parser.parse_args()

    # 입력 확인
    has_circle  = args.mpc_circle or args.lqr_circle
    has_figure8 = args.mpc_figure8 or args.lqr_figure8
    if not has_circle and not has_figure8:
        print("오류: bag 경로를 하나 이상 지정하세요.")
        parser.print_help()
        return

    # ---- 데이터 로드 ----
    print("\n[데이터 로드 중...]")
    circle_mpc  = load_bag(args.mpc_circle)  if args.mpc_circle  else None
    circle_lqr  = load_bag(args.lqr_circle)  if args.lqr_circle  else None
    fig8_mpc    = load_bag(args.mpc_figure8) if args.mpc_figure8 else None
    fig8_lqr    = load_bag(args.lqr_figure8) if args.lqr_figure8 else None

    # ---- 통계 계산 ----
    circle_mpc_s  = calc_stats(circle_mpc,  "MPC 원형")
    circle_lqr_s  = calc_stats(circle_lqr,  "LQR 원형")
    fig8_mpc_s    = calc_stats(fig8_mpc,    "MPC 8자")
    fig8_lqr_s    = calc_stats(fig8_lqr,    "LQR 8자")

    # ---- 터미널 출력 ----
    if has_circle:
        print_table("원형 (Circle)", circle_mpc_s, circle_lqr_s)
    if has_figure8:
        print_table("8자 (Figure-8)", fig8_mpc_s, fig8_lqr_s)

    # ---- 그래프 ----
    n_trajs = (1 if has_circle else 0) + (1 if has_figure8 else 0)
    fig = plt.figure(figsize=(16, 10 * n_trajs))
    fig.suptitle("MPC vs TVLQR — 경로별 제어기 비교", fontsize=14, fontweight="bold")

    row_idx = 0
    gs = gridspec.GridSpec(n_trajs * 2, 3, hspace=0.6, wspace=0.4)

    if has_circle:
        axes = [
            fig.add_subplot(gs[row_idx * 2,     0]),
            fig.add_subplot(gs[row_idx * 2,     1]),
            fig.add_subplot(gs[row_idx * 2,     2]),
            fig.add_subplot(gs[row_idx * 2 + 1, 0]),
            fig.add_subplot(gs[row_idx * 2 + 1, 1]),
            fig.add_subplot(gs[row_idx * 2 + 1, 2]),
        ]
        plot_trajectory("원형", circle_mpc, circle_lqr,
                        axes[0], axes[1], axes[2], axes[3], axes[4], axes[5])
        row_idx += 1

    if has_figure8:
        axes = [
            fig.add_subplot(gs[row_idx * 2,     0]),
            fig.add_subplot(gs[row_idx * 2,     1]),
            fig.add_subplot(gs[row_idx * 2,     2]),
            fig.add_subplot(gs[row_idx * 2 + 1, 0]),
            fig.add_subplot(gs[row_idx * 2 + 1, 1]),
            fig.add_subplot(gs[row_idx * 2 + 1, 2]),
        ]
        plot_trajectory("8자", fig8_mpc, fig8_lqr,
                        axes[0], axes[1], axes[2], axes[3], axes[4], axes[5])

    plt.savefig(args.out, dpi=150, bbox_inches="tight")
    print(f"\n[완료] 그래프 저장: {args.out}")
    plt.show()


if __name__ == "__main__":
    main()
