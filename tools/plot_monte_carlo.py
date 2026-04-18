"""
AMR Monte Carlo 결과 통합 대시보드 시각화 스크립트 (mc_dashboard.py)
====================================================================
목적:
    monte_carlo.py가 생성한 결과 CSV를 읽어 KPI 카드, 분포도,
    trial별 시계열을 하나의 대시보드 PNG로 출력한다.
    포트폴리오 README 및 발표 자료에 삽입하기 위한 용도로 설계되었다.

입력:
    - monte_carlo_results.csv : monte_carlo.py 출력 CSV
      필수 컬럼:
        trial           : trial 번호 (1~N)
        result          : "SUCCESS" | "COLLISION"
        mean_rmse_m     : trial 내 평균 tracking RMSE (m)
        min_clearance_m : trial 내 최소 장애물 거리 (m), 미측정 시 9999.0
        recovery_time_sec : SAFE_STOP → NORMAL 재획득 시간 (s)

출력 (--output-dir 디렉터리):
    - mc_dashboard_all_in_one.png : 4행 통합 대시보드
        1행: KPI 요약 카드 (Success Rate / Collision Rate / Recovery Time / Mean RMSE)
        2행: Mean RMSE 분포 히스토그램 + Recovery Time 분포 히스토그램
        3행: Trial별 Mean RMSE 시계열 (성공=초록, 충돌=빨강)
        4행: Trial별 Minimum Clearance 시계열
    - mc_summary_stats.csv : 집계 통계 수치 (success_rate, rmse_mean 등)

사용법:
    # 기본 (현재 디렉터리의 monte_carlo_results.csv 사용)
    python3 mc_dashboard.py

    # 경로 직접 지정
    python3 mc_dashboard.py \\
        --input  ~/amr_ws/monte_carlo_results.csv \\
        --output-dir ~/amr_ws/results/

비고:
    - min_clearance_m = 9999.0 은 장애물 미감지 sentinel 값으로,
      플롯 시 NaN으로 처리하여 시계열에서 제외한다.
    - 의존 패키지: numpy, pandas, matplotlib, seaborn
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.gridspec import GridSpec


def setup_style():
    sns.set_theme(style="whitegrid", context="talk")
    plt.rcParams.update({
        "figure.dpi": 140,
        "savefig.dpi": 220,
        "axes.titlesize": 15,
        "axes.labelsize": 12,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        "legend.fontsize": 10,
        "font.family": "DejaVu Sans",
    })


def load_data(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path)

    # 9999.0은 미측정 sentinel로 간주함
    df["min_clearance_plot"] = df["min_clearance_m"].replace(9999.0, np.nan)
    df["is_success"] = df["result"].astype(str).str.upper().eq("SUCCESS")
    df["is_collision"] = df["result"].astype(str).str.upper().eq("COLLISION")
    df["recovery_pass"] = df["recovery_time_sec"] <= 5.0
    return df


def compute_summary(df: pd.DataFrame) -> pd.DataFrame:
    valid_clear = df.loc[df["min_clearance_m"] != 9999.0, "min_clearance_m"]

    summary = {
        "total_trials": len(df),
        "success_count": int(df["is_success"].sum()),
        "collision_count": int(df["is_collision"].sum()),
        "success_rate_pct": round(100.0 * df["is_success"].mean(), 2),
        "collision_rate_pct": round(100.0 * df["is_collision"].mean(), 2),
        "rmse_mean_m": round(df["mean_rmse_m"].mean(), 4),
        "rmse_p95_m": round(df["mean_rmse_m"].quantile(0.95), 4),
        "recovery_mean_sec": round(df["recovery_time_sec"].mean(), 3),
        "recovery_p95_sec": round(df["recovery_time_sec"].quantile(0.95), 3),
        "min_clearance_valid_mean_m": round(valid_clear.mean(), 4) if not valid_clear.empty else np.nan,
        "min_clearance_valid_min_m": round(valid_clear.min(), 4) if not valid_clear.empty else np.nan,
    }
    return pd.DataFrame([summary])


def draw_kpi_card(ax, title: str, value: str, subtitle: str, color: str):
    ax.set_facecolor("#F7F7F7")
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.text(0.05, 0.78, title, fontsize=14, fontweight="bold", color="#333333", transform=ax.transAxes)
    ax.text(0.05, 0.40, value, fontsize=24, fontweight="bold", color=color, transform=ax.transAxes)
    ax.text(0.05, 0.12, subtitle, fontsize=10.5, color="#666666", transform=ax.transAxes)



def create_combined_figure(df: pd.DataFrame, outdir: Path):
    success_rate = 100.0 * df["is_success"].mean()
    collision_rate = 100.0 * df["is_collision"].mean()
    recovery_mean = df["recovery_time_sec"].mean()
    rmse_mean = df["mean_rmse_m"].mean()

    fig = plt.figure(figsize=(20, 24))
    gs = GridSpec(4, 4, figure=fig, height_ratios=[1.1, 1.5, 1.6, 1.6], hspace=0.42, wspace=0.28)

    # 1행: KPI 카드 4개 배치함
    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[0, 2])
    ax4 = fig.add_subplot(gs[0, 3])

    draw_kpi_card(ax1, "Success Rate", f"{success_rate:.1f}%", "Target > 90%", "#2E8B57")
    draw_kpi_card(ax2, "Collision Rate", f"{collision_rate:.1f}%", "Target = 0%", "#C0392B")
    draw_kpi_card(ax3, "Recovery Time", f"{recovery_mean:.2f} s", "Mean of 50 trials", "#E67E22")
    draw_kpi_card(ax4, "Mean RMSE", f"{rmse_mean:.3f} m", "Average tracking error", "#1F77B4")

    # 2행: 분포도 2개를 넓게 배치함
    ax5 = fig.add_subplot(gs[1, 0:2])
    ax6 = fig.add_subplot(gs[1, 2:4])

    sns.histplot(df["mean_rmse_m"], bins=12, kde=True, color="#8E44AD", ax=ax5)
    ax5.set_title("Mean RMSE Distribution", fontweight="bold")
    ax5.set_xlabel("Mean RMSE [m]")
    ax5.set_ylabel("Count")
    ax5.axvline(df["mean_rmse_m"].mean(), color="#2C3E50", linestyle="--", linewidth=1.5, label="Mean")
    ax5.legend()

    sns.histplot(df["recovery_time_sec"], bins=12, kde=True, color="#E67E22", ax=ax6)
    ax6.set_title("Recovery Time Distribution", fontweight="bold")
    ax6.set_xlabel("Recovery Time [s]")
    ax6.set_ylabel("Count")
    ax6.axvline(df["recovery_time_sec"].mean(), color="#2C3E50", linestyle="--", linewidth=1.5, label="Mean")
    ax6.axvline(5.0, color="#C0392B", linestyle=":", linewidth=1.8, label="5 s target")
    ax6.legend()

    # 3행: trial별 RMSE 추이 그림임
    ax7 = fig.add_subplot(gs[2, :])
    line_colors = df["result"].map({"SUCCESS": "#2E8B57", "COLLISION": "#C0392B"}).fillna("#7F8C8D")
    ax7.plot(df["trial"], df["mean_rmse_m"], color="#1F77B4", linewidth=2.0, alpha=0.9)
    ax7.scatter(df["trial"], df["mean_rmse_m"], c=line_colors, s=60, edgecolors="white", linewidths=0.7, zorder=3)
    ax7.set_title("Trial-wise Mean RMSE", fontweight="bold")
    ax7.set_xlabel("Trial")
    ax7.set_ylabel("Mean RMSE [m]")
    ax7.set_xlim(df["trial"].min() - 0.5, df["trial"].max() + 0.5)
    ax7.axhline(df["mean_rmse_m"].mean(), color="#2C3E50", linestyle="--", linewidth=1.5, label="50-Trial Average RMSE")
    ax7.legend(loc="upper right")

    # 4행: trial별 최소 이격거리 추이 그림임
    ax8 = fig.add_subplot(gs[3, :])
    ax8.plot(df["trial"], df["min_clearance_plot"], color="#16A085", linewidth=2.0, alpha=0.9)
    ax8.scatter(df["trial"], df["min_clearance_plot"], c=line_colors, s=60, edgecolors="white", linewidths=0.7, zorder=3)
    ax8.set_title("Trial-wise Minimum Clearance", fontweight="bold")
    ax8.set_xlabel("Trial")
    ax8.set_ylabel("Minimum Clearance [m]")
    ax8.set_xlim(df["trial"].min() - 0.5, df["trial"].max() + 0.5)
    ax8.axhline(0.2, color="#C0392B", linestyle=":", linewidth=1.8, label="0.2 m target")
    ax8.legend(loc="upper right")

    fig.suptitle("AMR Monte Carlo Simulation Dashboard", fontsize=24, fontweight="bold", y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.985])
    fig.savefig(outdir / "mc_dashboard_all_in_one.png", bbox_inches="tight")
    plt.close(fig)



def main():
    parser = argparse.ArgumentParser(description="Create an all-in-one Monte Carlo visualization dashboard")
    parser.add_argument("--input", type=str, default="monte_carlo_results.csv", help="Input CSV path")
    parser.add_argument("--output-dir", type=str, default="results", help="Output directory")
    args = parser.parse_args()

    input_path = Path(args.input)
    outdir = Path(args.output_dir)
    outdir.mkdir(parents=True, exist_ok=True)

    setup_style()
    df = load_data(input_path)
    summary_df = compute_summary(df)

    summary_df.to_csv(outdir / "mc_summary_stats.csv", index=False)
    create_combined_figure(df, outdir)

    print(f"Saved outputs to: {outdir.resolve()}")
    print("Created: mc_dashboard_all_in_one.png, mc_summary_stats.csv")


if __name__ == "__main__":
    main()