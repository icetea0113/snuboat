# snunav_pkg/snunav_pkg/utils/plot.py
import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from .data import load_data, get_run_dir  # utils.data.py의 함수 사용

def _pick_col(cols, candidates, default=None):
    return next((c for c in candidates if c in cols), default)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base-dir", default="~/snuboat/ws/src")
    ap.add_argument("--prefix",   default="FREE_RUNNING")
    ap.add_argument("--run-name", default=None)
    ap.add_argument("--dpi", type=int, default=300)
    args = ap.parse_args()

    # 데이터 로드 (sensor만 쓰지만 load_data는 (sensor, ctrl_cmd, ctrl_fb) 반환)
    df_sensor, _, _ = load_data(
        base_dir=args.base_dir,
        run_name=args.run_name,
        prefix=args.prefix,
        convert_if_needed=True
    )

    # 저장 경로 = 실제 run 폴더
    run_dir = get_run_dir(args.base_dir, args.prefix, args.run_name)

    # --- 컬럼 준비 ---
    cols = set(df_sensor.columns)
    # x축은 tick_secs가 우선
    tick = _pick_col(cols, ["tick_secs", "tick_sec", "tick", "time", "secs"])
    if tick is None:
        df_sensor = df_sensor.reset_index().rename(columns={"index": "tick_index"})
        tick = "tick_index"

    # 상태 컬럼(선택적)
    status_col = _pick_col(cols, ["status_key", "status", "Status", "STATUS", "key", "state", "flags"])

    # 숫자형 변환 + 정렬
    for c in [tick, "pose_0", "pose_1", "pose_5", "vel_0", "vel_1", "vel_6"]:
        if c in df_sensor.columns:
            df_sensor[c] = pd.to_numeric(df_sensor[c], errors="coerce")
    df_sensor = df_sensor.sort_values(by=tick)

    # ---------------- Fig 1: Trajectory (pose_1 vs pose_0) ----------------
    # status 마지막 자리==1 필터
    if status_col and status_col in df_sensor.columns:
        mask = (df_sensor[status_col].astype("Int64") % 10) == 1
        df_traj = df_sensor[mask] if mask.any() else df_sensor
    else:
        df_traj = df_sensor

    fig1, ax1 = plt.subplots(1, 1, figsize=(6, 6), tight_layout=True)
    if {"pose_0", "pose_1"}.issubset(df_sensor.columns):
        ax1.plot(df_traj["pose_0"], df_traj["pose_1"], linewidth=1)
        ax1.set_xlabel("pose_0 [m]")
        ax1.set_ylabel("pose_1 [m]")
        ax1.set_title("Trajectory (status%10==1 filtered)" if status_col else "Trajectory")
        ax1.axis("equal")
    else:
        ax1.text(0.5, 0.5, "pose_0/pose_1 missing", ha="center", va="center")
    fig1.savefig(run_dir / "fig1_trajectory.png", dpi=args.dpi)
    plt.close(fig1)

    # ---------------- Fig 2: pose_0, pose_1, pose_5 vs tick (3x1) ----------------
    fig2, axes2 = plt.subplots(3, 1, figsize=(10, 7), sharex=True, tight_layout=True)
    for ax, (col, lab) in zip(
        axes2,
        [("pose_0", "pose_0 [m]"), ("pose_1", "pose_1 [m]"), ("pose_5", "pose_5 [rad]")]
    ):
        if col in df_sensor.columns:
            ax.plot(df_sensor[tick], df_sensor[col], linewidth=1)
            ax.set_ylabel(lab)
        else:
            ax.text(0.5, 0.5, f"{col} missing", ha="center", va="center")
    axes2[-1].set_xlabel(tick)
    fig2.suptitle("Pose time series", y=1.02)
    fig2.savefig(run_dir / "fig2_pose_timeseries.png", dpi=args.dpi)
    plt.close(fig2)

    # ---------------- Fig 3: vel_0, vel_1, vel_6(deg/s) vs tick (3x1) ----------------
    fig3, axes3 = plt.subplots(3, 1, figsize=(10, 7), sharex=True, tight_layout=True)
    for ax, (col, lab) in zip(
        axes3,
        [("vel_0", "vel_0 [m/s]"), ("vel_1", "vel_1 [m/s]"), ("vel_6", "vel_6 [deg/s]")]
    ):
        if col == "vel_6" and "vel_6" in df_sensor.columns:
            ax.plot(df_sensor[tick], df_sensor["vel_6"] * (180.0 / np.pi), linewidth=1)
            ax.set_ylabel(lab)
        elif col in df_sensor.columns:
            ax.plot(df_sensor[tick], df_sensor[col], linewidth=1)
            ax.set_ylabel(lab)
        else:
            ax.text(0.5, 0.5, f"{col} missing", ha="center", va="center")
    axes3[-1].set_xlabel(tick)
    fig3.suptitle("Velocity time series", y=1.02)
    fig3.savefig(run_dir / "fig3_velocity_timeseries.png", dpi=args.dpi)
    plt.close(fig3)

    print(f"[saved] {run_dir / 'fig1_trajectory.png'}")
    print(f"[saved] {run_dir / 'fig2_pose_timeseries.png'}")
    print(f"[saved] {run_dir / 'fig3_velocity_timeseries.png'}")

if __name__ == "__main__":
    main()
