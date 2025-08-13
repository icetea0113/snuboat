from pathlib import Path
import re
import pandas as pd
import subprocess
from typing import Optional, Union, List, Tuple
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
from matplotlib.ticker import FuncFormatter

# 초 단위 포맷터: 소수점 3자리까지 표시
sec_formatter = FuncFormatter(lambda x, pos: f"{x:.3f}")

def _numeric_key(name: str):
    nums = re.findall(r"\d+", name)
    return tuple(int(n) for n in nums) if nums else ()

def _dt_key(name: str) -> Tuple[int, int]:
    m = re.search(r'_(\d{4})_(\d{6})(?:$|_)', name)
    if m:
        return int(m.group(1)), int(m.group(2))
    return (0, 0)

def _pick_latest_run(base: Path, prefix: str = "FREE_RUNNING") -> Path:
    runs = [d for d in base.iterdir() if d.is_dir() and d.name.startswith(prefix)]
    if not runs:
        raise FileNotFoundError(f"{base} 아래에 '{prefix}*' 폴더가 없습니다.")
    runs.sort(key=lambda p: (_dt_key(p.name), p.stat().st_mtime))
    return runs[-1]

def _find_nonempty_csv_candidates(run_dir: Path, fname: str) -> List[Path]:
    candidates = []
    direct = run_dir / fname
    if direct.exists() and direct.is_file() and direct.stat().st_size > 0:
        candidates.append(direct)
    for p in run_dir.rglob(fname):
        if p.is_file() and p.stat().st_size > 0:
            candidates.append(p)
    return sorted(set(candidates), key=lambda p: p.stat().st_mtime, reverse=True)

def _read_csv_flex(path: Path) -> pd.DataFrame:
    try:
        return pd.read_csv(path)
    except pd.errors.EmptyDataError:
        return pd.read_csv(path, header=None, engine="python", sep=None)
    except Exception:
        return pd.read_csv(path, engine="python", sep=None)

def _convert_all_db3_in_dir(run_dir: Path, converter_cmd: str = "ros2bag-convert") -> None:
    db3s = sorted(run_dir.glob("*.db3")) or sorted(run_dir.rglob("*.db3"))
    if not db3s:
        raise FileNotFoundError(f"DB3 파일을 찾지 못했습니다: {run_dir}")
    for db3 in db3s:
        out_dir = db3.with_suffix(".db")
        if out_dir.exists() and any(out_dir.rglob("*.csv")):
            continue
        try:
            subprocess.run([converter_cmd, str(db3)], check=True, text=True, capture_output=True)
        except FileNotFoundError:
            subprocess.run(["python3", "-m", "ros2bag_convert", str(db3)], check=True, text=True)
        except subprocess.CalledProcessError as e:
            msg = e.stderr or e.stdout or str(e)
            raise RuntimeError(f"DB3 변환 실패: {db3}\n{msg}")

def _concat_paths(paths: List[Path]) -> pd.DataFrame:
    dfs = []
    last_err = None
    for p in sorted(paths, key=lambda x: x.stat().st_mtime):
        try:
            dfs.append(_read_csv_flex(p))
        except Exception as e:
            last_err = e
    if not dfs:
        raise RuntimeError(f"CSV 파싱 실패. 마지막 에러: {last_err}")
    return pd.concat(dfs, ignore_index=True, sort=False)

def _pick_best_series(run_dir: Path, name_candidates: List[str]) -> Tuple[str, List[Path]]:
    """
    여러 후보 파일명 중에서 '가장 최신 mtime'을 가진 세트(동일 이름, 여러 세그먼트)를 선택
    """
    best_name, best_paths, best_mtime = None, [], -1.0
    for fname in name_candidates:
        paths = _find_nonempty_csv_candidates(run_dir, fname)
        if paths:
            latest = max(p.stat().st_mtime for p in paths)
            if latest > best_mtime:
                best_name, best_paths, best_mtime = fname, paths, latest
    return best_name, best_paths

def _load_first_of(run_dir: Path, name_candidates: List[str], convert_if_needed: bool) -> pd.DataFrame:
    name, paths = _pick_best_series(run_dir, name_candidates)
    if not paths and convert_if_needed:
        _convert_all_db3_in_dir(run_dir)
        name, paths = _pick_best_series(run_dir, name_candidates)
    if not paths:
        raise FileNotFoundError(f"{' or '.join(name_candidates)} 파일을 찾지 못했습니다.")
    return _concat_paths(paths)

def _abs_seconds(df: pd.DataFrame) -> np.ndarray:
    """절대초 배열 리턴(항상 np.ndarray)"""
    if "time" in df.columns:
        t = pd.to_datetime(df["time"], errors="coerce")
        if t.notna().any():
            return t.astype("int64").to_numpy() / 1e9  # ns -> s (np.ndarray)
    sec  = pd.to_numeric(df["tick_secs"],  errors="coerce").to_numpy() if "tick_secs"  in df.columns else None
    nsec = pd.to_numeric(df["tick_nsecs"], errors="coerce").to_numpy() if "tick_nsecs" in df.columns else None
    if sec is not None:
        return sec + (0.0 if nsec is None else nsec * 1e-9)
    cand = next((c for c in ["tick", "secs"] if c in df.columns), None)
    if cand:
        return pd.to_numeric(df[cand], errors="coerce").to_numpy()
    return np.arange(len(df), dtype=float)

def _abs_nanos(df: pd.DataFrame) -> np.ndarray:
    """절대시간을 나노초(int64)로 반환. 정렬/차이 계산에 안전.
    - tick_secs + tick_nsecs가 둘 다 있으면 이를 최우선으로 사용 (정밀/일관성)
    - 없으면 time 문자열을 파싱하여 ns로 변환
    - 모두 없으면 0..N-1의 증가 정수 반환
    """
    if {"tick_secs", "tick_nsecs"}.issubset(df.columns):
        sec  = pd.to_numeric(df["tick_secs"], errors="coerce").fillna(0).astype("int64").to_numpy()
        nsec = pd.to_numeric(df["tick_nsecs"], errors="coerce").fillna(0).astype("int64").to_numpy()
        return sec * 1_000_000_000 + nsec
    if "time" in df.columns:
        t = pd.to_datetime(df["time"], errors="coerce")
        if t.notna().any():
            # NaT는 0으로 대체해도 정렬에는 문제 없음 (mergesort로 원래 상대 순서 유지)
            ns = t.view("int64").fillna(0).to_numpy()
            return ns
    # fallback
    return np.arange(len(df), dtype="int64")

def get_run_dir(base_dir: str = "src", prefix: str = "FREE_RUNNING", run_name: Optional[str] = None) -> Path:
    """
    load_data와 동일한 규칙으로 실제 사용된 run 폴더 경로를 반환
    """
    base = Path(base_dir).expanduser().resolve()
    if not base.exists():
        raise FileNotFoundError(f"경로가 존재하지 않습니다: {base}")
    if run_name:
        rd = base / run_name
        if not rd.exists():
            # 주석 해제하면: 지정 폴더 없을 때 최신으로 대체
            # rd = _pick_latest_run(base, prefix=prefix)
            raise FileNotFoundError(f"실행 폴더가 없습니다: {rd}")
        return rd
    return _pick_latest_run(base, prefix=prefix)

def load_data(
    filename: Optional[Union[str, List[str]]] = None,
    base_dir: str = "src",
    run_name: Optional[str] = None,
    prefix: str = "FREE_RUNNING",
    convert_if_needed: bool = True,
) -> Union[pd.DataFrame, Tuple[pd.DataFrame, ...]]:
    """
    - run_name이 None이면 base_dir 아래에서 prefix로 시작하는 실행 폴더 중 최신 선택.
    - filename이 None이면 (sensor, ctrl_cmd, ctrl_fb) 튜플 반환:
        sensor   = sensor.csv
        ctrl_cmd = ctrl_cmd_sils.csv | ctrl_cmd_boat.csv | ctrl_cmd.csv
        ctrl_fb  = sils_motor_fb_data.csv | ctrl_fb_boat.csv | ctrl_fb.csv
      (여러 .db3 세그먼트면 concat)
    - filename이 str이면 해당 파일만 DataFrame 반환 (.csv 자동 보정)
    - filename이 list면 해당 순서대로 튜플 반환
    """
    base = Path(base_dir).expanduser().resolve()
    if not base.exists():
        raise FileNotFoundError(f"경로가 존재하지 않습니다: {base}")

    run_dir = (base / run_name) if run_name else _pick_latest_run(base, prefix=prefix)
    if not run_dir.exists():
        raise FileNotFoundError(f"실행 폴더가 없습니다: {run_dir}")

    if filename is None:
        sensor_df   = _load_first_of(run_dir, ["sensor.csv"], convert_if_needed)
        ctrl_cmd_df = _load_first_of(run_dir, ["ctrl_cmd_sils.csv", "ctrl_cmd_boat.csv", "ctrl_cmd.csv"], convert_if_needed)
        ctrl_fb_df  = _load_first_of(run_dir, ["sils_motor_fb_data.csv", "ctrl_fb_boat.csv", "ctrl_fb.csv"], convert_if_needed)
        return sensor_df, ctrl_cmd_df, ctrl_fb_df
    else:
        if isinstance(filename, str):
            fname = filename if filename.lower().endswith(".csv") else f"{filename}.csv"
            # 단일 파일 요청은 기존 로직 유지(정확 매칭)
            cands = _find_nonempty_csv_candidates(run_dir, fname)
            if not cands and convert_if_needed:
                _convert_all_db3_in_dir(run_dir)
                cands = _find_nonempty_csv_candidates(run_dir, fname)
            if not cands:
                raise FileNotFoundError(f"{fname} 파일을 찾지 못했습니다.")
            return _concat_paths(cands)
        else:
            targets = [f if f.lower().endswith(".csv") else f"{f}.csv" for f in filename]
            dfs = []
            for t in targets:
                cands = _find_nonempty_csv_candidates(run_dir, t)
                if not cands and convert_if_needed:
                    _convert_all_db3_in_dir(run_dir)
                    cands = _find_nonempty_csv_candidates(run_dir, t)
                if not cands:
                    raise FileNotFoundError(f"{t} 파일을 찾지 못했습니다.")
                dfs.append(_concat_paths(cands))
            return tuple(dfs)
        
def _pick_col(cols, candidates, default=None):
    return next((c for c in candidates if c in cols), default)

def _ensure_numeric(df, cols):
    for c in cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors='coerce')

def _get_run_dir_for_save(base_dir="~/snuboat/ws/src", prefix="FREE_RUNNING", run_name=None):
    base = Path(base_dir).expanduser().resolve()
    if run_name:
        return base / run_name
    return _pick_latest_run(base, prefix=prefix)

# 사용 예시
def main():
    df_sensor, df_ctrl, df_motor = load_data(
        base_dir="~/snuboat/ws/src",
        run_name=None,
        prefix="FREE_RUNNING",
        convert_if_needed=True
    )

    df_sensor = df_sensor[11:]  # filter로 인해서 앞에 초기화된 부분 자르기

    run_dir = _get_run_dir_for_save(base_dir="~/snuboat/ws/src", prefix="FREE_RUNNING", run_name=None)

    # 컬럼 확인/정리
    cols = set(df_sensor.columns)
    status_col = _pick_col(cols, ["status_key", "status", "Status", "STATUS", "key", "state", "flags"])
    pose_cols = ["pose_0", "pose_1", "pose_5"]
    vel_cols  = ["vel_0", "vel_1", "vel_5"]

    _ensure_numeric(df_sensor, pose_cols + vel_cols + ["tick_secs","tick_nsecs"])

    # ----------------------------
    # (중요) 절대시간 기준 정렬 + 공통 x축 생성  ✅
    # ----------------------------
    ts_ns = _abs_nanos(df_sensor)                                  # int64 ns
    order = np.argsort(ts_ns, kind="mergesort")
    df_sensor = df_sensor.iloc[order].reset_index(drop=True)
    ts_ns = ts_ns[order]
    x = (ts_ns - ts_ns[0]) * 1e-9                                   # seconds
    x_label = "time [s]"

    # ✅ tick 단일 컬럼 정렬은 사용하지 않습니다 (오히려 순서를 꼬이게 함)
    # df_sensor = df_sensor.sort_values(by=tick)  # <-- 삭제

    # 1) status key의 마지막 자리가 1인 인덱스만 골라 trajectory (pose_1 vs pose_0)
    if status_col is not None:
        mask = ((df_sensor[status_col].astype("Int64") % 10) == 1).fillna(False)
        df_traj = df_sensor[mask].copy()
        if df_traj.empty:
            df_traj = df_sensor.copy()
    else:
        df_traj = df_sensor.copy()

    # ---- Fig 1: Trajectory (pose_1, pose_0) ----
    fig1, ax1 = plt.subplots(1, 1, figsize=(6, 6), tight_layout=True)
    if {"pose_0","pose_1"}.issubset(df_sensor.columns):
        ax1.plot(df_traj["pose_1"].to_numpy(),
                 df_traj["pose_0"].to_numpy(),
                 linewidth=1)
        ax1.set_xlabel("Y [m]")
        ax1.set_ylabel("X [m]")
        ax1.set_title("Trajectory (filtered by status%10==1)" if status_col else "Trajectory")
        ax1.axis("equal")
    else:
        ax1.text(0.5, 0.5, "pose_0/pose_1 column missing", ha="center", va="center")
    fig1.savefig(run_dir / "fig1_trajectory.png", dpi=300)
    plt.close(fig1)

    # ---- Fig 2: pose_0, pose_1, pose_5 vs time ----
    fig2, axes2 = plt.subplots(3, 1, figsize=(10, 7), sharex=True, tight_layout=True)
    labels_pose = ["X [m]", "Y [m]", "psi [deg]"]
    for ax, col, lab in zip(axes2, pose_cols, labels_pose):
        if col in df_sensor.columns:
            if lab == "psi [deg]":
                ax.plot(x, df_sensor[col].to_numpy()*180/np.pi, linewidth=1)
            else:
                ax.plot(x, df_sensor[col].to_numpy(), linewidth=1)
            ax.set_ylabel(lab)
        else:
            ax.text(0.5, 0.5, f"{col} missing", ha="center", va="center")
        ax.xaxis.set_major_formatter(sec_formatter)
    axes2[-1].set_xlabel(x_label)
    fig2.savefig(run_dir / "fig2_pose_timeseries.png", dpi=300)
    plt.close(fig2)

    # ---- Fig 3: vel_0, vel_1, vel_5 vs time ----
    fig3, axes3 = plt.subplots(3, 1, figsize=(10, 7), sharex=True, tight_layout=True)
    series_list = [("vel_0", "u [m/s]"),
                   ("vel_1", "v [m/s]"),
                   ("vel_5", "r [rad/s]")]
    for i, (col, lab) in enumerate(series_list):
        if col in df_sensor.columns:
            axes3[i].plot(x, df_sensor[col].to_numpy(), linewidth=1)
            axes3[i].set_ylabel(lab)
        else:
            axes3[i].text(0.5, 0.5, f"{col} missing", ha="center", va="center")
        axes3[i].xaxis.set_major_formatter(sec_formatter)
    axes3[-1].set_xlabel(x_label)
    fig3.savefig(run_dir / "fig3_velocity_timeseries.png", dpi=300)
    plt.close(fig3)

    print(f"[saved] {run_dir/'fig1_trajectory.png'}")
    print(f"[saved] {run_dir/'fig2_pose_timeseries.png'}")
    print(f"[saved] {run_dir/'fig3_velocity_timeseries.png'}")

    # === Fig 4: ctrl_cmd vs ctrl_fb (subplot 2개) ===
    df_ctrl_cmd = df_ctrl
    df_ctrl_fb  = df_motor

    # 시간축 만들기 (절대초 → 공통 0초 기준, 모두 np.ndarray)  ✅ 두 DF 모두 절대시간 기준 재정렬
    cmd_ns = _abs_nanos(df_ctrl_cmd)
    fb_ns  = _abs_nanos(df_ctrl_fb)

    order_cmd = np.argsort(cmd_ns, kind="mergesort")
    order_fb  = np.argsort(fb_ns,  kind="mergesort")

    df_ctrl_cmd = df_ctrl_cmd.iloc[order_cmd].reset_index(drop=True)
    df_ctrl_fb  = df_ctrl_fb.iloc[order_fb].reset_index(drop=True)

    cmd_ns = cmd_ns[order_cmd]
    fb_ns  = fb_ns[order_fb]

    t0_ns = np.nanmin([cmd_ns[0], fb_ns[0]]).astype("int64")
    t_cmd = (cmd_ns - t0_ns) * 1e-9
    t_fb  = (fb_ns  - t0_ns) * 1e-9

    def arr(df, col):
        return pd.to_numeric(df[col], errors="coerce").to_numpy() if col in df.columns else None

    c0_cmd = arr(df_ctrl_cmd, "ctrl_0")
    c1_cmd = arr(df_ctrl_cmd, "ctrl_1")
    c2_cmd = arr(df_ctrl_cmd, "ctrl_2")
    c3_cmd = arr(df_ctrl_cmd, "ctrl_3")

    c0_fb  = arr(df_ctrl_fb,  "ctrl_0")
    c1_fb  = arr(df_ctrl_fb,  "ctrl_1")
    c2_fb  = arr(df_ctrl_fb,  "ctrl_2")
    c3_fb  = arr(df_ctrl_fb,  "ctrl_3")

    fig4, axes4 = plt.subplots(2, 1, figsize=(10, 6), sharex=True, tight_layout=True)

    # Subplot 1: ctrl_0, ctrl_1
    if c0_cmd is not None: axes4[0].plot(t_cmd, c0_cmd, linewidth=1.2, linestyle="-", label="cmd PORT")
    if c0_fb  is not None: axes4[0].plot(t_fb,  c0_fb,  linewidth=1.2, linestyle="--", label="fb PORT")
    if c1_cmd is not None: axes4[0].plot(t_cmd, c1_cmd, linewidth=1.2, linestyle="-", label="cmd STBD")
    if c1_fb  is not None: axes4[0].plot(t_fb,  c1_fb,  linewidth=1.2, linestyle="--", label="fb STBD")
    axes4[0].set_ylabel("n_p_cmd / n_s_cmd")
    axes4[0].legend(loc="best", fontsize=9)

    # Subplot 2: ctrl_2, ctrl_3
    if c2_cmd is not None: axes4[1].plot(t_cmd, c2_cmd, linewidth=1.2, linestyle="-", label="cmd PORT")
    if c2_fb  is not None: axes4[1].plot(t_fb,  c2_fb,  linewidth=1.2, linestyle="--", label="fb PORT")
    if c3_cmd is not None: axes4[1].plot(t_cmd, c3_cmd, linewidth=1.2, linestyle="-", label="cmd STBD")
    if c3_fb  is not None: axes4[1].plot(t_fb,  c3_fb,  linewidth=1.2, linestyle="--", label="fb STBD")
    axes4[1].set_ylabel("del_p_cmd / del_s_cmd")

    for ax in axes4:
        ax.xaxis.set_major_formatter(sec_formatter)
    axes4[-1].set_xlabel("time [s]")

    fig4.suptitle("ctrl_cmd (solid) vs ctrl_fb (dashed)", y=1.02)
    fig4.savefig(run_dir / "fig4_ctrl_timeseries.png", dpi=300)
    plt.close(fig4)
    print(f"[saved] {run_dir/'fig4_ctrl_timeseries.png'}")

if __name__ == "__main__":
    main()