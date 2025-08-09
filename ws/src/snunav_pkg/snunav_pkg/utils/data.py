from pathlib import Path
import re
import pandas as pd
import subprocess
from typing import Optional, Union, List, Tuple

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

# 사용 예시
def main():
    # 예: ~/snuboat/ws/src/FREE_RUNNING_TURNING_0809_085021 폴더에서
    # *.db3 → CSV 자동 변환 후 (sensor, ctrl_cmd_sils, sils_motor_fb_data) 반환
    df_sensor, df_ctrl, df_motor = load_data(
        base_dir="~/snuboat/ws/src",
        run_name=None,
        prefix="FREE_RUNNING",
        convert_if_needed=True
    )
    print("sensor.csv head:");          print(df_sensor.head())
    print("\nctrl_cmd_sils.csv head:"); print(df_ctrl.head())
    print("\nsils_motor_fb_data.csv head:"); print(df_motor.head())

if __name__ == "__main__":
    main()
