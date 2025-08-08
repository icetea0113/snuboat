from pathlib import Path
import re
import pandas as pd
from typing import Optional, Union, Dict, List, Tuple

def _numeric_key(name: str):
    nums = re.findall(r"\d+", name)
    return tuple(int(n) for n in nums) if nums else ()

def _pick_latest_run(base: Path, prefix: str = "FREE_RUNNING") -> Path:
    runs = [d for d in base.iterdir() if d.is_dir() and d.name.startswith(prefix)]
    if not runs:
        raise FileNotFoundError(f"{base} 아래에 '{prefix}*' 폴더가 없습니다.")
    runs.sort(key=lambda p: _numeric_key(p.name))
    return runs[-1]

def _find_nonempty_csv_candidates(run_dir: Path, fname: str) -> List[Path]:
    candidates = []
    direct = run_dir / fname
    if direct.exists() and direct.is_file() and direct.stat().st_size > 0:
        candidates.append(direct)
    for p in run_dir.rglob(fname):
        if p.is_file() and p.stat().st_size > 0:
            candidates.append(p)
    candidates = sorted(set(candidates), key=lambda p: p.stat().st_mtime, reverse=True)
    return candidates

def _read_csv_flex(path: Path) -> pd.DataFrame:
    try:
        return pd.read_csv(path)
    except pd.errors.EmptyDataError:
        return pd.read_csv(path, header=None, engine="python", sep=None)
    except Exception:
        return pd.read_csv(path, engine="python", sep=None)

def _load_one(run_dir: Path, fname: str) -> pd.DataFrame:
    cands = _find_nonempty_csv_candidates(run_dir, fname)
    if not cands:
        tried = [str(p) for p in run_dir.rglob(fname)]
        where = tried or [str(run_dir / fname)]
        raise FileNotFoundError(f"{fname}를 찾지 못했습니다. 시도한 위치 예: {where[0]}")
    last_err = None
    for p in cands:
        try:
            return _read_csv_flex(p)
        except Exception as e:
            last_err = e
    raise RuntimeError(f"{fname} 파싱 실패. 마지막 에러: {last_err}")

def load_data(
    filename: Optional[Union[str, List[str]]] = None,
    base_dir: str = "src",
    run_name: Optional[str] = None,
    prefix: str = "FREE_RUNNING",
) -> Union[pd.DataFrame, Tuple[pd.DataFrame, ...]]:
    """
    - run_name이 None이면 base_dir/src 아래에서 prefix로 시작하는 실행 폴더 중
      숫자 토큰 기준 '가장 최신' 폴더 선택.
    - filename이 None이면 (df1, df2) 튜플 반환:
        df1 = sensor.csv
        df2 = ctrl_cmd_sils.csv
    - filename이 str이면 해당 파일만 DataFrame 반환 (.csv 자동 보정)
    - filename이 list면 해당 순서대로 튜플 반환
    """
    base = Path(base_dir)
    if not base.exists():
        raise FileNotFoundError(f"경로가 존재하지 않습니다: {base.resolve()}")

    run_dir = base / run_name if run_name else _pick_latest_run(base, prefix=prefix)
    if not run_dir.exists():
        raise FileNotFoundError(f"실행 폴더가 없습니다: {run_dir}")

    if filename is None:
        targets = ["sensor.csv", "ctrl_cmd_sils.csv"]  # 반환 순서 고정: (sensor, ctrl_cmd_sils)
        dfs = tuple(_load_one(run_dir, t) for t in targets)
        return dfs
    else:
        if isinstance(filename, str):
            fname = filename if filename.lower().endswith(".csv") else f"{filename}.csv"
            return _load_one(run_dir, fname)
        else:
            targets = [f if f.lower().endswith(".csv") else f"{f}.csv" for f in filename]
            dfs = tuple(_load_one(run_dir, t) for t in targets)
            return dfs

# 사용 예시
def main():
    df1, df2 = load_data()  # df1=sensor.csv, df2=ctrl_cmd_sils.csv
    print("sensor.csv head:")
    print(df1.head())
    print("\nctrl_cmd_sils.csv head:")
    print(df2.head())

if __name__ == "__main__":
    main()
