#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import csv
from pathlib import Path
from typing import Any, Dict, List
import json
import re
import argparse
import datetime as dt
import json
import sys
import csv
from pathlib import Path
from typing import Any, Dict, List, Optional
from pydantic import BaseModel, Field, ValidationError, field_validator, ConfigDict
import git

from pathlib import Path

def sniff_dialect(path: Path) -> csv.Dialect:
    sample = path.read_text(encoding="utf-8", errors="ignore")[:4096]
    try:
        return csv.Sniffer().sniff(sample)
    except Exception:
        # 失敗したら標準CSV（カンマ区切り）
        class _D(csv.Dialect):
            delimiter = ","
            quotechar = '"'
            doublequote = True
            skipinitialspace = False
            lineterminator = "\n"
            quoting = csv.QUOTE_MINIMAL
        return _D()

def parse_value(v: str) -> Any:
    v = v.strip()
    if v == "":
        return ""
    # 真偽
    if v.lower() in ("true", "false"):
        return v.lower() == "true"
    # 数値
    try:
        if re.match(r"^-?\d+\.\d+$", v):
            return float(v)
        if re.match(r"^-?\d+$", v):
            return int(v)
    except Exception:
        pass
    # JSONっぽい
    if (v.startswith("{") and v.endswith("}")) or (v.startswith("[") and v.endswith("]")):
        try:
            return json.loads(v)
        except Exception:
            return v
    return v

def to_toml_literal(v: Any) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return str(v)
    if isinstance(v, list):
        return "[" + ", ".join(to_toml_literal(x) for x in v) + "]"
    if isinstance(v, dict):
        # 手早く JSON で埋める（必要なら inline table に変更も可）
        return json.dumps(v, ensure_ascii=False)
    # 文字列
    s = str(v).replace("\\", "\\\\").replace('"', '\\"')
    return f"\"{s}\""

def read_two_header_csv(csv_path: Path) -> tuple[list[str], list[str], list[dict[str, str]]]:
    """
    1行目: 日本語ヘッダ
    2行目: 英語キー
    3行目〜: データ
    を読み込む
    """
    dialect = sniff_dialect(csv_path)
    with csv_path.open(encoding="utf-8", newline="") as f:
        rdr = csv.reader(f, dialect)
        jp_header = [h.strip() for h in next(rdr)]
        en_header = [h.strip() for h in next(rdr)]
        rows: list[dict[str, str]] = []
        for raw in rdr:
            # 行長が短い/長いのズレを吸収
            padded = (raw + [""] * len(en_header))[:len(en_header)]
            row = {en_header[i]: (padded[i] or "").strip() for i in range(len(en_header))}
            # 空行はスキップ（scenario_id など主キーが空なら飛ばす）
            if any(v for v in row.values()):
                rows.append(row)
    return jp_header, en_header, rows

def build_jp_map(jp_header: List[str], en_header: List[str]) -> Dict[str, str]:
    """英語キー → 日本語名の辞書を作成"""
    n = min(len(jp_header), len(en_header))
    return {en_header[i]: jp_header[i] for i in range(n)}

def sanitize_filename(s: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", s)

def row_to_toml_with_jp_comments(row: Dict[str, str], jp_map: Dict[str, str]) -> str:
    """
    1行のデータを TOML 文にする。
    各キーの直前に「# 日本語名」を付ける。
    """
    lines: List[str] = []
    for key in row.keys():
        jp = jp_map.get(key, "")
        if jp:
            lines.append(f"# {jp}")
        val = parse_value(row[key])
        lines.append(f"{key} = {to_toml_literal(val)}")
        lines.append("")  # 見やすさのため空行
    return "\n".join(lines).rstrip() + "\n"

def  git_add_commit_push(
                        repo: git.Repo,
                        paths: List[Path],
                        remote_name: str,
                        branch: Optional[str],
                        message: Optional[str]
                        ):
    if not paths:
        print("[INFO] No files to commit; skip git operations.")
        return

    # 正しいリポジトリルートを取得
    if not getattr(repo, "working_tree_dir", None):
        raise RuntimeError("Repo has no working_tree_dir (bare repo?)")
    repo_root = Path(repo.working_tree_dir).resolve()

    # すべて絶対化
    abs_paths = [Path(p).resolve() for p in paths]

    # リポジトリ配下チェック（外なら明示エラー）
    for p in abs_paths:
        if repo_root not in p.parents and p != repo_root:
            raise ValueError(f"{p} is outside the repo root: {repo_root}")

    # 相対に揃える（relative_toが安全に通る）
    rels = [str(p.relative_to(repo_root)) for p in abs_paths]
    repo.index.add(rels)

    commit_msg = message or f"Add {len(paths)} config(s) at {dt.datetime.now():%Y-%m-%d %H:%M}"
    repo.index.commit(commit_msg)
    print(f"[OK] commit: {commit_msg}")

    origin = repo.remote(name=remote_name)
    if branch:
        res = origin.push(refspec=f"HEAD:{branch}")
    else:
        res = origin.push()
    for r in res:
        if r.flags & r.ERROR:
            print(f"[ERROR] push failed: {r.summary}", file=sys.stderr)
            sys.exit(3)

def parse_args():
    p = argparse.ArgumentParser(description="CSV → TOML → Git push")
    p.add_argument(
        "--csv",
        default=Path("common/simulation_params.csv"),
        help="Input CSV file path (default: common/simulation_params.csv)"
    )
    p.add_argument(
        "--out-dir",
        default=Path("test/configs"),
        help="Output directory for TOML files (default: test/configs)"
    )
    return p.parse_args()

def main():
    args = parse_args()
    # 受け取ったパスをPath型に変換（相対なら絶対化）
    CSV_PATH = Path(args.csv).resolve()
    OUT_DIR = Path(args.out_dir).resolve()
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] CSV_PATH = {CSV_PATH}")
    print(f"[INFO] OUT_DIR  = {OUT_DIR}")
    jp_header, en_header, data_rows = read_two_header_csv(CSV_PATH)
    jp_map = build_jp_map(jp_header, en_header)

    written = 0
    for row in data_rows:
        base = row.get("scenario_id") or f"case_{written+1:03d}"
        fname = sanitize_filename(str(base)) + ".toml"
        toml_text = row_to_toml_with_jp_comments(row, jp_map)
        (OUT_DIR / fname).write_text(toml_text, encoding="utf-8")
        written += 1
# OUT_DIR  = Path("test/configs")
    paths = []
    for i in range(written):
        path = OUT_DIR / f"{i+1}.toml"
        paths.append(path)
    print(f"[OK] {written} TOML files written to: {OUT_DIR}")
    repo = git.Repo("/Users/kashiisamutakeshi/vehicle-assistant-system")
    git_add_commit_push(repo=repo, paths=paths, remote_name="origin", branch="main", message="Add configs from CSV")

if __name__ == "__main__":
    # python3 common/config_generate.py \
    # --csv icmu2025/simulation_params.csv \
    # --out-dir icmu2025/configs
    main()


