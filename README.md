# Vehicle Evacuation System

## Overview
本リポジトリは、**車両DTN（Delay/Disruption Tolerant Network）** を用いた避難支援シミュレーション環境です。  
災害時における車両避難を対象とし、運転者の行動モデル・車両間通信・地図別経路特性を統合的に評価できます。

---

## Directory Structure

vehicle-evacuation-system/
├── pyproject.toml
├── README.md
├── .env.example # 認証・環境変数（Git Token / PATH など）
│
├── bin/ # CLI 実行エントリ
│ ├── evac-run # -> python -m evacsim.sim.runner_cli
│ └── evac-aggregate # -> python -m evacsim.viz.aggregate_cli
│
├── tests/ # pytest 用テストコード
│
├── evacsim/ # コアライブラリ（地図非依存の再利用可能部）
│ ├── init.py
│ │
│ ├── core/ # 実行エンジン層
│ │ ├── init.py
│ │ ├── map_base.py # MapAdapter インタフェース
│ │ ├── engine.py # 時間発展ループ
│ │ └── hooks.py # 可視化・ログ用フック
│ │
│ ├── io/ # 入出力層
│ │ ├── init.py
│ │ ├── schema.py # pydantic スキーマ
│ │ ├── config_loader.py # 設定ファイル読込
│ │ ├── results_writer.py # JSON / CSV 出力
│ │ └── storage.py # run_id / 出力管理
│ │
│ ├── sim/ # シミュレーション制御
│ │ ├── init.py
│ │ ├── runner_core.py # 共通 run(params) -> Path
│ │ ├── runner_cli.py # CLI エントリ (bin/evac-run)
│ │ ├── parallel.py # 並列実行・ロック管理
│ │ ├── utilities.py # 共通ユーティリティ
│ │ └── find_route.py # 経路探索処理
│ │
│ └── viz/ # 可視化・集計層
│       ├── init.py
│       ├── cdf.py # CDF描画
│       ├── tables.py # 表生成 (LaTeX/Markdown)
│       └── aggregate_cli.py # 集計CLI (bin/evac-aggregate)
│
├── agents/
│       ├── init.py
│       ├── Agent.py
│       ├── Shelter.py
│       └── CustomeEdge.py
└── scenarios/ # 学会・論文ごとの実験設定
    ├── dicomo2025/
    │       simcore/                # 共通（新規）
    │          runner_core.py        # 共通の実行本体（ここが “runner” の実体）
    │          map_base.py           # MapAdapter のインタフェース
    │        maps/                   # 地図ごとの実装（差分）
    │          __init__.py
    │          ishinomaki.py
    │          xxx_city.py
    │        simulation/
    │          runner.py             # エントリ（従来の import 先を runner_core に差し替え）
    │          runner_nosystem.py    # 必要なら同様に
    │
    └── its102/ 
            maps/                   # 地図ごとの実装（差分）
              __init__.py
              ishinomaki.py
              xxx_city.py
            simulation/
              runner.py             # エントリ（従来の import 先を runner_core に差し替え）
              runner_nosystem.py    # 必要なら同様に
            map_one/data/...        # 既存データはまずそのまま
