# e2e テスト計画(E4 の前の通し確認)— 2026-11-18

目的:E4(全条件の再実行)を投入する前に、
「投入 → 実行 → 集計 → 保存 → メール」が最後まで通ること、E4 で使う修正一式が 2 経路・4 経路の両方で実際に効いていること、
同じシードで結果が再現することを、少ない回数で確かめる。

## 対象と前提
- コード:`origin/jip/revision-26S015f`(58a6478 + この e2e 一式)。**e2e のファイルはまだコミットしていない**ので、
  コミット → push → 計算機で `git fetch origin && git archive origin/jip/revision-26S015f scenarios/jip_rev | tar -x` してから行う。
- E4 で使う修正一式(暫定):`fix_origin_edge` + `fix_alternative_from_table` + `travel_time_offset = 0`
  + `vehicle_max_speed = 9.0` + `free_flow_speed = 9.0`(原稿 Table 1 の最高速度。11/19 追加)
  + 4 経路のみ `fix_pre_x_alternatives`、終了 3500 s。目的避難所の更新・信号は未対応のまま(決まったらここに追加)。

## 追加したもの
| ファイル | 内容 |
| --- | --- |
| `experiments/e2e/E2E_layouts.toml` | 2 経路・4 経路 × μθ=150・P=0.10 × off / fixed × シード 1–2 = 8 回(legacy 通信) |
| `experiments/e2e/E2E_comm.toml` | 2 経路 × μθ=60・σθ=50 × none / v2v_only / v2v_dtn × シード 1–2 = 6 回 |
| `tools/e2e_preflight.py` | 実験定義の静的チェック(設定の expect、未知のフラグ、runner の対応、offset、修正一式との違い) |
| `tools/e2e_check.py` | 実行後の判定(下の P/R/F/A/D 項目)。`results/<ID>/e2e_report.md` を書く |
| `tools/e2e_run.sh` | `preflight` / `submit [--local]` / `check` |

## 手順(計算機)
```bash
cd ~/vehicle-assistant-system/scenarios/jip_rev
tools/e2e_run.sh preflight        # 静的チェック + E2/E2_none/E3/e2e の --print-only(投入しない)
tools/e2e_run.sh submit           # 3 実験を投入(8 + 6 + 再現性用 4 = 18 回、32 並列なら 15 分程度)
# メール 3 通([evac-sim] ... SUCCESS)を確認してから
tools/e2e_run.sh check            # 判定。FAIL が 0 なら合格
```
Slurm を使わない場合は `submit --local`(既定 4 並列)。

## 合格基準(tools/e2e_check.py)
| 項目 | 内容 | 不合格なら |
| --- | --- | --- |
| P1 | manifest と tasks.tsv、予定数 = タスク数、dirty でない | 投入の仕組み |
| P2 | 全回 SUCCESS、run.err に Traceback なし | runner / 環境 |
| R1 | raw.json の schema とシード | 記録 |
| R2 | raw.json の flags = 実験定義の版のフラグ | 設定の受け渡し(一番重要) |
| R3 | run.out の [RUN_CONFIG] にシードとフラグ | 同上 |
| R4 | 全 400 台到着、完了時刻 ≤ 終了時刻 | 終了時刻・詰まり |
| R5 | 経路変更の内訳の合計が一致 | 集計の前提 |
| F1 | fix_origin_edge なら E0 再生成 0 回 | E0 修正 |
| F2 | off では E0 再生成が出る(不具合を再現できている)| WARN のみ |
| F3 | 通信なしで情報による経路変更 0 回 | WARN のみ |
| F4 | information.jsonl.gz / sections がある | ログ |
| A1 | summary.csv・decision.md・paired.csv・figs | 集計 |
| A2 | summary の成功数 = raw.json の数 | 集計 |
| D1 | E2E-layouts と E2E-layouts-rep のシード 1 が完全一致 | 再現性 |

あわせて目で確認すること:メールが 3 通届く、添付の図が開ける、`archive/<ID>.tar.zst` と `.sha256` がある、
E2E-layouts の decision.md で 4 経路の fixed − off が E3 の 1 回分の傾向(U ターン減・同調増)と矛盾しない。

## 確認済み(2026-11-18、Mac の VM、SUMO なし)
- `e2e_preflight.py`:既存の E2 / E2_none / E3 と e2e 2 本でエラー 0。わざと壊した定義(条件の値違い、フラグの綴り違い、
  4 経路で v2v_dtn)で 5 件のエラーを検出。
- `e2e_check.py`:Step 2 の実データ(raw.json.gz 3 回分)で組んだ模擬の実験で PASS。フラグ違い・失敗した回・集計数の不一致を FAIL として検出。
- 実際のシミュレーションと `run_experiment.py`(git が必要)はまだ通していない → 上の手順で計算機で行う。

## 気づいた点
- E3 の after は `fix_alternative_from_table` と `travel_time_offset = 0` を含まない(11/05 の定義のまま)。
  E3 を「E4 の判断材料」に使うなら、E4 と同じ修正一式で回し直すか、E4 の定義を先に決める。
- `fix_alternative_from_table` は表の値をそのまま使うので、`travel_time_offset = 100` だと結果が変わる(Step 2 の「B6 は影響なし」は旧い計算での話)。
