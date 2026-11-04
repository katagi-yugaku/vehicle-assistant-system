# JIP 26-S015f 改訂:Step 0〜2 の記録(2026-11-04)

> 2026-11-05 追記:改訂用のものはすべて `scenarios/jip_rev/` に移した(修正版コードは `scenarios/jip_rev/code/`)。
> 既存の `evacsim/` と `scenarios/JIP/` は origin/main と同じ内容に戻してある。以下の記録中のパスは移動前のもの。

## Step 0:準備
| 項目 | 内容 |
| --- | --- |
| ブランチ | `jip/revision-26S015f`(origin/main `e981d5e` から作成。main と既存ブランチは変更なし) |
| worktree | `~/vehicle-assistant-system-rev` |
| 論文の元データ | `results_paper_v1/scenarios/JIP/{1-1-2,1-1-4}/results/`(`jip/26-07-12-2200-1-1-4-schubert` から `git archive` で取り出し。338 ファイル) |
| タグ(ローカルのみ) | `paper-v1-fig8-9` = e5468f8、`paper-v1-fig10-11` = 7dcbe1b、`paper-v1-fig12-v2vonly-code` = eda485c(軽量タグ) |
| 動作確認の環境 | Linux x86_64、Python 3.10.20、eclipse-sumo 1.20.0、traci 1.20.0、sumolib 1.20.0、numpy 2.2.6、tomli 2.5.0、`PYTHONHASHSEED=0` |

注:Mac 上の VM のシェルは 1 回 3 分で打ち切られるため、Step 2 の実行は Claude のクラウド作業環境(上の環境)で行った。計算機(schubert、Slurm)には接続していない。
Mac の VM 用には `~/.venv-rev`(VM 内、Python 3.10.12、同じ版の SUMO/traci/numpy/tomli)も作った。

## Step 1:フラグ(TOML の設定キー。未指定なら元の動作)
| キー | 既定 | 内容 | 変更した場所 |
| --- | --- | --- | --- |
| `fix_origin_edge` | false | B1。E0 を E1 と同じ「分岐前の区間」として扱う(共有平均による前方の `setRoute` だけを判断し、U ターン・同調・満杯判断に入れない)。逆向きの edge がない場合は U ターン候補を作らない | 両 runner の `PRE_X_DECISION_EDGES` / `PRE_X_SKIP_EDGES`、`routing.UTURN_REQUIRE_OPPOSITE_EDGE`(`_evaluate_u_turn_candidate`) |
| `fix_pre_x_alternatives` | false | B3(4 経路)。代替経路を、現在の経路と分岐し平均値を持つ経路のうち最小のものにする | 1-1-4 runner、`revision_options.select_pre_x_alternative` |
| `seed` / `--seed` | なし | B4。random と numpy に設定(1-1-4 は従来どおり SUMO の `--seed` にも渡す)。`[RUN_CONFIG]` 行に出力 | 両 runner |
| `save_raw` / `--raw-output` | false / `raw.json` | B5。シード、完了時刻、車両ごとの出発・到着時刻と到着時の経路、全経路変更イベント、E0 上の再生成回数を JSON で保存 | `revision_options.RawRecorder`、`arrival.ARRIVAL_RECORDER` |
| `log_sections` | false | 30 s ごとの edge 別の台数と平均速度(`section_interval` で変更可) | 両 runner の `run()` |
| `travel_time_offset` | 100 | B6 | `arrival.TRAVEL_TIME_OFFSET` |
| `communication_mode` / `--communication-mode` | `legacy` | B2。`legacy` = 元の車両ごとの `v2v_communication`。`v2v_dtn` / `v2v_only` = eda485c のラウンド単位の実装(1-1-2 のみ) | 1-1-2 runner、`evacsim/sim/communication_round.py`、`evacsim/agents/VehicleInfo_round.py` |
| `end_simulation_time` | 3000 | 終了時刻(両方式でそろえるため設定可能にした) | 両 runner |

B2 の取り込み方:eda485c の `communication.py` と `VehicleInfo.py` を `communication_round.py` / `VehicleInfo_round.py` として追加し(import 先の 1 行だけ変更)、`v2v_dtn` / `v2v_only` のときだけ `revision_options.bootstrap()` が元のモジュール名に差し替える。legacy のファイルは変更していない。runner には eda485c の `process_communication_round` と `_build_candidate_neighbors` を移した。「2 経路分の情報を持つまで判断しない」条件は残したまま(両方式に適用)。

## Step 2:動作確認(条件 config_scenario_1:μθ=150、P=0.10、シード 1)
比較は `tools/compat/run_seeded.py`(元のコードのシードを外から固定し、到着時刻と全経路変更イベントを書き出す)と `tools/compat/compare_traces.py` で行った。記録は `revision/step2_checks/`。

| 確認 | 結果 |
| --- | --- |
| 元のコードを同じシードで 2 回 | 一致(シード固定で再現できる) |
| 2-1 フラグ off、2 経路:元 vs 修正版 | **一致**(到着 400 台、イベント 164 件、カウンタ) |
| 2-1 フラグ off、4 経路:元 vs 修正版 | **一致**(到着 400 台、イベント 297 件、カウンタ) |
| 記録だけ on(save_raw + log_sections)、2 経路:元 vs 修正版 | 一致(記録が動作を変えない) |
| 2-2 `fix_origin_edge` のみ、2 経路 | E0 上の再生成 33 → **0**。E0 上の `setRoute` 0 → **56** |
| 2-3 `travel_time_offset` 100 vs 0、2 経路 | **一致**(到着時刻とイベントがすべて同じ。B6 は結果に影響しない) |

参考(1 回だけの実行。傾向の判断には使えない):
| 実行 | 完了時刻 | 平均到着 | 経路変更 | U ターン | E0 U ターン | E0 setRoute | 同調 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 2 経路 元 | 2423 | 1372.8 | 160 | 86 | 33 | 0 | 28 |
| 2 経路 fix_origin_edge | 2406 | 1379.3 | 158 | 90 | 0 | 56 | 40 |
| 4 経路 元 | 2597 | 1392.6 | 297 | 134 | 116 | 1 | 69 |
| 4 経路 fix_origin_edge | 2322 | 1336.5 | 291 | 24 | 0 | 7 | 132 |
| 4 経路 fix_origin_edge + fix_pre_x | 2342 | 1342.4 | 302 | 23 | 0 | 9 | 130 |

## 想定と違ったこと・新たに見つかったこと
1. **旧 V2V only(Fig.12)の出発間隔が DTN と違う。** eda485c の runner は `vehicle_interval_start/end` が設定にないと `vehicle_interval`(= 5 s)で一定にする。DTN は 4.0→3.0 s の線形。scenario 70–73 の設定には start/end がないため、旧 V2V only は 400 台を約 2,000 s かけて出発させていた(DTN は約 1,400 s)。また SUMO にもシードを渡していた。修正版は両方式とも元の 4.0→3.0 s。
2. eda485c の runner にある「経路変更時に VehicleInfo の行先も同期する処理」と「同調時の避難所 ID の修正」は取り込んでいない(legacy と同じ動作のまま)。必要なら追加のフラグにする。
3. `fix_origin_edge` を on にすると、分岐前の判断の大半が E1 ではなく E0 で起きる(2 経路で E1 の変更 74 → 12、E0 の setRoute 56)。E0 上で車両が長く滞留するため。
4. 4 経路では、`fix_origin_edge` だけで U ターンが 134 → 24 回に減り、同調による変更が 69 → 132 回に増え、完了時刻が 275 s 早くなった(1 回のみ)。E3 の 50 回で確かめる必要があるが、E4(全条件の再実行)が必要になる可能性が高い。
5. 4 経路の `fix_pre_x_alternatives` は、E0 での変更先が E0_E16_0 / E0_E13_1(実際に平均値がある経路)になることを確認した。E1 での変更は 0 回(判断が E0 で済むため)。
6. 1-1-2 の `fix_pre_x_alternatives` は未実装(2 経路では代替経路が 1 本で、元の固定 ID と同じになるため)。on にすると注意を出力する。

## 使い方の例
```
PYTHONHASHSEED=0 python3 -m scenarios.JIP.1-1-2.simulation.runner_simulator --nogui CONFIG.toml 1.0 1.0 --seed 7 --raw-output out/raw.json
```
`--seed` / `--raw-output` / `--communication-mode` は位置引数の後ろに置く(元の引数の位置を変えないため、runner の先頭で取り除く)。
