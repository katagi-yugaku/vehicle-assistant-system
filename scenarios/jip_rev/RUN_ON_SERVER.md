# 計算機(hendel / Slurm)での実行手順:JIP 26-S015f 改訂

改訂用のものはすべて `scenarios/jip_rev/` にある。既存の `evacsim/` と `scenarios/JIP/` は使わない・変えない。
実行するコードは、投入時に git のコミット(`origin/jip/revision-26S015f`)から `runs/<実験ID>/frozen/` に凍結する。
投入後にファイルやブランチを変えても、実行中のジョブには影響しない。

## 1. 取り込み(初回と、改訂用ブランチを更新したとき)
```bash
cd ~/vehicle-assistant-system
git fetch origin
# ブランチは切り替えず、jip_rev だけを作業ツリーに取り出す(index は変更しない)
git archive origin/jip/revision-26S015f scenarios/jip_rev | tar -x
cd scenarios/jip_rev
```
`git status` には `?? scenarios/jip_rev/` と出るが、問題ない(実行に使うのはコミットの中身)。

## 2. 環境の確認(初回)
```bash
python3 -c "import traci, sumolib, numpy, tomli, matplotlib; print('ok')"   # matplotlib がなければ図だけ作られない
sumo --version | head -1        # 1.20 系を推奨
```
足りないものがあれば `export EVAC_PYTHON=/path/to/venv/bin/python` で使う Python を指定する。

## 3. メールと NAS の設定(初回)
```bash
ls -l ~/.config/evac-mail/gmail.ini       # 権限 600。[smtp] に user / app_password / to
mkdir -p ~/.config/evac-sim
echo 'export EVAC_ARCHIVE_DEST=<NAS の保存先>' > ~/.config/evac-sim/archive.env
```
- `EVAC_ARCHIVE_DEST` は、計算ノードからマウントされているパス(例:`/mnt/debussy/...`)か、`host:/path`(rsync over ssh、鍵認証が必要)。
- 未設定なら NAS へのコピーはせず、`archive/` に圧縮ファイルを置くだけ(メールにその旨が出る)。

## 4. 投入前の確認
```bash
# 投入せずに sbatch のコマンドと 800 回分のタスク一覧を作る
tools/run_experiment.sh experiments/E3.toml --ref origin/jip/revision-26S015f --print-only
# 不要になった試しの runs/<実験ID> は削除してよい
```

## 5. 小さい本番確認(32 回、数分〜十数分)
```bash
tools/run_experiment.sh experiments/E3.toml --ref origin/jip/revision-26S015f --seeds 1-2 --name E3-smoke
```
メール `[evac-sim] <実験ID> SUCCESS` が届き、`results/<実験ID>/decision.md` に数値が入っていれば OK。

## 6. 本実行(E3:800 回)
```bash
tools/run_experiment.sh experiments/E3.toml --ref origin/jip/revision-26S015f --max-parallel 32
```
- ジョブ配列 1 本(`--array=1-800%32`)と、終了後の集計ジョブ 1 本(`--dependency=afterany`)が入る。
- パーティションは `ubuntu`(`--partition` / `EVAC_PARTITION` で変更可。basic は bach が不調のため当面使わない)。1 タスク 1 コア・4 GB・上限 2 時間。

## 6b. E2(通信方式の比較、μθ の掃引、修正版。2026-11-06 版)
E3 と同じ手順で、実験定義だけを変える。集計は `aggregate_e2.py` が自動で使われる。
- `experiments/E2.toml`:μθ = 60–300 s(30 s 刻み、σθ = 50 s)+ μθ = 150 s・σθ = 100 s、P = 0.10、
  V2V only / DTN、シード 1–50 → 1,000 回(1 回 約 12 分、32 並列で約 6.5 時間)
- `experiments/E2_none.toml`:通信なし、1 条件 × 50 回(1 回 約 3 分)
- どちらも fix_origin_edge + fix_alternative_from_table(原稿の式 (1))、travel_time_offset = 0、伝達の記録あり
```bash
# 投入前の確認(タスク数が 1000 / 50 になること)
tools/run_experiment.sh experiments/E2.toml --ref origin/jip/revision-26S015f --print-only
tools/run_experiment.sh experiments/E2_none.toml --ref origin/jip/revision-26S015f --print-only
rm -rf runs/*_E2_* runs/*_E2-none_*
# 小さい本番確認(4 回)
tools/run_experiment.sh experiments/E2.toml --ref origin/jip/revision-26S015f --seeds 1-1 --conditions mu060_sd050,mu300_sd050 --name E2-smoke2
# 本実行
tools/run_experiment.sh experiments/E2.toml --ref origin/jip/revision-26S015f --max-parallel 32
tools/run_experiment.sh experiments/E2_none.toml --ref origin/jip/revision-26S015f --max-parallel 32
# 終わったら伝達の分析(条件 × 方式ごとの表と図)
python3 tools/analyze_information.py runs/<E2 の実験ID> --out results/<E2 の実験ID>/information
# 避難所が共有した経路ごとの平均旅行時間の推移(経路, 平均所要時間, 時刻)。--window 200 で直近 200 s の平均も並べる
python3 tools/route_table_timeseries.py runs/<E2 の実験ID> --window 200
```

試験用の絞り込み(E3 でも使える):`--seeds 1-2`、`--conditions mu150_sd050,mu150_sd100`、`--variants v2v_dtn`、`--layouts 1-1-2`。
同じ分に同じ名前の実験を作ると ID が重なって止まるので、`--name` で名前を変える。

## 6c. 小実験 E2s(9 m/s 版の V2V only / DTN、2026-11-19)
Mac でコミット・push したあと、計算機では次の 1 行で「取り込み → 確認 → 投入」まで行う。
```bash
cd ~/vehicle-assistant-system/scenarios/jip_rev
tools/server_run.sh experiments/E2s_9ms.toml --print-only   # 初回だけ:タスク数 60 を確認(作られた runs/ は消してよい)
tools/server_run.sh experiments/E2s_9ms.toml                # 投入(60 回、32 並列で 30 分前後)
```
- 条件:μθ = 60(σθ 50)/ 150(σθ 100、論文の条件)/ 300(σθ 50)× V2V only / DTN × シード 1–10。
  最高速度・自由流速度 9 m/s、終了 4,000 s、伝達ログと判断ログあり。
- 終わると集計ジョブがメールを送る。本文に「完了時刻の差」「情報伝達の比較」「判断の記録(短縮時間と θ)」、
  添付に図・summary.csv・paired.csv・decisions_summary.csv・decisions_timeline.csv。
- 解析をやり直すとき:
  `python3 tools/analyze_decisions.py runs/<実験ID> --out results/<実験ID>/decisions`、
  `python3 tools/analyze_information.py runs/<実験ID> --out results/<実験ID>/information`。

### メールの宛先
宛先は計算機の `~/.config/evac-mail/gmail.ini` の `to`。アドレスはリポジトリに入れない。
```bash
sed -i 's/^to *=.*/to = <宛先のアドレス>/' ~/.config/evac-mail/gmail.ini && grep '^to' ~/.config/evac-mail/gmail.ini
```
一時的に変えるだけなら `export EVAC_MAIL_TO=<宛先>` してから投入する(集計ジョブにも引き継がれる)。

## 6d. 時間窓の感度 E2w(2026-11-19)
経路別平均を「直近 W 秒の到着(窓内が 3 台未満なら直近 3 台まで広げる)」で計算する版(フラグ `avg_window_s`・`avg_window_min_samples`)。
```bash
cd ~/vehicle-assistant-system/scenarios/jip_rev
tools/server_run.sh experiments/E2w_window.toml --print-only                       # タスク 160 を確認 → runs/ の確認用ディレクトリは消す
tools/server_run.sh experiments/E2w_window.toml --partition=ubuntu --max-parallel 72  # 投入
```
- 版:`{only,dtn}_{cum,w200,w300,w600}`(V2V only / DTN × 累積・W=200/300/600 s)、条件 μθ = 60(σθ 50)/ 150(σθ 100)、シード 1–10。9 m/s、終了 4,000 s。
- メールの「差」の表は、各 W での V2V only → DTN と、DTN での累積 → 各 W の 7 組。判断の記録に、窓を入れたときの分岐前の短縮と θ が出る。
- `--partition=basic` を使う場合は、先に bach・wagner で `srun -p basic -w <ノード> python3 -c "import traci,sumolib,tomli,numpy"` と `sumo --version` が通ることを確認する。
- 投入中の実験(E2s など)は投入時のコードで凍結されているので、push しても影響しない。

## 6e. 本実験 E4h(μθ × P のヒートマップ、V2V only / DTN、2026-11-19)
E2w のメールで W を 1 つ決めてから投入する(実験定義の既定は 300 s。`--set` で上書き)。
```bash
cd ~/vehicle-assistant-system/scenarios/jip_rev
df -h ~                                                     # 生データ約 5 GB(1 回 約 1.9 MB × 2,500)が入るか確認
tools/server_run.sh experiments/E4h_heatmap.toml --set avg_window_s=<W> --print-only   # タスク 2,500 を確認
rm -rf runs/*_E4h-heatmap_*
tools/server_run.sh experiments/E4h_heatmap.toml --set avg_window_s=<W> --partition=ubuntu --max-parallel 72
```
- 条件:論文 Fig.8/9 と同じ 5×5(μθ = 150〜350 s、σθ = 100 s × P = 0.10〜0.30)× V2V only / DTN × シード 1–50 = 2,500 回。
  9 m/s、時間窓(最低 3 台)、終了 4,000 s、伝達・判断ログあり。ubuntu の 56 並列で 9〜11 時間。
- 集計ジョブが `plot_heatmaps.py` でヒートマップ(完了時刻・平均到着・経路変更。V2V only、DTN、差 DTN − V2V only、
  差は 95% CI が 0 をまたがないセルに *)を作り、メール本文に表、添付に図を入れる。
- bach は srun のステップが起動しない状態だったので(11/19)、当面は `--partition=ubuntu` を付ける。

## 7. 進み具合・取り消し
```bash
tools/experiment_status.sh runs/<実験ID>
squeue -u $USER
scancel <配列ジョブ ID> <集計ジョブ ID>       # 投入時に表示される
```

## 8. 終わったら
メール(件名 `SUCCESS` / `INCOMPLETE` / `FAILED`)に、完了時刻の平均と 95% 信頼区間、
修正前後の差(同じシードの組)、E4 の判断材料、図、summary.csv が付く。

| 置き場所 | 中身 |
| --- | --- |
| `results/<実験ID>/` | summary.csv、paired.csv、runs.csv、decision.md / .json、figs/、manifest.json、finalize.log(git で管理する) |
| `runs/<実験ID>/` | 1 回ごとの run.out、run.err、raw.json、tripinfo.xml、status.txt、slurm/、frozen/(作業用。git に入れない) |
| `archive/<実験ID>.tar.zst` と NAS | runs/<実験ID> を丸ごと圧縮したもの + .sha256(生データの正本) |

結果を git に入れる(改訂用ブランチとは別の結果用ブランチ、または Mac 側で):
```bash
git -C ~/vehicle-assistant-system/scenarios/jip_rev status --short results/
```

## 9. 失敗したとき
- `INCOMPLETE`:失敗した回の `run.err` の末尾がメールに出る。原因を直したら、同じ実験をやり直す
  (新しい実験 ID になる)。成功分だけの集計は `results/<実験ID>/` に残る。
- 集計だけやり直す:`runs/<実験ID>/frozen/tools/finalize_experiment.sh runs/<実験ID> --dry-run-mail`
- 生データは自動では消さない。NAS の照合(`sha256 OK`)と結果の確認が済んだら `runs/<実験ID>` を手で消してよい。
