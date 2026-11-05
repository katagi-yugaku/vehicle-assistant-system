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
- パーティションは `ubuntu`(`--partition` / `EVAC_PARTITION` で変更可)。1 タスク 1 コア・4 GB・上限 2 時間。

## 6b. E2(通信方式の比較:V2V only / DTN / 通信なし、600 回)
E3 と同じ手順で、実験定義だけを変える。集計は `aggregate_e2.py`(E2.toml の `aggregator`)が自動で使われる。
```bash
# 投入前の確認(600 タスク。tasks.tsv の最後の列 v2v_rate が none の版だけ 0.0 になっていること)
tools/run_experiment.sh experiments/E2.toml --ref origin/jip/revision-26S015f --print-only
# 小さい本番確認(1 条件 × 3 版 × 2 シード = 6 回。1 回 6 分前後)
tools/run_experiment.sh experiments/E2.toml --ref origin/jip/revision-26S015f --seeds 1-2 --conditions mu150_p010 --name E2-smoke
# 本実行
tools/run_experiment.sh experiments/E2.toml --ref origin/jip/revision-26S015f --max-parallel 32
```
- E2 だけ(通信なしを除く 400 回)にするときは `--variants v2v_only,v2v_dtn`。
- メールには版ごとの完了時刻、同じシードの組の差(DTN − V2V only、DTN − 通信なし、V2V only − 通信なし)、
  「全条件で短いか」のまとめ、Fig.12 の下書き(`figs/arrival_cdf*.png`)が付く。E4 の判定は出ない。
- 終了時刻は 3500 s。到着がそろわない回があれば decision.md の先頭に警告が出る(その回の完了時刻は短く出る)。

試験用の絞り込み(E3 でも使える):`--seeds 1-2`、`--conditions mu150_p010,mu350_p030`、`--variants v2v_dtn`、`--layouts 1-1-2`。

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
