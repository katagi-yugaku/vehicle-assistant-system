import toml
import glob

def generate_parameter_table(toml_data):
    # すべての LaTeX コマンドの { } を {{ }} にエスケープしています
    # Python から値を埋め込む場所だけ {変数名} となっています
    template = f"""\begin{table}[tb]
    \centering
    \caption{{シミュレーションパラメータの分類}}
    \label{{table:parameter}}
    \scalebox{{0.9}}{{
    \begin{tabular}{lr}
        \hline\hline
        \multicolumn{{2}}{{l}}{{\text{{シミュレーション設定}}}} \\
        \hline
        シミュレーション回数 & 異なる乱数シード値で{num_simulations}回 \\
        津波の予兆が視認できる時間 & \SIrange{{{tsunami_sign_start}}}{{{tsunami_sign_end}}}{{\mathrm{{sec.}}}} \\
        車両台数 & {num_vehicles}\,\text{{台}} \\
        車両の生成間隔 & {vehicle_interval:.1f}\,\mathrm{{sec.}} \\
        \hline\hline
        \multicolumn{{2}}{{l}}{{\text{{通信条件}}}} \\
        \hline
        車両間通信の通信可能範囲 & {comm_range:.1f}\,\mathrm{{m}} \\
        \hline\hline
        \multicolumn{{2}}{{l}}{{\text{{運転者モデル関連パラメータ}}}} \\
        \hline
        逆走決意の閾値 $\theta_{{i}}$ & {motivation_threshold_dist} \\
        逆走に対するモチベーション減少の 下限 $\theta_{{\mathrm{{min}},\,i}}$ & {min_motivation_dist} \\
        積極的逆走者における情報取得後の 上昇幅 $U_{{\,\mathrm{{norm}},\,i}}$ & {positive_lanechange_dist} \\
        その他における情報取得後の 上昇幅 $U_{{\,\mathrm{{norm}},\,i}}$ & {negative_lanechange_dist} \\
        同調性バイアスによるモチベーションの 増減量 $U_{{\,\mathrm{{majority}}}}$, $D_{{\,\mathrm{{majority}}}}$ & {positive_majority_bias:.0f} \\
        運転者の視認距離 & {visibility_distance:.1f}\,\mathrm{{m}} \\
        車群形成追従率 & {following_rate:.1f} \\
        \hline
        \end{tabular}
        }}
    \end{table}"""

    # 階層型データ取得用ヘルパー
    def get_val(keys, default=0):
        d = toml_data 
        for k in keys:
            if isinstance(d, dict):
                d = d.get(k, default)
            else:
                return default
        return d

    # 範囲を u[start:end] 形式にするヘルパー
    def to_dist(range_list):
        if isinstance(range_list, list) and len(range_list) == 2:
            return rf"$\mathrm{{u}}[{range_list[0]}:{range_list[1]}]$"
        return "N/A"

    # コンテキストの作成
    context = {
        "num_simulations": get_val(["scenario", "num_simulations"]),
        "tsunami_sign_start": get_val(["event", "tsunami_sign_start"]),
        "tsunami_sign_end": get_val(["event", "tsunami_sign_end"]),
        "num_vehicles": get_val(["demand", "num_vehicles"]),
        "vehicle_interval": get_val(["demand", "vehicle_interval"]),
        "comm_range": get_val(["communication", "comm_range"]),
        "motivation_threshold_dist": to_dist(get_val(["event", "motivation_threshold_range"])),
        "min_motivation_dist": to_dist(get_val(["event", "min_motivation_range"])),
        "positive_lanechange_dist": rf"$\mathrm{{u}}[{get_val(['driver', 'active', 'lanechange_start'])}:{get_val(['driver', 'active', 'lanechange_end'])}]$" ,
        "negative_lanechange_dist": rf"$\mathrm{{u}}[{get_val(['driver', 'cautious', 'lanechange_start'])}:{get_val(['driver', 'cautious', 'lanechange_end'])}]$" ,
        "positive_majority_bias": get_val(["driver", "common", "positive_majority_bias"]),
        "visibility_distance": get_val(["driver", "common", "visibility_distance"]),
        "following_rate": get_val(["driver", "common", "following_rate"]),
    }

    return template.format(**context)

if __name__ == "__main__":
    # 自動で scenario_*.toml を探して処理する
    files = glob.glob("scenario_1.toml")
    if not files:
        print("TOMLファイル(scenario_*.toml)が見つかりません。")
    else:
        target = files[0]
        data = toml.load(target)
        latex_table = generate_parameter_table(data)
        
        with open("table_parameter.tex", "w", encoding="utf-8") as f:
            f.write(latex_table)
        print(f"Successfully generated table_parameter.tex from {target}")


