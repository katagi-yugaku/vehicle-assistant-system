import json
import numpy as np
import matplotlib.pyplot as plt


def load_cdf_source(json_path: str) -> dict[str, list[float]]:
    """
    JSON から cdf_source を取り出す
    """
    with open(json_path, "r", encoding="utf-8") as f:
        obj = json.load(f)

    if "cdf_source" not in obj:
        raise KeyError("JSON に 'cdf_source' が存在しません。")

    return obj["cdf_source"]


def plot_cdfs(data_dict: dict[str, list[float]], label_suffix: str):
    """
    各キーに対応する到着時間リストのCDFを描画する関数

    Parameters
    ----------
    data_dict : dict[str, list[float]]
        例:
        {
            "0.1": [...],
            "0.5": [...],
            "0.9": [...],
            "nosystem": [...]
        }

    label_suffix : str
        保存ファイル名に付与するサフィックス
    """
    plt.figure(figsize=(10, 6))

    # スタイル定義
    style_map = {
        "0.1": {"color": "r",    "linestyle": "-",  "label": "early_rate=0.1"},
        "0.5": {"color": "blue", "linestyle": "-",  "label": "early_rate=0.5"},
        "0.9": {"color": "g",    "linestyle": "-",  "label": "early_rate=0.9"},
        "nosystem": {"color": "black", "linestyle": "--", "label": "no system"},
    }

    # 描画順を固定
    plot_order = ["0.1", "0.5", "0.9", "nosystem"]

    for key in plot_order:
        if key not in data_dict:
            continue

        value = np.array(data_dict[key], dtype=float)
        if len(value) == 0:
            continue

        value = np.sort(value)
        cdf = np.arange(1, len(value) + 1) / len(value)

        style = style_map.get(
            key,
            {"color": "gray", "linestyle": "-", "label": key}
        )

        plt.plot(
            value,
            cdf,
            label=style["label"],
            color=style["color"],
            linestyle=style["linestyle"],
            linewidth=2,
        )

    plt.xlim(600, 1250)
    plt.ylim(0.5, 1.0)
    plt.xticks(
        ticks=np.arange(600, 1251, 100),
        fontsize=20,
        fontweight="semibold"
    )
    plt.yticks(
        ticks=np.arange(0.5, 1.01, 0.1),
        fontsize=20,
        fontweight="semibold"
    )

    plt.xlabel("Arrival time [s]", fontsize=20, fontweight="semibold")
    plt.ylabel("CDF", fontsize=20, fontweight="semibold")
    # plt.legend(fontsize=14)
    # plt.grid(True, alpha=0.3)

    save_path = f'/Users/kashiisamutakeshi/lane_ingight{label_suffix}_normalcy400700_result.pdf'
    plt.savefig(save_path, bbox_inches="tight")
    print(f"✅ Saved figure as: {save_path}")

    plt.show()


if __name__ == "__main__":
    json_path = "/mnt/data/output.json"   # 適宜変更
    cdf_source = load_cdf_source(json_path)
    plot_cdfs(cdf_source, label_suffix="30")