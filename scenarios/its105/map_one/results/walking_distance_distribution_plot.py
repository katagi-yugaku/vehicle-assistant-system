import json
import numpy as np
import matplotlib.pyplot as plt


def load_walking_distance_distribution(json_path: str) -> dict:
    """
    JSON から walking_distance_distribution を取り出す
    """
    with open(json_path, "r", encoding="utf-8") as f:
        obj = json.load(f)

    if "walking_distance_distribution" not in obj:
        raise KeyError("JSON に 'walking_distance_distribution' が存在しません。")

    return obj["walking_distance_distribution"]


def plot_walking_distance_distribution(dist_dict: dict, label_suffix: str):
    """
    walking_distance_distribution から回数分布を描画する関数

    Parameters
    ----------
    dist_dict : dict
        例:
        {
            "bin_width": 50,
            "bin_edges": [200, 250, 300, ... ,1000],
            "counts": [108, 112, 98, 8, ...]
        }

    label_suffix : str
        保存ファイル名に付与するサフィックス
    """
    bin_width = dist_dict["bin_width"]
    bin_edges = np.array(dist_dict["bin_edges"], dtype=float)
    counts = np.array(dist_dict["counts"], dtype=int)

    if len(bin_edges) != len(counts) + 1:
        raise ValueError(
            f"bin_edges の長さは counts より 1 だけ多い必要があります。"
            f" len(bin_edges)={len(bin_edges)}, len(counts)={len(counts)}"
        )

    # 各階級の左端
    left_edges = bin_edges[:-1]

    plt.figure(figsize=(10, 6))

    plt.bar(
        left_edges,
        counts,
        width=bin_width,
        align="edge",
        edgecolor="black",
        linewidth=1.0
    )

    plt.xticks(
        ticks=np.arange(bin_edges[0], bin_edges[-1] + 1, 100),
        fontsize=20,
        fontweight="semibold"
    )
    plt.yticks(
        fontsize=20,
        fontweight="semibold"
    )

    plt.xlabel("Walking distance [m]", fontsize=20, fontweight="semibold")
    plt.ylabel("Count", fontsize=20, fontweight="semibold")
    plt.grid(True, axis="y", alpha=0.3)

    save_path = f"/Users/kashiisamutakeshi/walking_distance_distribution_{label_suffix}.pdf"
    plt.savefig(save_path, bbox_inches="tight")
    print(f"✅ Saved figure as: {save_path}")

    plt.show()


if __name__ == "__main__":
    json_path = "/mnt/data/output.json"   # 適宜変更
    dist = load_walking_distance_distribution(json_path)
    plot_walking_distance_distribution(dist, label_suffix="30")