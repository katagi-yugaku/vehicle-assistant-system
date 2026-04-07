import json
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np


def normalize_scenario_arg(arg: str) -> str:
    arg = arg.strip()
    if arg.isdigit():
        return f"scenario{arg}"
    return arg


# def load_walking_distance_distribution(json_path: Path) -> dict:
#     if not json_path.exists():
#         raise FileNotFoundError(f"JSON file not found: {json_path}")

#     with json_path.open("r", encoding="utf-8") as f:
#         data = json.load(f)

#     if "walking_distance_distribution" not in data:
#         raise KeyError(f"'walking_distance_distribution' not found in {json_path}")

#     return data["walking_distance_distribution"]

def load_walking_distance_distribution(json_path: Path) -> dict:
    if not json_path.exists():
        raise FileNotFoundError(f"JSON file not found: {json_path}")

    with json_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    if "abandon_time_distribution" not in data:
        raise KeyError(f"'abandon_time_distribution' not found in {json_path}")

    return data["abandon_time_distribution"]



def plot_distribution(dist: dict, label_suffix: str) -> None:
    bin_width = dist["bin_width"]
    bin_edges = dist["bin_edges"]
    counts = dist["counts"]
    ratios = dist["ratios"]

    left_edges = np.array(bin_edges[:-1], dtype=float)

    plt.figure(figsize=(10, 6))
    plt.bar(left_edges, counts, width=bin_width, align="edge", edgecolor="black")

    plt.xlabel("Walking distance [m]")
    plt.ylabel("Count")
    plt.title(f"Walking distance distribution ({label_suffix})")
    plt.grid(True, axis="y", alpha=0.3)

    out_path = Path(__file__).resolve().parent / f"walking_distance_distribution_{label_suffix}.pdf"
    plt.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"saved: {out_path}")
    # plt.show()  # 必要なときだけ有効化


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 walking_distance_distribution_plot.py <scenario_id>")
        sys.exit(1)

    scenario_name = normalize_scenario_arg(sys.argv[1])

    base_dir = Path(__file__).resolve().parent
    json_path = base_dir / scenario_name / "output.json"

    dist = load_walking_distance_distribution(json_path)
    plot_distribution(dist, scenario_name)