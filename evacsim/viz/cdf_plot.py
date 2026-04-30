# =========================
# CDF plot helpers
# =========================

import matplotlib.pyplot as plt


def plot_cdf(cdf_dict, fulltime_by_ratio: dict):
    """
    CDF をプロットする。

    既存 utilities.py の plot_cdf の挙動を維持する。
    """
    plt.figure(figsize=(8, 6))

    for ratio, (times, cdf_values) in cdf_dict.items():
        print(f'rate{str(round(ratio, 1)).replace(".", "")} = {times}')
        print(f'rate{str(round(ratio, 1)).replace(".", "")}fulltime = {fulltime_by_ratio[ratio]}')

        plt.plot(
            times,
            cdf_values,
            marker='.',
            linestyle='-',
            label=f"Ratio {ratio}",
        )

        # plt.vlines(fulltime_by_ratio[ratio], 0.0, 1.0, linestyles='dotted')

    plt.xlabel("Arrival Time")
    plt.ylabel("CDF")
    plt.title("Cumulative Distribution Function (CDF) of Arrival Times")

    # 既存 utilities.py の保存先を維持
    plt.savefig('/Users/kashiisamutakeshi/vehicle-assistant-system/sumo/its102/result_graph.pdf')

    plt.legend()
    plt.grid()
    plt.show()
