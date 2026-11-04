# =========================
# CDF metrics
# =========================

import numpy as np


def calculate_cdf(arrival_time_dict):
    """
    arrival_time_dict: {比率: [到着時間リスト]}
    例: {0.1: [3, 5, 2, 8], 0.2: [4, 1, 6, 7]}

    戻り値: {比率: (到着時間リスト, CDF値リスト)}
    """
    cdf_dict = {}

    for ratio, arrival_times in arrival_time_dict.items():
        n = len(arrival_times)
        cdf_values = np.arange(1, n + 1) / n

        cdf_dict[ratio] = (arrival_times, cdf_values)

    return cdf_dict


def convert_to_cdf(data: dict, plot=True):
    """
    辞書形式のデータを CDF 形式に変換し、オプションでグラフを表示する。

    Parameters:
        data:
            {key: list of time values} の形式の辞書
        plot:
            True にすると CDF をグラフ表示

    Returns:
        dict:
            各 key に対して [(time, cdf_value), ...] のリストを持つ辞書
    """
    cdf_result = {}

    for key, times in data.items():
        sorted_times = sorted(times)
        n = len(sorted_times)

        if n == 0:
            cdf_result[key] = []
            continue

        cdf = [i / n for i in range(1, n + 1)]
        cdf_result[key] = list(zip(sorted_times, cdf))

        if plot:
            import matplotlib.pyplot as plt

            plt.plot(sorted_times, cdf, label=f"Key = {key}")

    if plot:
        import matplotlib.pyplot as plt

        plt.title("CDF for Each Key")
        plt.xlabel("Time")
        plt.ylabel("Cumulative Probability")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    return cdf_result
