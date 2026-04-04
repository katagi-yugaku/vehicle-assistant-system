# runner_param_sweep_no_pandas.py

import itertools
import random
import csv
import matplotlib.pyplot as plt

# =====================
# 設定
# =====================
SEED = 42
N_AGENTS = 300
TARGET_ABANDON_RATE = 0.20
TIME_GRID = list(range(0, 181, 5))
TSUNAMI_OBTAIN_TIMES = [0, 10, 20, 30]

random.seed(SEED)

# =====================
# パラメタ
# =====================
ALPHA_LIST = [0.2, 0.5, 1.0]
BETA_LIST  = [0.0, 0.5, 1.0]
GAMMA_LIST = [0.5, 1.0, 1.5]
DELTA_LIST = [1.0, 2.0, 3.0]

NORMALCY_RANGES = [(2000,4000),(4000,6000),(6000,8000)]
MAJORITY_RANGES = [(1000,3000),(3000,5000),(5000,7000)]
THRESHOLD_RANGES = [(2000,4000),(4000,6000),(6000,8000)]

# =====================
# Agent生成
# =====================
def sample_agents(n, n_range, m_range, t_range):
    agents = []
    for _ in range(n):
        agents.append({
            "normalcy": random.uniform(*n_range),
            "majority": random.uniform(*m_range),
            "threshold": random.uniform(*t_range),
        })
    return agents

# =====================
# 判定式
# =====================
def abandon_formula(t, jam_start, tsunami_t, agent, a,b,g,d):
    jam = max(0, t - jam_start)
    tsunami = max(0, t - tsunami_t)

    value = (
        a*(jam**2)
        + b*tsunami
        - g*agent["normalcy"]
        + d*agent["majority"]
    )
    return value > agent["threshold"]

# =====================
# 評価
# =====================
def evaluate_case(case_id, params):
    random.seed(SEED + case_id)

    agents = sample_agents(
        N_AGENTS,
        params["normalcy"],
        params["majority"],
        params["threshold"]
    )

    jam_start = 0

    def calc_rate(mult):
        rates = []
        for t in TIME_GRID:
            count = 0
            for ag in agents:
                flags = []
                for ts in TSUNAMI_OBTAIN_TIMES:
                    flags.append(
                        abandon_formula(
                            t, jam_start, ts, ag,
                            params["alpha"],
                            params["beta"],
                            params["gamma"],
                            params["delta"]
                        )
                    )
                if sum(flags)/len(flags) >= 0.5:
                    count += 1
            rates.append(count/len(agents))
        return rates

    no_social = calc_rate(1.0)
    social = calc_rate(1.5)

    final_rate = social[-1]
    uplift = social[-1] - no_social[-1]

    score = abs(final_rate - TARGET_ABANDON_RATE) + max(0, 0.05-uplift)

    return {
        "case_id": case_id,
        "params": params,
        "final_rate": final_rate,
        "uplift": uplift,
        "score": score,
        "curve": social
    }

# =====================
# メイン
# =====================
def main():
    results = []
    case_id = 0

    for combo in itertools.product(
        ALPHA_LIST,BETA_LIST,GAMMA_LIST,DELTA_LIST,
        NORMALCY_RANGES,MAJORITY_RANGES,THRESHOLD_RANGES
    ):
        a,b,g,d,n,m,t = combo

        params = {
            "alpha":a,
            "beta":b,
            "gamma":g,
            "delta":d,
            "normalcy":n,
            "majority":m,
            "threshold":t
        }

        res = evaluate_case(case_id, params)
        results.append(res)

        if case_id % 50 == 0:
            print(f"{case_id} done")

        case_id += 1

    # =====================
    # CSV保存
    # =====================
    with open("sweep_results.csv","w",newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["case_id","alpha","beta","gamma","delta",
                         "normalcy","majority","threshold",
                         "final_rate","uplift","score"])

        for r in results:
            p = r["params"]
            writer.writerow([
                r["case_id"],
                p["alpha"],p["beta"],p["gamma"],p["delta"],
                p["normalcy"],p["majority"],p["threshold"],
                r["final_rate"],r["uplift"],r["score"]
            ])

    # =====================
    # 上位表示
    # =====================
    results.sort(key=lambda x: x["score"])

    print("\n=== TOP 10 ===")
    for r in results[:10]:
        print(r)

    # =====================
    # 可視化
    # =====================
    best = results[0]

    plt.plot(TIME_GRID, best["curve"])
    plt.axhline(0.20, linestyle="--")
    plt.title("Best case")
    plt.savefig("best_curve.png")

    print("\nDone.")

if __name__ == "__main__":
    main()