from __future__ import annotations

from .db import Database
from .repositories import ScenarioRepository


def main() -> None:
    db = Database.from_env()

    # トランザクション単位で Repository を使う
    with db.transaction() as conn:
        repo = ScenarioRepository(conn)

        inserted = repo.generate_all_simulation_combinations(50)
        print(f"inserted rows: {inserted}")


if __name__ == "__main__":
    main()