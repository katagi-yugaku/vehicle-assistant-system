# =========================
# SUMO environment helper
# =========================

import os
import sys


def ensure_sumo_tools_on_path() -> None:
    """
    SUMO_HOME/tools を sys.path に追加する。

    元の evacsim/utilities.py と同じく、
    SUMO_HOME が設定されている場合のみ追加する。
    """
    if "SUMO_HOME" not in os.environ:
        return

    sumo_tools_path = os.path.join(os.environ["SUMO_HOME"], "tools")

    if sumo_tools_path not in sys.path:
        sys.path.append(sumo_tools_path)
