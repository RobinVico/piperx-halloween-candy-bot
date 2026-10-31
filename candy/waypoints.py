"""Taught PiPER X poses for the candy bot, stored in candy/waypoints.json.

Each waypoint: {"q": [J1..J6 deg], "grip": opening_mm}. Record them with the "记录点位" button in the
teleop panel (teleop/teleop_real.py) while driving the arm with the SO-101.
"""

import json
import time
from pathlib import Path

import numpy as np

FILE = Path(__file__).resolve().parent / "waypoints.json"

# Name -> what it is (shown in the teach dropdown). grasp_* can be 1..N.
PRESETS = {
    "home": "休息位（折叠，夹爪朝上）",
    "box_above": "糖盒正上方 10-15 cm，夹爪朝下、张开",
    "grasp_1": "抓糖点 1：夹爪伸进糖里（张开）",
    "grasp_2": "抓糖点 2",
    "grasp_3": "抓糖点 3",
    "handoff_above": "递糖区上方 25-30 cm（松爪处）",
    "transit": "（可选）糖盒和递糖区之间的过渡点",
}
REQUIRED = ["home", "box_above", "grasp_1", "handoff_above"]


def load(path: Path = FILE) -> dict:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def save_point(name: str, q_deg, grip_mm: float, path: Path = FILE) -> None:
    wps = load(path)
    wps[name] = {"q": [round(float(v), 2) for v in q_deg], "grip": round(float(grip_mm), 1),
                 "t": time.strftime("%Y-%m-%d %H:%M:%S")}
    path.write_text(json.dumps(wps, indent=2, ensure_ascii=False), encoding="utf-8")


def missing(wps: dict) -> list[str]:
    return [n for n in REQUIRED if n not in wps]


def q(wps: dict, name: str) -> np.ndarray:
    return np.array(wps[name]["q"], dtype=float)


def grasp_names(wps: dict) -> list[str]:
    return sorted(n for n in wps if n.startswith("grasp_"))
