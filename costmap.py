#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Costmap visualizer for CalcPassDir (C++ logic port).
- Step 1) select best teammate by score = -dist(ball, tm) - tm.x (within min/max thresholds)
- Step 2) search pass target (tx,ty) in grid around that teammate
- Step 3) score = base - |dx|*w - |dy|*w - x*w_x  - sum(opponent penalties near pass segment)
- Visualize the score map (heatmap), positions, and chosen best target.

Requires: numpy, matplotlib
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple
import numpy as np
import matplotlib.pyplot as plt


# =========================
# 0) Hyperparameters (TUNE HERE)
# =========================
PARAMS = {
    # teammate selection
    "min_pass_threshold": 1.0,
    "max_pass_threshold": 4.0,
    "tm_select_w_dist": 1.0,   # score = -(w_dist*dist) - (w_x*tm.x)
    "tm_select_w_x": 1.2,

    # field limits (same meaning as fd.length/2, fd.width/2)
    "field_half_length": 4.5,   # example: 9m field length -> half=4.5
    "field_half_width": 3.0,    # example: 6m field width  -> half=3.0

    # grid around chosen teammate
    "grid_half_xrange": 3.0,     # tmPos +/- n meter
    "grid_half_yrange": 2.0,
    "grid_step": 0.2,

    # base score terms (exactly matching your C++ expression)
    "base_score": 10.0,
    "w_abs_dx": 1.1,            # -(abs(x-tm.x) * w_abs_dx)
    "w_abs_dy": 0.7,            # -(abs(y-tm.y) * w_abs_dy)
    "w_x": 0.95,                 # -(x * w_x)  (x lower -> higher score, if field direction matches)
    "w_y": 0.45,                 # prefer y = 0

    # opponent penalty near pass path
    "opp_path_margin": 1.15,     # if distToPassPath < margin
    "opp_penalty": 24.0,        # score -= (1 - dist/margin) * opp_penalty * confidence
    "opp_memory_sec": 5.0,      # confidenceFactor = max(0, (memory - elapsed)/memory)

    # decision threshold
    "score_threshold": 6.0,

    # visualization
    "figsize": (12, 5),
    "show_all_teammates": True,
}


# =========================
# 1) Data model (example inputs)
# =========================
@dataclass
class Pose2D:
    x: float
    y: float

@dataclass
class Teammate:
    player_id: int
    pos: Pose2D
    is_alive: bool = True
    label: str = "Teammate"

@dataclass
class Opponent:
    pos: Pose2D
    last_seen_sec_ago: float  # elapsed seconds since last observation
    label: str = "Opponent"


# =========================
# 2) Geometry: point to segment distance
#    (projection + clamp), matches typical implementation.
# =========================
def point_to_segment_distance(px: float, py: float,
                              ax: float, ay: float,
                              bx: float, by: float) -> float:
    """Minimum distance from point P to segment AB."""
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-12:
        # A == B
        return np.hypot(apx, apy)

    t = (apx * abx + apy * aby) / ab2
    t = max(0.0, min(1.0, t))  # clamp to segment
    cx, cy = ax + t * abx, ay + t * aby
    return np.hypot(px - cx, py - cy)


# =========================
# 3) Ported logic
# =========================
def select_best_teammate(ball: Pose2D,
                         teammates: List[Teammate],
                         my_player_id: int,
                         params: dict) -> Optional[Teammate]:
    best = None
    best_score = -1e18

    for tm in teammates:
        if tm.player_id == my_player_id:
            continue
        if not tm.is_alive:
            continue

        dist = np.hypot(ball.x - tm.pos.x, ball.y - tm.pos.y)
        if dist <= params["min_pass_threshold"] or dist >= params["max_pass_threshold"]:
            continue

        score = -(params["tm_select_w_dist"] * dist) - (params["tm_select_w_x"] * tm.pos.x)
        if score > best_score:
            best_score = score
            best = tm

    return best


def confidence_factor(last_seen_sec_ago: float, memory_sec: float) -> float:
    # C++: max(0, (5 - elapsed)/5)
    return max(0.0, (memory_sec - last_seen_sec_ago) / memory_sec)


def compute_score_for_target(ball: Pose2D,
                             tm: Pose2D,
                             tx: float, ty: float,
                             opponents: List[Opponent],
                             params: dict) -> float:
    # base score (same algebra as your C++)
    score = (params["base_score"]
             - (abs(tx - tm.x) * params["w_abs_dx"])
             - (abs(ty - tm.y) * params["w_abs_dy"])
             - (tx * params["w_x"])
             - (abs(ty) * params["w_y"]))

    # pass segment
    ax, ay = ball.x, ball.y
    bx, by = tx, ty

    # opponent penalties
    margin = params["opp_path_margin"]
    for opp in opponents:
        if opp.label != "Opponent":
            continue
        cf = confidence_factor(opp.last_seen_sec_ago, params["opp_memory_sec"])
        if cf <= 0.0:
            continue

        d = point_to_segment_distance(opp.pos.x, opp.pos.y, ax, ay, bx, by)
        if d < margin:
            # C++: score -= (1 - distToPassPath) * 20 * confidenceFactor (with margin=1.0)
            # generalized to margin:
            score -= (1.0 - d / margin) * params["opp_penalty"] * cf

    return score


def compute_costmap(ball: Pose2D,
                    tm: Pose2D,
                    opponents: List[Opponent],
                    params: dict) -> Tuple[np.ndarray, np.ndarray, np.ndarray, Tuple[float, float, float]]:
    """Returns (Xgrid, Ygrid, ScoreGrid, best=(tx,ty,score))."""
    hlx = params["field_half_length"]
    hly = params["field_half_width"]
    Rx = params["grid_half_xrange"]
    Ry = params["grid_half_yrange"]
    step = params["grid_step"]

    xs = np.arange(tm.x - Rx, tm.x + Rx + 1e-9, step)
    ys = np.arange(tm.y - Ry, tm.y + Ry + 1e-9, step)

    X, Y = np.meshgrid(xs, ys)
    S = np.full_like(X, fill_value=-1e18, dtype=float)

    best_score = -1e18
    best_tx, best_ty = float("nan"), float("nan")

    for iy in range(Y.shape[0]):
        for ix in range(X.shape[1]):
            tx, ty = float(X[iy, ix]), float(Y[iy, ix])

            # field boundary check (same as your C++)
            if abs(tx) > hlx or abs(ty) > hly:
                continue

            # pass distance threshold check (same as your newly added C++)
            pass_dist = np.hypot(tx - ball.x, ty - ball.y)
            if pass_dist < params["min_pass_threshold"] or pass_dist > params["max_pass_threshold"]:
                continue

            sc = compute_score_for_target(ball, tm, tx, ty, opponents, params)
            S[iy, ix] = sc

            if sc > best_score:
                best_score = sc
                best_tx, best_ty = tx, ty

    return X, Y, S, (best_tx, best_ty, best_score)


def pass_speed_from_distance(ball: Pose2D, tx: float, ty: float, k: float = 1.7, cap: float = 2.0) -> float:
    d = np.hypot(ball.x - tx, ball.y - ty)
    speed = k * np.sqrt(d / 10.0)
    return min(speed, cap)


# =========================
# 4) Visualization
# =========================
def visualize(ball: Pose2D,
              teammates: List[Teammate],
              opponents: List[Opponent],
              my_player_id: int,
              params: dict):
    best_tm = select_best_teammate(ball, teammates, my_player_id, params)
    if best_tm is None:
        print("No pass candidate teammate found (thresholds too strict or no alive teammate).")
        return

    X, Y, S, best = compute_costmap(ball, best_tm.pos, opponents, params)
    tx, ty, best_score = best

    pass_found = best_score >= params["score_threshold"]
    pass_speed = pass_speed_from_distance(ball, tx, ty) if pass_found else None

    print(f"[Teammate Selected] id={best_tm.player_id} pos=({best_tm.pos.x:.2f},{best_tm.pos.y:.2f})")
    print(f"[Best Target] (tx,ty)=({tx:.2f},{ty:.2f}) score={best_score:.2f} pass_found={pass_found}")
    if pass_found:
        print(f"[Pass Speed] speed_limit={pass_speed:.2f}")

    fig = plt.figure(figsize=params["figsize"])
    ax0 = fig.add_subplot(1, 2, 1)
    ax1 = fig.add_subplot(1, 2, 2)

    # --- Panel 1: Field overview ---
    hlx = params["field_half_length"]
    hly = params["field_half_width"]
    ax0.set_title("Field overview")
    ax0.set_xlim(-hlx, hlx)
    ax0.set_ylim(-hly, hly)
    ax0.set_aspect("equal", adjustable="box")
    ax0.grid(True, alpha=0.3)

    # field border
    ax0.plot([-hlx, hlx, hlx, -hlx, -hlx], [-hly, -hly, hly, hly, -hly], linewidth=1)

    # ball
    ax0.scatter([ball.x], [ball.y], marker="o", s=80)
    ax0.annotate("Ball", (ball.x, ball.y))

    # teammates
    if params["show_all_teammates"]:
        for tm in teammates:
            if not tm.is_alive:
                continue
            ax0.scatter([tm.pos.x], [tm.pos.y], marker="^", s=60)
            ax0.annotate(f"TM{tm.player_id}", (tm.pos.x, tm.pos.y))

    # highlight best teammate
    ax0.scatter([best_tm.pos.x], [best_tm.pos.y], marker="^", s=140)
    ax0.annotate(f"BEST TM{best_tm.player_id}", (best_tm.pos.x, best_tm.pos.y))

    # opponents
    for i, opp in enumerate(opponents):
        ax0.scatter([opp.pos.x], [opp.pos.y], marker="x", s=80)
        cf = confidence_factor(opp.last_seen_sec_ago, params["opp_memory_sec"])
        ax0.annotate(f"OPP{i} cf={cf:.2f}", (opp.pos.x, opp.pos.y))

    # pass line to best target
    if pass_found:
        ax0.plot([ball.x, tx], [ball.y, ty], linewidth=2)
        ax0.scatter([tx], [ty], marker="*", s=140)
        ax0.annotate(f"Target\nscore={best_score:.1f}", (tx, ty))

    # --- Panel 2: Heatmap around chosen teammate ---
    ax1.set_title("Score heatmap around chosen teammate")
    ax1.set_aspect("equal", adjustable="box")
    ax1.grid(False)

    # mask invalid cells
    S_plot = S.copy()
    S_plot[S_plot < -1e17] = np.nan

    extent = [np.nanmin(X), np.nanmax(X), np.nanmin(Y), np.nanmax(Y)]
    im = ax1.imshow(S_plot, origin="lower", extent=extent, interpolation="nearest")
    plt.colorbar(im, ax=ax1, fraction=0.046, pad=0.04, label="score")

    ax1.scatter([best_tm.pos.x], [best_tm.pos.y], marker="^", s=120)
    ax1.annotate("TM", (best_tm.pos.x, best_tm.pos.y))

    ax1.scatter([ball.x], [ball.y], marker="o", s=70)
    ax1.annotate("Ball", (ball.x, ball.y))

    for i, opp in enumerate(opponents):
        ax1.scatter([opp.pos.x], [opp.pos.y], marker="x", s=70)
        ax1.annotate(f"OPP{i}", (opp.pos.x, opp.pos.y))

    if pass_found:
        ax1.scatter([tx], [ty], marker="*", s=160)
        ax1.annotate(f"BEST\n{best_score:.1f}", (tx, ty))
        ax1.plot([ball.x, tx], [ball.y, ty], linewidth=1)

    plt.tight_layout()
    plt.show()


# =========================
# 5) Example usage (edit these to your scenario)
# =========================
def main():
    # Example scenario (바꿔서 쓰시면 됩니다)
    my_player_id = 1

    ball = Pose2D(x=0.9, y=0.5)

    teammates = [
        Teammate(player_id=1, pos=Pose2D(1.0,  0.5), is_alive=True),  # me (ignored)
        Teammate(player_id=2, pos=Pose2D(3.5,  0.0), is_alive=True),
        Teammate(player_id=3, pos=Pose2D(-0.5, 0.5), is_alive=True),
    ]

    opponents = [
        Opponent(pos=Pose2D(-3.5, 0.0), last_seen_sec_ago=1.0),
        Opponent(pos=Pose2D(0.0, 1.5), last_seen_sec_ago=0.0),
        Opponent(pos=Pose2D(0.5, -0.5), last_seen_sec_ago=1.5),
    ]

    visualize(ball, teammates, opponents, my_player_id, PARAMS)


if __name__ == "__main__":
    main()
