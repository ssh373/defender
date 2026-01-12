#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Integrated Simulator
- User controls 1 opponent defender with arrow keys (speed=0.5m/s, discrete step per keypress).
- Striker moves off-the-ball according to given logic (speed=0.5m/s).
- Defender (passer) decides pass target via CalcPassDir costmap logic; if pass not found, stands still.

Fixes:
- Colorbar is created ONCE and never duplicated.
- Score colorbar range is fixed to [-30, +10] (static).

Requires: numpy, matplotlib
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple
import numpy as np
import matplotlib.pyplot as plt


# ============================================================
# 0) Parameters (KEEP AS-IS, but separated names to avoid clash)
# ============================================================

# --- Pass (CalcPassDir) params (same values you provided) ---
PASS_PARAMS = {
    # teammate selection
    "min_pass_threshold": 1.0,
    "max_pass_threshold": 4.0,
    "tm_select_w_dist": 1.0,   # score = -(w_dist*dist) - (w_x*tm.x)
    "tm_select_w_x": 1.2,

    # field limits (9x6 => half: 4.5 / 3.0)
    "field_half_length": 4.5,
    "field_half_width": 3.0,

    # grid around chosen teammate
    "grid_half_xrange": 3.0,
    "grid_half_yrange": 2.5,
    "grid_step": 0.2,

    # base score terms
    "base_score": 10.0,
    "w_abs_dx": 1.1,
    "w_abs_dy": 0.7, 
    "w_x": 0.95,
    "w_y": 0.45,

    # opponent penalty near pass path
    "opp_path_margin": 1.15,
    "opp_penalty": 24.0,
    "opp_memory_sec": 5.0,

    # decision threshold
    "score_threshold": 6.5,
}

# --- Striker off-the-ball params (same values you provided) ---
ST_PARAMS = {
    "field_length": 9.0,
    "field_width": 6.0,
    "penalty_dist": 1.5,
    "goal_width": 2.0,
    "circle_radius": 0.75,
    "penalty_area_length": 2.0,
    "penalty_area_width": 5.0,
    "goal_area_length": 1.0,
    "goal_area_width": 3.0,

    "dist_from_goal": 2.0,
    "base_x_weight": 5.0,
    "center_y_weight": 3.0,
    "defender_dist_weight": 20.0,
    "defender_dist_cap": 3.0,

    "hysteresis_x_weight": 3.0,
    "hysteresis_y_weight": 3.0,

    "penalty_weight": 10.0,
    "path_margin": 1.5,

    "opp_memory_sec": 5.0,

    "pass_penalty_weight": 15.0,
    "shot_penalty_weight": 3.0,
    "movement_penalty_weight": 30.0,
    "symmetry_weight": 10.0,
    "ball_dist_weight": 3.0,
    "forward_weight": 5.2,

    "path_confidence": 0.5,

    "search_x_margin": 1.7,
    "grid_step": 0.1,
}

# --- Simulation params ---
SIM = {
    "dt": 0.1,                 # seconds per tick
    "player_speed": 0.5,       # m/s (all move at 0.5 or stop)
    "user_step": 0.1 * 0.5,    # dt * speed = 0.05m per keypress
    "score_vmin": -20.0,       # FIXED colorbar range
    "score_vmax": +10.0,       # FIXED colorbar range
}

# ============================================================
# 1) Data structures
# ============================================================

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
    last_seen_sec_ago: float = 0.0
    label: str = "Opponent"


# ============================================================
# 2) Geometry / utilities
# ============================================================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def point_to_segment_distance(px: float, py: float,
                              ax: float, ay: float,
                              bx: float, by: float) -> float:
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-12:
        return np.hypot(apx, apy)
    t = (apx * abx + apy * aby) / ab2
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * abx, ay + t * aby
    return np.hypot(px - cx, py - cy)

def confidence_factor(last_seen_sec_ago: float, memory_sec: float) -> float:
    return max(0.0, (memory_sec - last_seen_sec_ago) / memory_sec)

def move_towards(cur: Pose2D, target: Pose2D, speed: float, dt: float) -> Pose2D:
    dx, dy = target.x - cur.x, target.y - cur.y
    d = np.hypot(dx, dy)
    if d < 1e-9:
        return Pose2D(cur.x, cur.y)
    step = speed * dt
    if step >= d:
        return Pose2D(target.x, target.y)
    ux, uy = dx / d, dy / d
    return Pose2D(cur.x + ux * step, cur.y + uy * step)


# ============================================================
# 3) PASS logic (CalcPassDir port)
# ============================================================

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


def compute_pass_score_for_target(ball: Pose2D,
                                 tm: Pose2D,
                                 tx: float, ty: float,
                                 opponents: List[Opponent],
                                 params: dict) -> float:
    score = (params["base_score"]
             - (abs(tx - tm.x) * params["w_abs_dx"])
             - (abs(ty - tm.y) * params["w_abs_dy"])
             - (tx * params["w_x"])
             - (abs(ty) * params["w_y"]))

    ax, ay = ball.x, ball.y
    bx, by = tx, ty

    margin = params["opp_path_margin"]
    for opp in opponents:
        if opp.label != "Opponent":
            continue
        cf = confidence_factor(opp.last_seen_sec_ago, params["opp_memory_sec"])
        if cf <= 0.0:
            continue

        d = point_to_segment_distance(opp.pos.x, opp.pos.y, ax, ay, bx, by)
        if d < margin:
            score -= (1.0 - d / margin) * params["opp_penalty"] * cf

    return score


def compute_pass_costmap(ball: Pose2D,
                         tm: Pose2D,
                         opponents: List[Opponent],
                         params: dict) -> Tuple[np.ndarray, np.ndarray, np.ndarray, Tuple[float, float, float]]:
    hlx = params["field_half_length"]
    hly = params["field_half_width"]
    Rx = params["grid_half_xrange"]
    Ry = params["grid_half_yrange"]
    step = params["grid_step"]

    xs = np.arange(tm.x - Rx, tm.x + Rx + 1e-9, step)
    ys = np.arange(tm.y - Ry, tm.y + Ry + 1e-9, step)
    X, Y = np.meshgrid(xs, ys)
    S = np.full_like(X, fill_value=np.nan, dtype=float)

    best_score = -1e18
    best_tx, best_ty = float("nan"), float("nan")

    for iy in range(Y.shape[0]):
        for ix in range(X.shape[1]):
            tx, ty = float(X[iy, ix]), float(Y[iy, ix])

            if abs(tx) > hlx or abs(ty) > hly:
                continue

            # (추가하신) pass distance threshold check
            pass_dist = np.hypot(tx - ball.x, ty - ball.y)
            if pass_dist < params["min_pass_threshold"] or pass_dist > params["max_pass_threshold"]:
                continue

            sc = compute_pass_score_for_target(ball, tm, tx, ty, opponents, params)
            S[iy, ix] = sc
            if sc > best_score:
                best_score = sc
                best_tx, best_ty = tx, ty

    return X, Y, S, (best_tx, best_ty, best_score)


# ============================================================
# 4) STRIKER off-the-ball logic (as given)
# ============================================================

def compute_striker_score(tx: float, ty: float,
                          robot: Pose2D,
                          ball: Pose2D,
                          opponents: List[Opponent],
                          params: dict) -> float:
    fl = params["field_length"]
    goal_x = -(fl / 2.0)
    base_x = goal_x + params["dist_from_goal"]

    score = 0.0
    score -= abs(tx - base_x) * params["base_x_weight"]
    score -= abs(ty) * params["center_y_weight"]
    score -= abs(tx - robot.x) * params["hysteresis_x_weight"]
    score -= abs(ty - robot.y) * params["hysteresis_y_weight"]

    defenders = [opp for opp in opponents if abs(opp.pos.x - goal_x) < 4.0]
    dist_to_defender = 0.0
    normalizer = max(1.0, float(len(defenders)))
    for opp in defenders:
        d = np.hypot(ty - opp.pos.y, tx - opp.pos.x)
        d = min(d, params["defender_dist_cap"])
        dist_to_defender += d
    dist_to_defender /= normalizer
    score += dist_to_defender * params["defender_dist_weight"]

    if defenders:
        avg_opp_y = sum(d.pos.y for d in defenders) / len(defenders)
        sym_target_y = -avg_opp_y
        score -= abs(ty - sym_target_y) * params["symmetry_weight"]

    dist_x_to_ball = abs(tx - ball.x)
    score -= abs(dist_x_to_ball - 2.5) * params["ball_dist_weight"]

    score += (-tx) * params["forward_weight"]

    pass_path = (ball.x, ball.y, tx, ty)
    shot_path = (base_x, ty, goal_x, 0.0)

    for opp in opponents:
        cf = confidence_factor(opp.last_seen_sec_ago, params["opp_memory_sec"])
        if cf <= 0.0:
            continue

        dist_pass = point_to_segment_distance(opp.pos.x, opp.pos.y, *pass_path)
        if dist_pass < params["path_margin"]:
            score -= (params["path_margin"] - dist_pass) * params["penalty_weight"] * cf

        dist_shot = point_to_segment_distance(opp.pos.x, opp.pos.y, *shot_path)
        if dist_shot < params["path_margin"]:
            score -= (params["path_margin"] - dist_shot) * params["penalty_weight"] * cf

    dist_robot_target = np.hypot(tx - robot.x, ty - robot.y)
    if dist_robot_target > 0.1:
        for opp in opponents:
            cf = confidence_factor(opp.last_seen_sec_ago, params["opp_memory_sec"])
            if cf <= 0.0:
                continue

            vec_rt_x = tx - robot.x
            vec_rt_y = ty - robot.y
            vec_ro_x = opp.pos.x - robot.x
            vec_ro_y = opp.pos.y - robot.y

            dot_prod = vec_ro_x * vec_rt_x + vec_ro_y * vec_rt_y
            len_sq = vec_rt_x**2 + vec_rt_y**2
            t = dot_prod / len_sq

            if t < 0.0:
                closest_x, closest_y = robot.x, robot.y
            elif t > 1.0:
                closest_x, closest_y = tx, ty
            else:
                closest_x = robot.x + t * vec_rt_x
                closest_y = robot.y + t * vec_rt_y

            dist_to_path = np.hypot(opp.pos.x - closest_x, opp.pos.y - closest_y)

            if 0.0 < t < 1.0 and dist_to_path < params["path_margin"]:
                penalty = (params["path_margin"] - dist_to_path) * params["movement_penalty_weight"] * cf
                score -= penalty

    return score


def compute_striker_costmap(robot: Pose2D, ball: Pose2D, opponents: List[Opponent], params: dict):
    fl = params["field_length"]
    fw = params["field_width"]
    goal_x = -(fl / 2.0)
    base_x = goal_x + params["dist_from_goal"]
    max_y = fw / 2.0 - 0.5

    xs = np.arange(base_x - params["search_x_margin"], base_x + params["search_x_margin"] + 1e-9, params["grid_step"])
    ys = np.arange(-max_y, max_y + 1e-9, params["grid_step"])
    X, Y = np.meshgrid(xs, ys)

    best_score = -1e9
    best_pos = (base_x, 0.0)

    for iy in range(X.shape[0]):
        for ix in range(X.shape[1]):
            tx, ty = float(X[iy, ix]), float(Y[iy, ix])
            sc = compute_striker_score(tx, ty, robot, ball, opponents, params)
            if sc > best_score:
                best_score = sc
                best_pos = (tx, ty)

    return best_pos, best_score


# ============================================================
# 5) Integrated simulation + visualization (NO colorbar duplication)
# ============================================================

class IntegratedSim:
    def __init__(
        self,
        *,
        # ---- initial positions (Pose2D) ----
        ball: "Pose2D" = None,
        passer: "Pose2D" = None,      # 수비수(패서)
        striker: "Pose2D" = None,     # 스트라이커(off-the-ball)
        gk: "Pose2D" = None,
        # ---- opponents ----
        opp_user: "Pose2D" = None,    # 방향키로 조종할 상대 수비 1명
        opp_gk: "Pose2D" = None,      # 고정 골키퍼
        other_opps: "list[tuple[float,float,float]]" = None,
        # other_opps: [(x,y,last_seen_sec_ago), ...]
    ):
        # -------- defaults (keeps old behavior) --------
        if ball is None:    ball    = Pose2D(x=-2.0, y=-1.9)
        if passer is None:  passer  = Pose2D(x=-2.0, y=-2.0)
        if striker is None: striker = Pose2D(x=-3.0, y=-3.0)
        if gk is None:      gk      = Pose2D(x=3.5, y=0.0)
        if opp_user is None: opp_user = Pose2D(x=-2.5, y=-1.0)
        if opp_gk is None:   opp_gk   = Pose2D(x=-3.5, y=0.0)

        # 상대 골키퍼는 “0.0초에 관측” → confidence=1.0 의미
        self.opp_gk = Opponent(pos=opp_gk, last_seen_sec_ago=0.0)
        self.opp_user = Opponent(pos=opp_user, last_seen_sec_ago=0.0)

        if other_opps is None:
            other_opps = [
                (-1.5, 0.5, 1.0),
            ]

        self.ball = ball
        self.defender = passer
        self.striker = striker
        self.gk = gk

        self.opponents = [self.opp_gk, self.opp_user]
        for (x, y, last_seen) in other_opps:
            self.opponents.append(Opponent(pos=Pose2D(x, y), last_seen_sec_ago=float(last_seen)))

        # cached pass decision outputs
        self.best_tm: Optional[Teammate] = None
        self.pass_target = Pose2D(np.nan, np.nan)
        self.pass_score = -1e18
        self.pass_found = False

        # cached striker outputs
        self.st_target = Pose2D(np.nan, np.nan)
        self.st_score = -1e18

        # setup figure
        self._setup_plot()



    def _setup_plot(self):
        self.fig = plt.figure(figsize=(14, 6))
        self.ax_field = self.fig.add_subplot(1, 2, 1)
        self.ax_heat = self.fig.add_subplot(1, 2, 2)

        # --- heatmap: create ONCE (fix duplication) ---
        dummy = np.zeros((10, 10), dtype=float)
        self.im = self.ax_heat.imshow(
            dummy, origin="lower", extent=[-1, 1, -1, 1],
            vmin=SIM["score_vmin"], vmax=SIM["score_vmax"],
            interpolation="nearest", aspect="equal"
        )
        self.cbar = self.fig.colorbar(self.im, ax=self.ax_heat, fraction=0.046, pad=0.04, label="score")
        # NOTE: colorbar created once and never re-created.

        # texts (update-only, never re-create)
        self.text_info = self.ax_field.text(0.01, 0.99, "", transform=self.ax_field.transAxes,
                                            va="top", ha="left")

        # artists to update
        self.l_pass, = self.ax_field.plot([], [], linewidth=2)      # pass line
        self.l_st, = self.ax_field.plot([], [], linestyle="--", linewidth=2)  # striker target line (optional)
        self.sc_ball = self.ax_field.scatter([], [], s=80, marker="o")
        self.sc_def = self.ax_field.scatter([], [], s=120, marker="s")
        self.sc_st = self.ax_field.scatter([], [], s=120, marker="^")
        self.sc_gk = self.ax_field.scatter([], [], s=90, marker="^")
        self.sc_opp_gk = self.ax_field.scatter([], [], s=90, marker="x")
        self.sc_opp_user = self.ax_field.scatter([], [], s=90, marker="x")
        self.sc_opp_other = self.ax_field.scatter([], [], s=90, marker="x")
        self.sc_pass_target = self.ax_field.scatter([], [], s=160, marker="*")
        self.sc_st_target = self.ax_field.scatter([], [], s=160, marker="*")

        # connect key handler
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

        # timer loop
        self.timer = self.fig.canvas.new_timer(interval=int(SIM["dt"] * 1000))
        self.timer.add_callback(self.on_tick)
        self.timer.start()

        self._draw_field_static()

    def _draw_field_static(self):
        self.ax_field.clear()
        self.ax_field.set_title("Integrated Field View (Arrow keys: move 1 opponent defender)")
        self.ax_field.set_aspect("equal", adjustable="box")
        self.ax_field.grid(True, alpha=0.25)

        hlx = PASS_PARAMS["field_half_length"]
        hly = PASS_PARAMS["field_half_width"]
        self.ax_field.set_xlim(-hlx, hlx)
        self.ax_field.set_ylim(-hly, hly)

        # border
        self.ax_field.plot([-hlx, hlx, hlx, -hlx, -hlx], [-hly, -hly, hly, hly, -hly], linewidth=2)

        # center line & circle (from ST_PARAMS)
        self.ax_field.plot([0, 0], [-hly, hly], linewidth=1)
        self.ax_field.add_patch(plt.Circle((0, 0), ST_PARAMS["circle_radius"], fill=False, linewidth=1))

        # re-add artists (because ax_field was cleared)
        self.text_info = self.ax_field.text(0.01, 0.99, "", transform=self.ax_field.transAxes,
                                            va="top", ha="left")

        self.l_pass, = self.ax_field.plot([], [], linewidth=2)
        self.l_st, = self.ax_field.plot([], [], linestyle="--", linewidth=2)

        self.sc_ball = self.ax_field.scatter([], [], s=80, marker="o")
        self.sc_def = self.ax_field.scatter([], [], s=120, marker="s")
        self.sc_st = self.ax_field.scatter([], [], s=120, marker="^")
        self.sc_gk = self.ax_field.scatter([], [], s=90, marker="^")
        self.sc_opp_gk = self.ax_field.scatter([], [], s=90, marker="x")
        self.sc_opp_user = self.ax_field.scatter([], [], s=90, marker="x")
        self.sc_opp_other = self.ax_field.scatter([], [], s=90, marker="x")
        self.sc_pass_target = self.ax_field.scatter([], [], s=160, marker="*")
        self.sc_st_target = self.ax_field.scatter([], [], s=160, marker="*")

    def on_key(self, event):
        # Discrete move per keypress: step = speed*dt = 0.1m (0.5m/s, dt=0.2s)
        step = SIM["user_step"]
        hlx = PASS_PARAMS["field_half_length"]
        hly = PASS_PARAMS["field_half_width"]

        if event.key == "up":
            self.opp_user.pos.y += step
        elif event.key == "down":
            self.opp_user.pos.y -= step
        elif event.key == "left":
            self.opp_user.pos.x -= step
        elif event.key == "right":
            self.opp_user.pos.x += step
        else:
            return

        # clamp to field
        self.opp_user.pos.x = clamp(self.opp_user.pos.x, -hlx, hlx)
        self.opp_user.pos.y = clamp(self.opp_user.pos.y, -hly, hly)

    def _update_logic(self):
        # ball at passer
        self.ball.x, self.ball.y = self.defender.x, self.defender.y

        opponents = self.opponents

        # --- striker: compute best target and move at 0.5m/s ---
        best_pos, best_sc = compute_striker_costmap(self.striker, self.ball, opponents, ST_PARAMS)
        self.st_target = Pose2D(best_pos[0], best_pos[1])
        self.st_score = best_sc
        self.striker = move_towards(self.striker, self.st_target, SIM["player_speed"], SIM["dt"])

        # --- pass: teammates list includes striker and GK ---
        teammates = [
            Teammate(player_id=1, pos=Pose2D(self.defender.x, self.defender.y), is_alive=True),  # me (ignored)
            Teammate(player_id=2, pos=Pose2D(self.striker.x, self.striker.y), is_alive=True),
            Teammate(player_id=3, pos=Pose2D(self.gk.x, self.gk.y), is_alive=True),
        ]

        best_tm = select_best_teammate(self.ball, teammates, 1, PASS_PARAMS)
        self.best_tm = best_tm

        if best_tm is None:
            self.pass_found = False
            self.pass_target = Pose2D(np.nan, np.nan)
            self.pass_score = -1e18
            self.last_costmap = None
            return

        X, Y, S, best = compute_pass_costmap(self.ball, best_tm.pos, opponents, PASS_PARAMS)
        tx, ty, sc = best
        self.pass_target = Pose2D(tx, ty)
        self.pass_score = sc
        self.pass_found = (sc >= PASS_PARAMS["score_threshold"])
        self.last_costmap = (X, Y, S)

    def _update_plot(self):
        # --- update field artists ---
        self.sc_ball.set_offsets([[self.ball.x, self.ball.y]])
        self.sc_def.set_offsets([[self.defender.x, self.defender.y]])
        self.sc_st.set_offsets([[self.striker.x, self.striker.y]])
        self.sc_gk.set_offsets([[self.gk.x, self.gk.y]])

        self.sc_opp_gk.set_offsets([[self.opp_gk.pos.x, self.opp_gk.pos.y]])
        self.sc_opp_user.set_offsets([[self.opp_user.pos.x, self.opp_user.pos.y]])
        
        # Multiple other opponents
        others = self.opponents[2:]
        if others:
            pts = [[o.pos.x, o.pos.y] for o in others]
            self.sc_opp_other.set_offsets(pts)
        else:
            self.sc_opp_other.set_offsets(np.empty((0, 2)))

        # pass line / target
        if self.pass_found and np.isfinite(self.pass_target.x):
            self.l_pass.set_data([self.ball.x, self.pass_target.x], [self.ball.y, self.pass_target.y])
            self.sc_pass_target.set_offsets([[self.pass_target.x, self.pass_target.y]])
        else:
            self.l_pass.set_data([], [])
            self.sc_pass_target.set_offsets(np.empty((0, 2)))

        # striker target marker/line (optional)
        if np.isfinite(self.st_target.x):
            self.l_st.set_data([self.striker.x, self.st_target.x], [self.striker.y, self.st_target.y])
            self.sc_st_target.set_offsets([[self.st_target.x, self.st_target.y]])
        else:
            self.l_st.set_data([], [])
            self.sc_st_target.set_offsets(np.empty((0, 2)))

        # info text
        bt = f"PASS best_tm=TM{self.best_tm.player_id}" if self.best_tm else "PASS best_tm=None"
        info = (
            f"OPP_USER: ({self.opp_user.pos.x:+.2f}, {self.opp_user.pos.y:+.2f})  speed=0.5m/s (arrow keys)\n"
            f"ST target: ({self.st_target.x:+.2f}, {self.st_target.y:+.2f})  score={self.st_score:+.1f}\n"
            f"{bt}  target=({self.pass_target.x:+.2f}, {self.pass_target.y:+.2f})  "
            f"score={self.pass_score:+.1f}  found={self.pass_found}"
        )
        self.text_info.set_text(info)

        # --- update heatmap (NO new colorbar) ---
        self.ax_heat.set_title("Pass Score Heatmap (around selected best teammate)")
        self.ax_heat.set_aspect("equal", adjustable="box")

        if getattr(self, "last_costmap", None) is None:
            self.im.set_data(np.zeros((10, 10)))
            self.im.set_extent([-1, 1, -1, 1])
        else:
            X, Y, S = self.last_costmap
            # keep NaNs as NaN (imshow will just not draw them; fine)
            self.im.set_data(S)
            self.im.set_extent([np.nanmin(X), np.nanmax(X), np.nanmin(Y), np.nanmax(Y)])

        # colorbar range fixed (static)
        self.im.set_clim(SIM["score_vmin"], SIM["score_vmax"])

        # NOTE: DO NOT call plt.colorbar / fig.colorbar here.

        self.fig.canvas.draw_idle()

    def on_tick(self):
        self._update_logic()
        self._update_plot()

    def run(self):
        plt.show()


def main():
    # ===== 초기 위치를 여기서 간단히 설정 =====
    init_ball    = Pose2D(x=0.0, y=0.0)
    init_passer  = Pose2D(x=0.1, y=0.0)   # 패서(수비수)
    init_striker = Pose2D(x=-3.0, y=3.0)   # 스트라이커
    init_gk     = Pose2D(x=3.5,  y=0.0)
    init_opp_user = Pose2D(x=-1.8, y=0.5)  # 방향키로 조종할 상대 수비 1명
    init_opp_gk   = Pose2D(x=-3.8, y=0.5)   # 고정 골키퍼(0.0초 관측)

    other_opps = [
        (-3.4, 2.0, 1.0),  # (x,y,last_seen_sec_ago)
    ]

    sim = IntegratedSim(
        ball=init_ball,
        passer=init_passer,
        striker=init_striker,
        gk=init_gk,
        opp_user=init_opp_user,
        opp_gk=init_opp_gk,
        other_opps=other_opps,
    )
    sim.run()


if __name__ == "__main__":
    main()
