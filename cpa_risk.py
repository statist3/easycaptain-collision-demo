import csv
import math 

import sys 
from dataclasses import dataclass
from typing import List, Tuple 

import matplotlib.pyplot as plt


@dataclass
class Target:
    id: str 
    x: float 
    y: float 
    speed: float 
    course_deg: float 

@dataclass 
class CpaResult:
    cpa_distance: float 
    tcpa: float 
    collision_risk: bool 
    closing: bool 
    valid: bool 


CPA_THRESHOLD_METERS = 50.0 
TCPA_THRESHOLD_SECONDS = 30.0 


def load_targets_from_csv(path: str) -> List[Target]:
    targets: List[Target] = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            targets.append(
                Target(
                    id = str(row["id"]),
                    x = float(row["x"]),
                    y = float(row["y"]),
                    speed = float(row["speed"]),
                    course_deg = float(row["course"]),
                )
            )
    return targets 

def cource_to_velocity(speed: float, course_deg: float) -> Tuple[float, float]:
    rad = math.radians(course_deg)
    vx = speed * math.cos(rad)
    vy = speed * math.sin(rad)
    return vx, vy

def compute_cpa(
        own_pos: Tuple[float, float],
        own_vel: Tuple[float, float],
        tgt_pos: Tuple[float, float],
        tgt_vel: Tuple[float, float],
) -> CpaResult:
    x_own, y_own = own_pos 
    vx_own, vy_own = own_vel 
    x_t, y_t = tgt_pos 
    vx_t, vy_t = tgt_vel 

    rx = x_t - x_own 
    ry = y_t - y_own 

    vx_rel = vx_t - vx_own
    vy_rel = vy_t - vy_own 

    v2 = vx_rel * vx_rel + vy_rel * vy_rel
    eps = 1e-9 

    # if the relative speed is almost zero, the objects are standing/moving parallel
    if v2 < eps: 
        dist = math.hypot(rx, ry)
        return CpaResult(
            cpa_distance = dist, 
            tcpa = 0.0,
            collision_risk= False,
            closing= False,
            valid= False
        )
    
    dot = rx * vx_rel + ry * vy_rel 
    tcpa = -dot/v2

    # position at the time of CPA
    rx_cpa = rx + vx_rel * tcpa 
    ry_cpa = ry + vy_rel * tcpa
    cpa_dist = math.hypot(rx_cpa, ry_cpa)

    closing = tcpa >= 0.0 
    collision_risk = (
        closing and 
        (cpa_dist < CPA_THRESHOLD_METERS) and 
        (tcpa < TCPA_THRESHOLD_SECONDS)
    )

    return CpaResult(
        cpa_distance=cpa_dist,
        tcpa = tcpa,
        collision_risk=collision_risk,
        closing=closing,
        valid= True
    )

def print_results(targets: List[Target], results: List[CpaResult]) -> None:
    for t, r in zip(targets, results):
        status = "COLLISION RISK" if r.collision_risk else "Safe"
        print(
            f"Target {t.id}: "
            f"CPA={r.cpa_distance:.1f}m, "
            f"TCPA={r.tcpa:.1f}s "
            f"{status}"
        )

def plot_scene(targets: List[Target], results: List[CpaResult], own_x: float, own_y: float) -> None:
    xs_safe, ys_safe = [], []
    xs_risk, ys_risk = [], []

    for t, r in zip(targets, results):
        if r.collision_risk:
            xs_risk.append(t.x)
            ys_risk.append(t.y)
        else:
            xs_safe.append(t.x)
            ys_safe.append(t.y)

    plt.figure(figsize=(6,6))

    # Our catamaran is the blue dot
    plt.scatter([own_x], [own_y], c = "blue", label="Ownship")

    # Safe targets - red
    if xs_safe:
        plt.scatter(xs_safe, ys_safe, c = "red", label="Targets (safe)")

    if xs_risk:
        plt.scatter(xs_risk, ys_risk, c = "green", label = "Collision risk")

    plt.axhline(0, color = "black", linewidth = 0.5)
    plt.axvline(0, color = "black", linewidth = 0.5)
    plt.gca().set_aspect("equal", "box")
    plt.grid(True, linestyle="--", linewidth = 0.5)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("CPA/TCPA scenario")
    plt.legend()
    plt.tight_layout()
    plt.savefig('1.png')


def main() -> None:

    csv_path = sys.argv[1]
    targets = load_targets_from_csv(csv_path)

    if not targets:
        print("No targets_from CSV.")
        sys.exit(1)

    own_pos = (0.0, 0.0)
    own_speed = 20.0
    own_cource_deg = 30.0
    own_vel = cource_to_velocity(own_speed, own_cource_deg)

    results: List[CpaResult] = []

    for t in targets:
        tgt_pos = (t.x, t.y)
        tgt_vel = cource_to_velocity(t.speed, t.course_deg)
        r = compute_cpa(own_pos, own_vel, tgt_pos, tgt_vel)
        results.append(r)

    print_results(targets, results)

    plot_scene(targets,results, own_pos[0], own_pos[1])


if __name__ == "__main__":
    main()
