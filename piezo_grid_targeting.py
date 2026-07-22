"""
Arbitrary-point targeting test for the piezo steering mirror.

Where the master / cross scans *characterize* the mirror (sensitivity,
hysteresis, per-axis symmetry), this experiment *uses* that characterization
open-loop: given the two axes' sensitivities, can the mirror be commanded to
land the beam on an arbitrary requested point on the detector face, without any
feedback? For each point on a grid we invert the (supposed) sensitivities to a
voltage, apply it blind, measure where the beam actually landed, and score the
miss. The headline output is a 2D map of the detector face showing every
requested target, where the beam actually went, and how far off it was.

Method (mirrors the numbered plan):
  1. Sensitivities of each axis are plug-in-able constants below. Two knobs steer
     the 2D detector face: piezo X lands on camera Y, piezo Y lands on camera X
     (camera mounted rotated 90 deg; piezo Z also lands on camera Y but is
     redundant here). Give each axis's *signed* sensitivity in microns of beam
     motion on the camera per volt. Characterization reports urad/V, so
     urad_per_V_to_cam_um_per_V() converts (a mirror tilt theta moves the beam
     2*theta over the detector distance).
  2. Park both axes at their center voltages and read the beam's Gaussian-fit
     center -- that measured spot is the origin the grid is built around.
  3. Build an N x N grid of target points spanning the reachable box around the
     origin (shrunk by GRID_FILL to stay off the voltage rails). Applied in
     raster ("dictionary") order or shuffled (RANDOMIZE_ORDER) so systematic
     drift can't masquerade as a smooth field.
  4. For each target, invert the sensitivities to the two voltages that should
     reach it, clip-check against the device limit, apply blind, settle, and
     measure the landing spot.
  5. Save a CSV of every (target, applied voltages, hit, miss) and render the 2D
     detector-face map: targets, hits, target->hit miss vectors, colored by miss
     distance, annotated with mean / median / max / RMS miss.

Hardware (same stack as piezo_triangle_scan.py):
  - Thorlabs MDT693B piezo controller, driven via ./MDT_COMMAND_LIB.py
  - Thorlabs BC1-series beam profiler, driven via ./tlbc1.py
"""

import csv
import datetime
import os
import time

import matplotlib.pyplot as plt
import numpy as np

from MDT_COMMAND_LIB import (
    mdtListDevices, mdtOpen, mdtClose, mdtGetLimtVoltage,
)

import tlbc1
from piezo_triangle_scan import Sensor, _SET_AXIS, _GET_AXIS, CAM_OF_PIEZO

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------

# --- (1) plug-in-able sensitivities ---------------------------------------
# Distance from the steering mirror to the beam-profiler sensor (meters). Only
# used to convert characterized urad/V into camera um/V; keep in sync with the
# analysis notebooks.
DETECTOR_DISTANCE_M = 0.42


def urad_per_V_to_cam_um_per_V(urad_per_V):
    """Mirror-angle sensitivity (urad/V, as analysis.ipynb reports) -> camera
    sensitivity (um of beam motion per volt). A mirror tilted by theta steers the
    beam by 2*theta, which over distance L lands 2*theta*L off-center; with theta
    in urad and L in m the unit factors cancel to give microns directly."""
    return urad_per_V * 2.0 * DETECTOR_DISTANCE_M


# Signed camera displacement per volt for each piezo axis. The SIGN is the
# direction the beam moves on the camera as the voltage rises -- measure it once
# (e.g. from a triangle scan's transfer curve) and plug it in; a wrong sign just
# sends every target to the opposite corner. Piezo X lands on camera Y, piezo Y
# on camera X. Defaults are representative values from the 7.15 master run.
SENS_PIEZO_X_CAM_UM_PER_V = urad_per_V_to_cam_um_per_V(2.6)   # -> camera Y
SENS_PIEZO_Y_CAM_UM_PER_V = urad_per_V_to_cam_um_per_V(1.9)   # -> camera X

# --- operating point & grid -----------------------------------------------
V_CENTER_X = 75.0        # piezo X park voltage that defines the origin (V)
V_CENTER_Y = 75.0        # piezo Y park voltage that defines the origin (V)
V_MIN = 1.0              # never command below this (convention: stay off 0 V)
GRID_N = 5               # grid is GRID_N x GRID_N target points
GRID_FILL = 0.8          # span this fraction of the reachable box (keep off rails)
RANDOMIZE_ORDER = True   # apply targets shuffled (True) or in raster order (False)
RANDOM_SEED = 0          # seed for the shuffle, for reproducibility

# --- timing / measurement --------------------------------------------------
SETTLE_TIME = 1.0        # seconds to let both axes settle after a large jump
N_AVG = 3                # frames averaged per position measurement
FIXED_EXPOSURE_MS = 0.5  # exposure time (ms) used for the whole session
OUTPUT_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scans")

# camera column ("x"/"y") steered by each piezo axis, from the shared rotation
# map. Two axes on distinct columns are enough to reach any 2D point; Z shares
# camera Y with X and is unused here.
CAM_X_AXIS = next(a for a, c in CAM_OF_PIEZO.items() if c == "X")   # piezo Y
CAM_Y_AXIS = next(a for a, c in CAM_OF_PIEZO.items() if c == "Y")   # piezo X (Z also, unused)


def read_beam(bc1_vi, sensor, n_avg=1):
    """Measure the beam's Gaussian-fit center on the camera, averaged over n_avg
    frames. Returns (cam_x_um, cam_y_um, rating_x, rating_y) with positions in
    microns relative to the sensor center -- the same convention as the
    gaussfit_center_{x,y}_um columns written by piezo_triangle_scan.py."""
    xs, ys, rx, ry = [], [], [], []
    for _ in range(max(1, n_avg)):
        scan = tlbc1.get_scan_data(bc1_vi)
        xs.append((scan.gaussianFitCentroidPositionX - sensor.center_x) * sensor.pitch_h)
        ys.append((scan.gaussianFitCentroidPositionY - sensor.center_y) * sensor.pitch_v)
        rx.append(scan.gaussianFitRatingX)
        ry.append(scan.gaussianFitRatingY)
    return float(np.mean(xs)), float(np.mean(ys)), float(np.mean(rx)), float(np.mean(ry))


def reachable_halfspan(sens_um_per_v, v_center, v_min, v_limit):
    """Half-width (camera um) of the symmetric reachable interval around the
    origin for one axis: how far the beam can move in *either* direction before
    the axis hits a voltage rail (V_MIN or v_limit). The smaller of the two
    one-sided reaches bounds a symmetric box, so a grid built on it is fully
    reachable on every side. Returns a non-negative micron distance."""
    reach_up = abs(sens_um_per_v) * (v_limit - v_center)
    reach_dn = abs(sens_um_per_v) * (v_center - v_min)
    return max(0.0, min(reach_up, reach_dn))


def build_grid(cam_x0, cam_y0, half_x, half_y, n):
    """N x N target points (camera um) centered on (cam_x0, cam_y0), spanning
    +/- half_x by +/- half_y. Returned in raster order as a list of
    (row, col, target_x_um, target_y_um); row/col index the grid so a shuffled
    application order can still be tied back to a grid cell."""
    xs = np.linspace(cam_x0 - half_x, cam_x0 + half_x, n)
    ys = np.linspace(cam_y0 - half_y, cam_y0 + half_y, n)
    grid = []
    for r, ty in enumerate(ys):
        for c, tx in enumerate(xs):
            grid.append((r, c, float(tx), float(ty)))
    return grid


def target_to_voltages(target_x_um, target_y_um, cam_x0, cam_y0):
    """Invert the (supposed) sensitivities to the two voltages that should land
    the beam on (target_x_um, target_y_um): camera X is driven by piezo Y and
    camera Y by piezo X. Returns (v_x, v_y) piezo voltages (unclipped)."""
    v_y = V_CENTER_Y + (target_x_um - cam_x0) / SENS_PIEZO_Y_CAM_UM_PER_V
    v_x = V_CENTER_X + (target_y_um - cam_y0) / SENS_PIEZO_X_CAM_UM_PER_V
    return v_x, v_y


def save_results(rows, out_dir, run_stamp):
    """Write the per-target CSV; return its path."""
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, f"grid_targeting_{run_stamp}.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "apply_order", "grid_row", "grid_col",
            "target_cam_x_um", "target_cam_y_um",
            "v_x_cmd_V", "v_y_cmd_V", "v_x_applied_V", "v_y_applied_V",
            "v_x_readback_V", "v_y_readback_V",
            "hit_cam_x_um", "hit_cam_y_um", "miss_um",
            "gaussfit_rating_x", "gaussfit_rating_y", "reachable",
        ])
        writer.writerows(rows)
    print(f"Saved {len(rows)} targets to {csv_path}")
    return csv_path


def plot_targeting(rows, cam_x0, cam_y0, out_dir, run_stamp):
    """2D detector-face map: requested targets (open squares), measured landings
    (dots colored by miss distance), target->hit miss vectors, and the origin.
    Only reachable targets that produced a valid measurement are plotted. Saves
    a PNG beside the CSV and returns its path."""
    hit = [r for r in rows if r[16] and not np.isnan(r[13])]   # reachable & measured
    fig, ax = plt.subplots(figsize=(8, 7))

    if hit:
        tx = np.array([r[3] for r in hit])
        ty = np.array([r[4] for r in hit])
        hx = np.array([r[11] for r in hit])
        hy = np.array([r[12] for r in hit])
        miss = np.array([r[13] for r in hit])

        # miss vectors target -> hit
        ax.quiver(tx, ty, hx - tx, hy - ty, angles="xy", scale_units="xy",
                  scale=1, color="gray", width=0.003, alpha=0.6,
                  label="miss vector")
        ax.scatter(tx, ty, marker="s", facecolors="none", edgecolors="tab:blue",
                   s=70, label="requested target")
        sc = ax.scatter(hx, hy, c=miss, cmap="viridis", s=45, zorder=3,
                        label="measured landing")
        cbar = fig.colorbar(sc, ax=ax)
        cbar.set_label("Miss distance (um)")

    unreachable = [r for r in rows if not r[16]]
    if unreachable:
        ax.scatter([r[3] for r in unreachable], [r[4] for r in unreachable],
                   marker="x", color="tab:red", s=60,
                   label="unreachable target (clipped)")

    ax.scatter([cam_x0], [cam_y0], marker="+", color="black", s=140,
               linewidths=2, label="origin (V center)")
    ax.set_xlabel("Camera X (um)  [driven by piezo Y]")
    ax.set_ylabel("Camera Y (um)  [driven by piezo X]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="best", fontsize=8, framealpha=0.9)

    if hit:
        miss = np.array([r[13] for r in hit])
        title = (f"Open-loop targeting: {len(hit)}/{len(rows)} points hit  |  "
                 f"miss mean {miss.mean():.1f}  median {np.median(miss):.1f}  "
                 f"max {miss.max():.1f}  RMS {np.sqrt(np.mean(miss ** 2)):.1f} um")
    else:
        title = "Open-loop targeting (no valid landings)"
    ax.set_title(title, fontsize=10)
    fig.tight_layout()

    png_path = os.path.join(out_dir, f"grid_targeting_{run_stamp}.png")
    fig.savefig(png_path, dpi=150)
    print(f"Saved targeting map to {png_path}")
    return png_path


def main():
    # --- connect to the piezo controller ---
    devs = mdtListDevices()
    print("Piezo devices found:", devs)
    if not devs:
        print("No MDT693B/694B devices connected.")
        return

    piezo_hdl = mdtOpen(devs[0][0], 115200, 3)
    if piezo_hdl < 0:
        print("Failed to open piezo device", devs[0][0])
        return
    print("Connected to piezo", devs[0][0])

    limit_voltage = [0]
    mdtGetLimtVoltage(piezo_hdl, limit_voltage)
    v_limit = limit_voltage[0]
    print(f"Piezo voltage limit: {v_limit} V")

    # --- connect to the beam profiler ---
    try:
        bc1_vi, bc1_info = tlbc1.open_first_device()
    except RuntimeError as e:
        print(e)
        mdtClose(piezo_hdl)
        return
    print("Connected to beam profiler", bc1_info["model_name"], bc1_info["serial_number"])

    pixel_count_x, pixel_count_y, pixel_pitch_h, pixel_pitch_v = tlbc1.get_sensor_information(bc1_vi)
    sensor = Sensor(pixel_count_x / 2.0, pixel_count_y / 2.0, pixel_pitch_h, pixel_pitch_v)

    # disable auto-exposure for the whole session (restored in finally)
    prev_auto_exposure = tlbc1.get_auto_exposure(bc1_vi)
    prev_exposure_time = tlbc1.get_exposure_time(bc1_vi)
    tlbc1.set_auto_exposure(bc1_vi, False)
    tlbc1.set_exposure_time(bc1_vi, FIXED_EXPOSURE_MS)

    run_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(OUTPUT_ROOT, f"targeting_{run_stamp}")

    rows = []
    try:
        # --- (2) park at the center voltages and read the origin ---
        if not (V_MIN <= V_CENTER_X <= v_limit and V_MIN <= V_CENTER_Y <= v_limit):
            print(f"Center voltages ({V_CENTER_X}, {V_CENTER_Y}) V outside "
                  f"[{V_MIN}, {v_limit}] V; adjust V_CENTER_X/Y.")
            return
        _SET_AXIS[CAM_Y_AXIS](piezo_hdl, float(V_CENTER_X))   # piezo X
        _SET_AXIS[CAM_X_AXIS](piezo_hdl, float(V_CENTER_Y))   # piezo Y
        time.sleep(SETTLE_TIME)
        cam_x0, cam_y0, _, _ = read_beam(bc1_vi, sensor, N_AVG)
        print(f"Origin at V=({V_CENTER_X:.1f}, {V_CENTER_Y:.1f}) V -> "
              f"beam ({cam_x0:.1f}, {cam_y0:.1f}) um on camera")

        # --- (3) build the reachable grid ---
        half_x = GRID_FILL * reachable_halfspan(
            SENS_PIEZO_Y_CAM_UM_PER_V, V_CENTER_Y, V_MIN, v_limit)   # camera X <- piezo Y
        half_y = GRID_FILL * reachable_halfspan(
            SENS_PIEZO_X_CAM_UM_PER_V, V_CENTER_X, V_MIN, v_limit)   # camera Y <- piezo X
        grid = build_grid(cam_x0, cam_y0, half_x, half_y, GRID_N)
        print(f"Reachable box (fill {GRID_FILL:g}): +/-{half_x:.1f} um (cam X) x "
              f"+/-{half_y:.1f} um (cam Y); {len(grid)} targets")

        order = list(range(len(grid)))
        if RANDOMIZE_ORDER:
            np.random.default_rng(RANDOM_SEED).shuffle(order)
            print(f"Applying targets in randomized order (seed {RANDOM_SEED}).")
        else:
            print("Applying targets in raster order.")

        # --- (4) reach each target open-loop and measure the landing ---
        for k, gi in enumerate(order):
            r, c, tx, ty = grid[gi]
            v_x_cmd, v_y_cmd = target_to_voltages(tx, ty, cam_x0, cam_y0)
            v_x = min(v_limit, max(V_MIN, v_x_cmd))
            v_y = min(v_limit, max(V_MIN, v_y_cmd))
            reachable = (abs(v_x - v_x_cmd) < 1e-6) and (abs(v_y - v_y_cmd) < 1e-6)

            _SET_AXIS[CAM_Y_AXIS](piezo_hdl, float(v_x))   # piezo X drives camera Y
            _SET_AXIS[CAM_X_AXIS](piezo_hdl, float(v_y))   # piezo Y drives camera X
            time.sleep(SETTLE_TIME)

            if reachable:
                hx, hy, rate_x, rate_y = read_beam(bc1_vi, sensor, N_AVG)
                miss = float(np.hypot(hx - tx, hy - ty))
            else:
                hx = hy = miss = np.nan
                rate_x = rate_y = np.nan

            vx_rb, vy_rb = [0.0], [0.0]
            _GET_AXIS[CAM_Y_AXIS](piezo_hdl, vx_rb)
            _GET_AXIS[CAM_X_AXIS](piezo_hdl, vy_rb)

            rows.append([
                k, r, c, tx, ty, v_x_cmd, v_y_cmd, v_x, v_y,
                vx_rb[0], vy_rb[0], hx, hy, miss, rate_x, rate_y, reachable,
            ])
            tag = "" if reachable else "  [UNREACHABLE -> clipped, skipped]"
            miss_str = f"{miss:6.1f}" if reachable else "   n/a"
            print(f"[{k + 1:>2}/{len(grid)}] target=({tx:7.1f},{ty:7.1f}) um  "
                  f"V=({v_x:5.1f},{v_y:5.1f})  hit=({hx if reachable else float('nan'):7.1f},"
                  f"{hy if reachable else float('nan'):7.1f})  miss={miss_str} um{tag}")

    except KeyboardInterrupt:
        print(f"\nInterrupted -- {len(rows)} targets collected so far.")

    finally:
        for setter in _SET_AXIS.values():
            setter(piezo_hdl, 0.0)
        mdtClose(piezo_hdl)
        tlbc1.set_exposure_time(bc1_vi, prev_exposure_time)
        tlbc1.set_auto_exposure(bc1_vi, prev_auto_exposure)
        tlbc1.close(bc1_vi)
        print("Devices closed.")

    # --- (5) save + plot ---
    if rows:
        save_results(rows, out_dir, run_stamp)
        plot_targeting(rows, cam_x0, cam_y0, out_dir, run_stamp)
        hit = [r for r in rows if r[16] and not np.isnan(r[13])]
        if hit:
            miss = np.array([r[13] for r in hit])
            print(f"\nTargeting summary: {len(hit)}/{len(rows)} reachable & measured; "
                  f"miss mean {miss.mean():.1f} um, median {np.median(miss):.1f} um, "
                  f"max {miss.max():.1f} um, RMS {np.sqrt(np.mean(miss ** 2)):.1f} um.")
        plt.show(block=False)
        plt.pause(0.1)
    else:
        print("No targets collected.")


if __name__ == "__main__":
    main()
