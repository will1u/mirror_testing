"""
Cross-talk test for the piezo steering mirror: does driving one axis disturb
the *other* axis's beam position?

Where piezo_xy_cross_scan.py parks X and sweeps Y to ask how Y's own
sensitivity depends on where X sits, this experiment asks the complementary
question -- while one axis (HELD) is parked at a fixed setpoint and the other
(DRIVEN) is swept full-range, does the beam wander on the HELD axis's detector
column? Any motion there is cross-talk leaking from the driven axis into the
held one (mechanical coupling in the mount, electrical coupling in the
controller, or shared-ground pickup).

The camera is mounted rotated 90 deg, so each piezo axis lands on a distinct
detector column: piezo X (and Z) -> camera Y, piezo Y -> camera X
(CAM_OF_PIEZO). We therefore read cross-talk off the HELD axis's column while
the DRIVEN axis's own (intended) motion shows on the other column. The two axes
must map to *different* camera columns or their motions can't be told apart
optically -- X+Z is rejected, same guard as the master scan.

Method:
  1. Park the held axis at each HELD_SETPOINT; sweep the driven axis with a
     multi-cycle triangle wave (reusing piezo_triangle_scan.run_triangle_scan,
     so every sub-scan is a standard CSV that logs BOTH camera columns).
  2. From each sub-scan, fit:
       - the DRIVEN axis's own sensitivity (its column vs. its voltage), and
       - the CROSS-TALK slope (the HELD axis's column vs. the driven voltage).
     Report the coupling ratio = cross-talk slope / driven sensitivity (the
     fraction of driven motion that leaks into the held axis) and the held
     column's residual scatter (instability induced beyond the linear leak).
  3. Plot the held-axis cross-talk transfer next to the driven-axis intended
     transfer (same x-axis) so the leaked motion can be compared to the motion
     that caused it, and -- across multiple held setpoints -- the coupling ratio
     vs. held setpoint.

All displacements are converted to mirror tilt angle (urad) the same way as the
analysis notebooks (theta = d / (2 L), a mirror tilt steers the beam by 2 theta
over the mirror->detector distance). Output goes to scans/crosstalk_<stamp>/:
one tagged sub-scan CSV per held setpoint, a crosstalk_summary.csv of the
metrics, and the plots. Press Ctrl+C to stop early -- finished sub-scans and
their metrics are still saved and plotted.

Hardware:
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
from piezo_triangle_scan import Sensor, run_triangle_scan, _SET_AXIS, CAM_OF_PIEZO

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
HELD_AXIS = "X"            # axis parked at a fixed setpoint; its column carries the cross-talk
DRIVEN_AXIS = "Y"          # axis swept full-range; the disturbance source
# Held setpoints (V) to repeat the test at, to see if cross-talk depends on where
# the held axis sits. Use one for a quick check, several to map the dependence.
HELD_SETPOINTS = [75.0]
# HELD_SETPOINTS = [30.0, 75.0, 120.0]
V_SWEEP_MIN = 5.0          # driven-axis triangle low voltage (V); off 0 per convention
V_SWEEP_MAX = 150.0        # driven-axis triangle high voltage (V), clipped to device limit
POINTS_PER_RAMP = 30       # samples per up/down ramp of the driven sweep
N_CYCLES = 3               # triangle cycles per sweep (>=2 exposes any induced hysteresis)
SETTLE_TIME = 0.02         # seconds after setting the driven voltage before reading
HELD_SETTLE_TIME = 1.0     # seconds to let the held axis settle after parking it
FIXED_EXPOSURE_MS = 0.5    # exposure time (ms) used for the whole session
SUBSCAN_BUFFER_TIME = 0.1
DETECTOR_DISTANCE_M = 0.42  # mirror->sensor distance (m); keep in sync with the notebooks
OUTPUT_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scans")

# row layout emitted by run_triangle_scan (see piezo_triangle_scan.save_results):
# [i, ts, v_set, v_readback, centroid_x, centroid_y, gfit_x, gfit_y, ...]
_V_READBACK = 3
_GFIT_UM = {"X": 6, "Y": 7}   # gaussfit-center um column index for each CAMERA axis


def um_to_mirror_urad(disp_um):
    """Camera-plane beam displacement (um) -> mirror tilt angle (urad). A mirror
    tilt theta steers the beam 2*theta; over distance L it lands d = 2*theta*L
    off-center, so theta = d / (2 L); with d in um and L in m the units cancel."""
    return disp_um / (2.0 * DETECTOR_DISTANCE_M)


def _fit_line(x, y):
    """Least-squares line y = slope*x + intercept -> (slope, intercept, slope_err).
    slope_err is the 1-sigma standard error from the fit covariance, or NaN when
    there are too few points for one (needs > deg + 2 = 3)."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    if len(x) > 3:
        coeffs, cov = np.polyfit(x, y, 1, cov=True)
        return coeffs[0], coeffs[1], float(np.sqrt(cov[0, 0]))
    coeffs = np.polyfit(x, y, 1)
    return coeffs[0], coeffs[1], np.nan


def _validate_axes(held, driven):
    """Cross-talk needs the two axes to land on *different* camera columns, else
    their motions overlap on one column and can't be separated. Rejects a held
    axis equal to the driven axis and the X/Z pair (both -> camera Y). Raises
    ValueError before any hardware is opened."""
    for a in (held, driven):
        if a not in CAM_OF_PIEZO:
            raise ValueError(f"Unknown piezo axis {a!r}; use X, Y, or Z.")
    if held == driven:
        raise ValueError("HELD_AXIS and DRIVEN_AXIS must differ.")
    if CAM_OF_PIEZO[held] == CAM_OF_PIEZO[driven]:
        raise ValueError(
            f"{held} and {driven} both land on camera {CAM_OF_PIEZO[held]}, so "
            f"cross-talk can't be separated optically (X and Z share camera Y).")


def _tag_with_held_setpoint(csv_path, held_axis, held_v):
    """Rename a finished sub-scan's CSV (+sibling PNG) to embed the held axis and
    voltage as a `_<held>set<V>` token before the extension, matching the naming
    piezo_xy_cross_scan.py uses. Returns the new CSV path."""
    if not csv_path:
        return csv_path
    token = f"_{held_axis.lower()}set{held_v:g}"
    new_csv = csv_path[:-len(".csv")] + token + ".csv"
    os.rename(csv_path, new_csv)
    old_png = csv_path[:-len(".csv")] + ".png"
    if os.path.exists(old_png):
        os.rename(old_png, new_csv[:-len(".csv")] + ".png")
    return new_csv


def crosstalk_metrics(rows, held_axis, driven_axis, held_v):
    """Reduce one driven sweep (held axis parked at held_v) to cross-talk metrics.

    Reads the held axis's camera column as the cross-talk signal and the driven
    axis's column as the intended motion, both vs. the driven readback voltage.
    Returns a dict of mirror-angle (urad) fits: the driven sensitivity, the
    cross-talk slope (+error), their ratio (leaked fraction), and the held
    column's residual scatter and peak-to-peak (induced instability)."""
    held_cam = CAM_OF_PIEZO[held_axis]
    driven_cam = CAM_OF_PIEZO[driven_axis]
    v = np.array([r[_V_READBACK] for r in rows], float)
    held_urad = um_to_mirror_urad(np.array([r[_GFIT_UM[held_cam]] for r in rows], float))
    driven_urad = um_to_mirror_urad(np.array([r[_GFIT_UM[driven_cam]] for r in rows], float))

    driven_slope, _, driven_err = _fit_line(v, driven_urad)         # intended sensitivity
    xt_slope, xt_intercept, xt_err = _fit_line(v, held_urad)        # cross-talk leak
    residual = held_urad - (xt_slope * v + xt_intercept)
    ratio = xt_slope / driven_slope if driven_slope else np.nan
    return {
        "held_axis": held_axis,
        "driven_axis": driven_axis,
        "held_setpoint_V": held_v,
        "driven_sensitivity_urad_per_V": driven_slope,
        "driven_sensitivity_err": driven_err,
        "crosstalk_slope_urad_per_V": xt_slope,
        "crosstalk_slope_err": xt_err,
        "coupling_ratio": ratio,
        "held_residual_std_urad": float(np.std(residual)),
        "held_peak_to_peak_urad": float(held_urad.max() - held_urad.min()),
        "_v": v, "_held_urad": held_urad, "_driven_urad": driven_urad,   # for plotting
    }


def save_summary(metrics, out_dir):
    """Write crosstalk_summary.csv (one row per held setpoint); return its path."""
    cols = ["held_axis", "driven_axis", "held_setpoint_V",
            "driven_sensitivity_urad_per_V", "driven_sensitivity_err",
            "crosstalk_slope_urad_per_V", "crosstalk_slope_err", "coupling_ratio",
            "held_residual_std_urad", "held_peak_to_peak_urad"]
    path = os.path.join(out_dir, "crosstalk_summary.csv")
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(cols)
        for m in metrics:
            writer.writerow([m[c] for c in cols])
    print(f"Saved cross-talk summary to {path}")
    return path


def plot_crosstalk(metrics, held_axis, driven_axis, out_dir, run_stamp):
    """Two-panel figure sharing the driven-voltage x-axis: (left) the HELD axis's
    cross-talk transfer with its linear fit, (right) the DRIVEN axis's intended
    transfer, so leaked motion can be compared to the motion causing it. A single
    held setpoint is colored by sample order (reveals induced hysteresis); several
    setpoints get one color each. With >1 setpoint, a third panel plots the
    coupling ratio vs. held setpoint. Saves a PNG and returns its path."""
    single = len(metrics) == 1
    ncols = 2 if single or len(metrics) == 1 else 3
    fig, axes = plt.subplots(1, 3 if len(metrics) > 1 else 2,
                             figsize=(6 * (3 if len(metrics) > 1 else 2), 5))
    ax_xt, ax_drv = axes[0], axes[1]
    cmap = plt.get_cmap("viridis")

    for i, m in enumerate(metrics):
        v, held, driven = m["_v"], m["_held_urad"], m["_driven_urad"]
        label = f"{held_axis} @ {m['held_setpoint_V']:.0f} V"
        if single:
            order = np.arange(len(v))
            sc = ax_xt.scatter(v, held, c=order, cmap="viridis", s=16)
            cbar = fig.colorbar(sc, ax=ax_xt)
            cbar.set_label("Sample (time order)")
            ax_drv.scatter(v, driven, c=order, cmap="viridis", s=16)
        else:
            color = cmap(i / max(1, len(metrics) - 1))
            ax_xt.scatter(v, held, s=14, color=color, label=label)
            ax_drv.scatter(v, driven, s=14, color=color, label=label)
        # cross-talk fit line
        vv = np.linspace(v.min(), v.max(), 2)
        ax_xt.plot(vv, m["crosstalk_slope_urad_per_V"] * vv
                   + (held.mean() - m["crosstalk_slope_urad_per_V"] * v.mean()),
                   "-", color="tab:red" if single else cmap(i / max(1, len(metrics) - 1)),
                   linewidth=1.2, alpha=0.9)

    ax_xt.set_xlabel(f"Driven {driven_axis} voltage, readback (V)")
    ax_xt.set_ylabel(f"Held {held_axis} mirror angle (urad)")
    ax_xt.set_title(f"Cross-talk: {driven_axis} sweep -> {held_axis} motion")
    ax_drv.set_xlabel(f"Driven {driven_axis} voltage, readback (V)")
    ax_drv.set_ylabel(f"Driven {driven_axis} mirror angle (urad)")
    ax_drv.set_title(f"Intended: {driven_axis} sweep -> {driven_axis} motion")
    if not single:
        ax_xt.legend(fontsize=8)
        ax_drv.legend(fontsize=8)

    if len(metrics) > 1:
        ax_r = axes[2]
        sp = [m["held_setpoint_V"] for m in metrics]
        ratio = [100.0 * m["coupling_ratio"] for m in metrics]
        ax_r.plot(sp, ratio, "o-", color="tab:purple")
        ax_r.set_xlabel(f"Held {held_axis} setpoint (V)")
        ax_r.set_ylabel("Coupling ratio (% of driven motion)")
        ax_r.set_title("Cross-talk vs. held setpoint")

    fig.suptitle(f"Piezo cross-talk ({driven_axis} driven, {held_axis} held) "
                 f"[crosstalk_{run_stamp}]")
    fig.tight_layout()
    png_path = os.path.join(out_dir, f"crosstalk_{run_stamp}.png")
    fig.savefig(png_path, dpi=150)
    print(f"Saved cross-talk plot to {png_path}")
    return png_path


def main():
    held_axis = HELD_AXIS.upper()
    driven_axis = DRIVEN_AXIS.upper()
    _validate_axes(held_axis, driven_axis)

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

    v_sweep_max = min(V_SWEEP_MAX, v_limit)
    if v_sweep_max < V_SWEEP_MAX:
        print(f"Requested driven max {V_SWEEP_MAX}V exceeds device limit {v_limit}V, "
              f"clipping to {v_sweep_max}V.")

    run_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(OUTPUT_ROOT, f"crosstalk_{run_stamp}")
    os.makedirs(out_dir, exist_ok=True)
    print(f"Cross-talk run output: {out_dir}")

    setpoints = [s for s in HELD_SETPOINTS if V_SWEEP_MIN <= s <= v_limit]
    skipped = [s for s in HELD_SETPOINTS if s not in setpoints]
    if skipped:
        print(f"Skipping held setpoints outside [{V_SWEEP_MIN}, {v_limit}] V: {skipped}")
    print(f"{len(setpoints)} cross-talk sub-scans planned: hold {held_axis} at "
          f"{setpoints} V, sweep {driven_axis} {V_SWEEP_MIN:.1f}-{v_sweep_max:.1f} V each. "
          f"Camera latency dominates -- press Ctrl+C to stop early.")

    metrics = []
    try:
        for held_v in setpoints:
            _SET_AXIS[held_axis](piezo_hdl, float(held_v))
            if HELD_SETTLE_TIME:
                time.sleep(HELD_SETTLE_TIME)
            print(f"\n--- {held_axis} parked at {held_v:.1f} V; "
                  f"sweeping {driven_axis} to probe cross-talk ---")

            csv_path, rows = run_triangle_scan(
                piezo_hdl, driven_axis, bc1_vi, sensor, V_SWEEP_MIN, v_sweep_max,
                POINTS_PER_RAMP, N_CYCLES, SETTLE_TIME, SUBSCAN_BUFFER_TIME, out_dir,
                live=False,
            )
            _tag_with_held_setpoint(csv_path, held_axis, held_v)
            if rows:
                m = crosstalk_metrics(rows, held_axis, driven_axis, held_v)
                metrics.append(m)
                print(f"    driven sensitivity {m['driven_sensitivity_urad_per_V']:.3f} urad/V | "
                      f"cross-talk {m['crosstalk_slope_urad_per_V']:.4f} +/- "
                      f"{m['crosstalk_slope_err']:.4f} urad/V | "
                      f"coupling {100 * m['coupling_ratio']:.2f}% | "
                      f"held scatter {m['held_residual_std_urad']:.3f} urad "
                      f"(p2p {m['held_peak_to_peak_urad']:.3f})")

    except KeyboardInterrupt:
        print(f"\nCross-talk run interrupted -- {len(metrics)} sub-scans saved.")

    finally:
        for setter in _SET_AXIS.values():
            setter(piezo_hdl, 0.0)
        mdtClose(piezo_hdl)
        tlbc1.set_exposure_time(bc1_vi, prev_exposure_time)
        tlbc1.set_auto_exposure(bc1_vi, prev_auto_exposure)
        tlbc1.close(bc1_vi)
        print("Devices closed.")

    if metrics:
        save_summary(metrics, out_dir)
        plot_crosstalk(metrics, held_axis, driven_axis, out_dir, run_stamp)
        worst = max(metrics, key=lambda m: abs(m["coupling_ratio"]))
        print(f"\nCross-talk summary ({driven_axis} driven -> {held_axis} held): "
              f"coupling {100 * worst['coupling_ratio']:.2f}% of driven motion at worst "
              f"(held {held_axis}={worst['held_setpoint_V']:.0f} V); "
              f"induced held scatter up to "
              f"{max(m['held_residual_std_urad'] for m in metrics):.3f} urad.")
        plt.show(block=False)
        plt.pause(0.1)
    else:
        print("No cross-talk sub-scans collected.")


if __name__ == "__main__":
    main()
