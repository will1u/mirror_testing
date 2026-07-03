"""
Drive the MDT693B piezo controller (steering mirror) through several cycles
of a triangle wave on one axis, while tracking the beam position on the
Thorlabs BC1 beam profiler in real time.

Saves a CSV with the piezo setpoint/readback voltage and the beam centroid
and Gaussian-fit center position (um, relative to sensor center) for every
sample, and shows a live plot while the scan runs. Press Ctrl+C at any time
to stop early -- the data collected so far is still saved and plotted.

The scan loop is factored into `run_triangle_scan()` so it can be reused for a
single scan (this script's `main()`) or driven many times around different
setpoints by the master interface (`piezo_master_scan.py`) while sharing one
open piezo + camera session.

Hardware:
  - Thorlabs MDT693B piezo controller, driven via ./MDT_COMMAND_LIB.py
  - Thorlabs BC1-series beam profiler, driven via ./tlbc1.py

Gaussian fit note: TLBC1_get_scan_data() already computes a Gaussian fit to
the beam on-camera, exposed in the TLBC1_Calculations struct as
gaussianFitCentroidPositionX/Y (pixel units). This script logs that fitted
center in place of the old profilePeakPosX/Y (raw brightest-pixel position),
since the fit center is far less sensitive to single-pixel noise/saturation.

Camera orientation note: the camera is mounted rotated 90 deg, so the camera's
X axis is the physical/piezo Y axis and vice versa. The raw camera columns are
logged unchanged; only the plot *labels* are put in the piezo frame (camera X
data is labelled "Y", camera Y data "X"), so the plots show the effect of the
driven piezo axis rather than which camera axis it lands on.

Units assumption: gaussianFitCentroidPositionX/Y is treated as a pixel-index
float, same convention as centroidPositionX/Y elsewhere in this struct, so
it's converted to physical units the same way: (value - sensor_center) *
pixel_pitch. If your TLBC1 header defines this field as already-physical
units, drop the pixel_pitch multiply below -- you'd notice the fitted center
sitting off from the plain centroid by roughly a pixel_pitch.

Performance note: profiling shows TLBC1_get_scan_data() takes ~2.5-4s per
call no matter which getter functions are used (the official Thorlabs C
sample exhibits the same per-call latency) -- the bottleneck is the camera
driver's frame-acquisition round trip, not Python/plotting/fit overhead.
The single lever that meaningfully helps is disabling auto-exposure, which
cuts ~35-40% off each call (auto-exposure re-evaluates exposure every
frame). This script does that automatically for the duration of the scan
and restores the previous auto-exposure setting afterwards.
"""

import csv
import datetime
import os
import time
from collections import namedtuple

import matplotlib.pyplot as plt
import numpy as np

from MDT_COMMAND_LIB import (
    mdtListDevices, mdtOpen, mdtClose,
    mdtGetLimtVoltage, mdtSetXAxisVoltage, mdtGetXAxisVoltage,
    mdtSetYAxisVoltage, mdtGetYAxisVoltage, mdtGetZAxisVoltage,
    mdtSetZAxisVoltage
)

import tlbc1

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
AXIS = "X"              # which piezo axis to scan: "X" or "Y"
V_MIN = 40.0             # triangle wave low voltage (V)
V_MAX = 50.0            # triangle wave high voltage (V), clipped to device limit
POINTS_PER_RAMP = 10    # samples per up/down ramp
N_CYCLES = 3            # number of full triangle cycles
SETTLE_TIME = 0.02      # seconds to wait after setting voltage before reading
SUBSCAN_BUFFER_TIME = 3
FIXED_EXPOSURE_MS = 0.5  # exposure time (ms) used during the scan; auto-exposure
                          # is disabled for the scan and restored afterwards
PLOT_UPDATE_EVERY = 10     # redraw the live plot every N samples instead of every sample

# new scans are written here (existing root-level CSVs are left untouched)
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scans")

# bundle of sensor geometry needed to convert pixel coordinates to microns
Sensor = namedtuple("Sensor", "center_x center_y pitch_h pitch_v")

# piezo axis -> setter/getter for that axis' voltage
_SET_AXIS = {"X": mdtSetXAxisVoltage, "Y": mdtSetYAxisVoltage, "Z": mdtSetZAxisVoltage}
_GET_AXIS = {"X": mdtGetXAxisVoltage, "Y": mdtGetYAxisVoltage, "Z": mdtGetZAxisVoltage}


def triangle_wave(v_min, v_max, points_per_ramp, n_cycles):
    up = np.linspace(v_min, v_max, points_per_ramp, endpoint=False)
    down = np.linspace(v_max, v_min, points_per_ramp, endpoint=False)
    one_cycle = np.concatenate([up, down])
    return np.tile(one_cycle, n_cycles)


def save_results(rows, fig, output_dir, axis, v_min, v_max, points_per_ramp, n_cycles):
    """Write one sub-scan's CSV (+PNG) to output_dir; return the CSV path.

    `points_per_ramp` and `n_cycles` describe the triangle-wave structure and are
    written as constant columns on every row so the analysis notebook can pick
    out individual ramps/cycles (e.g. omit the first up-ramp) without having to
    re-infer the ramp length from the voltage-setpoint pattern.
    """
    os.makedirs(output_dir, exist_ok=True)
    run_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    axis = axis.upper()
    base = f"piezo_scan_{run_stamp}_min{v_min}_max{v_max}_axis{axis}"

    csv_path = os.path.join(output_dir, base + ".csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "sample", "timestamp", "voltage_setpoint_V", "voltage_readback_V",
            "centroid_x_um", "centroid_y_um", "gaussfit_center_x_um", "gaussfit_center_y_um",
            "gaussfit_rating_x", "gaussfit_rating_y",
            "peak_intensity_counts", "total_power_mW",
            "points_per_ramp", "n_cycles",
        ])
        writer.writerows(row + [points_per_ramp, n_cycles] for row in rows)
    print(f"Saved {len(rows)} samples to {csv_path}")

    plot_path = os.path.join(output_dir, base + ".png")
    fig.savefig(plot_path, dpi=150)
    print(f"Saved plot to {plot_path}")
    return csv_path


def run_triangle_scan(piezo_hdl, axis, bc1_vi, sensor, v_min, v_max,
                      points_per_ramp, n_cycles, settle_time, subscan_buffer_time, output_dir,
                      live=True):
    """Run one triangle sweep on `axis` and save its CSV (+PNG) to output_dir.

    Assumes the piezo controller and beam profiler are already open and the
    camera exposure is already configured (so many sub-scans can share one
    session). `v_min`/`v_max` should already be clipped to the device limit by
    the caller. `sensor` is a Sensor namedtuple. When `live` is True the figure
    is updated incrementally as the scan runs; when False it is rendered once at
    the end purely to save the PNG and then closed. Returns (csv_path, rows).
    Re-raises KeyboardInterrupt after saving the partial sub-scan so a caller
    driving many scans can stop the whole sweep.
    """
    axis = axis.upper()
    set_axis_voltage = _SET_AXIS[axis]
    get_axis_voltage = _GET_AXIS[axis]

    voltages = triangle_wave(v_min, v_max, points_per_ramp, n_cycles)
    print(f"\n=== Piezo {axis} sub-scan {v_min:.1f}-{v_max:.1f} V "
          f"({len(voltages)} samples) ===")

    # --- figure: hysteresis loop for the driven axis (drives live view + PNG) ---
    if live:
        plt.ion()
    fig, ax = plt.subplots(figsize=(7, 6))

    # camera rotated 90 deg: the driven piezo axis's motion lands on the *other*
    # camera column (piezo X -> camera Y, piezo Y -> camera X). Plot that column
    # so the hysteresis loop shows the effect of the driven axis, labelled in the
    # piezo frame. (gfx_hist = camera X data, gfy_hist = camera Y data.)
    driven_is_x = (axis == "X")

    ax.set_xlabel("Piezo voltage, readback (V)")
    ax.set_ylabel(f"Piezo {axis} displacement (um)")
    ax.set_title(f"Piezo {axis} hysteresis ({v_min:.1f}-{v_max:.1f} V)")
    line_path, = ax.plot([], [], "-", color="gray", alpha=0.3, linewidth=0.8)
    scatter = ax.scatter([], [], c=[], cmap="viridis", s=20, vmin=0, vmax=1)
    cbar = fig.colorbar(scatter, ax=ax)
    cbar.set_label("Sample (time order)")
    fig.tight_layout()

    def redraw(rows, gfx_hist, gfy_hist):
        xs = list(range(len(rows)))
        volts = [r[3] for r in rows]                    # readback voltage
        disp = gfy_hist if driven_is_x else gfx_hist    # driven-axis displacement
        line_path.set_data(volts, disp)
        offsets = np.column_stack([volts, disp]) if rows else np.empty((0, 2))
        scatter.set_offsets(offsets)
        scatter.set_array(np.array(xs))
        if xs:
            scatter.set_clim(0, max(xs))
        # Axes.relim() ignores scatter collections, so set limits explicitly from
        # the data -- otherwise the plot renders blank / clipped in the PNG.
        if rows:
            xlo, xhi = min(volts), max(volts)
            ylo, yhi = min(disp), max(disp)
            xpad = (xhi - xlo) * 0.05 or 1.0   # fallback pad when points coincide
            ypad = (yhi - ylo) * 0.05 or 1.0
            ax.set_xlim(xlo - xpad, xhi + xpad)
            ax.set_ylim(ylo - ypad, yhi + ypad)

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    # --- scan loop ---
    rows = []
    gfx_hist, gfy_hist = [], []
    interrupted = False

    try:
        for i, v_set in enumerate(voltages):
            set_axis_voltage(piezo_hdl, float(v_set))
            if settle_time:
                time.sleep(settle_time)

            v_readback = [0.0]

            scan = tlbc1.get_scan_data(bc1_vi)
            get_axis_voltage(piezo_hdl, v_readback)
            centroid_x_um = (scan.centroidPositionX - sensor.center_x) * sensor.pitch_h
            centroid_y_um = (scan.centroidPositionY - sensor.center_y) * sensor.pitch_v
            gfit_center_x_um = (scan.gaussianFitCentroidPositionX - sensor.center_x) * sensor.pitch_h
            gfit_center_y_um = (scan.gaussianFitCentroidPositionY - sensor.center_y) * sensor.pitch_v
            total_power_mw = 10.0 ** (scan.totalPower / 10.0)

            timestamp = datetime.datetime.now()
            rows.append([
                i, timestamp.isoformat(), v_set, v_readback[0],
                centroid_x_um, centroid_y_um, gfit_center_x_um, gfit_center_y_um,
                scan.gaussianFitRatingX, scan.gaussianFitRatingY,
                scan.peakIntensity, total_power_mw,
            ])

            gfx_hist.append(gfit_center_x_um)
            gfy_hist.append(gfit_center_y_um)

            print(f"[{timestamp:%H:%M:%S.%f}] {i + 1}/{len(voltages)} "
                  f"V_set={v_set:6.2f}V V_read={v_readback[0]:6.2f}V "
                  f"Centroid=({centroid_x_um:7.2f}, {centroid_y_um:7.2f}) um "
                  f"GaussFitCenter=({gfit_center_x_um:7.2f}, {gfit_center_y_um:7.2f}) um "
                  f"rating=({scan.gaussianFitRatingX:.3f}, {scan.gaussianFitRatingY:.3f})")

            if live and (i % PLOT_UPDATE_EVERY == 0 or i == len(voltages) - 1):
                redraw(rows, gfx_hist, gfy_hist)

    except KeyboardInterrupt:
        interrupted = True
        print(f"\nInterrupted after {len(rows)} samples -- saving this sub-scan.")

    csv_path = None
    if rows:
        redraw(rows, gfx_hist, gfy_hist)
        csv_path = save_results(rows, fig, output_dir, axis, v_min, v_max,
                                points_per_ramp, n_cycles)

    if not live:
        plt.close(fig)

    if interrupted:
        raise KeyboardInterrupt
    
    if subscan_buffer_time:
        time.sleep(subscan_buffer_time)
    return csv_path, rows


def main():
    axis = AXIS.upper()

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
    v_max = min(V_MAX, limit_voltage[0])
    if v_max < V_MAX:
        print(f"Requested max {V_MAX}V exceeds device limit {limit_voltage[0]}V, clipping to {v_max}V.")

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

    # disabling auto-exposure cuts ~35-40% off each get_scan_data() call;
    # remember the previous settings so they can be restored afterwards
    prev_auto_exposure = tlbc1.get_auto_exposure(bc1_vi)
    prev_exposure_time = tlbc1.get_exposure_time(bc1_vi)
    tlbc1.set_auto_exposure(bc1_vi, False)
    tlbc1.set_exposure_time(bc1_vi, FIXED_EXPOSURE_MS)

    print("Camera frame latency dominates scan time (seconds/sample) -- press "
          "Ctrl+C any time to stop early and save.")

    try:
        run_triangle_scan(piezo_hdl, axis, bc1_vi, sensor, V_MIN, v_max,
                          POINTS_PER_RAMP, N_CYCLES, SETTLE_TIME, SUBSCAN_BUFFER_TIME, OUTPUT_DIR,
                          live=True)
    except KeyboardInterrupt:
        print("Scan interrupted by user.")

    finally:
        # return piezo to 0V, restore exposure settings, release both devices
        _SET_AXIS[axis](piezo_hdl, 0.0)
        mdtClose(piezo_hdl)
        tlbc1.set_exposure_time(bc1_vi, prev_exposure_time)
        tlbc1.set_auto_exposure(bc1_vi, prev_auto_exposure)
        tlbc1.close(bc1_vi)
        print("Devices closed.")

    plt.ioff()
    plt.show(block=False)
    plt.pause(0.1)


if __name__ == "__main__":
    main()
