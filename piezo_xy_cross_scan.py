"""
Cross-scan interface for the piezo steering mirror: how the Y-axis sensitivity
depends on where the X axis is parked.

Unlike piezo_master_scan.py (small sub-scans around several setpoints on each
axis), this holds the X piezo at each of a few DC setpoints spanning the full
range and, at each one, runs a single dense full-range triangle scan on the Y
axis. Each Y sub-scan produces its own CSV in the same format as
piezo_triangle_scan.py, all collected under scans/cross_<timestamp>/, with the
held X setpoint encoded in the filename as an `_xset<V>` token so the
accompanying notebook (y_sensitivity_vs_x.ipynb) can fit the Y sensitivity at
each sub-scan and plot Y sensitivity vs. X voltage.

X (held) lands on camera Y and Y (scanned) lands on camera X (camera rotated 90
deg), so parking X only DC-offsets the column we don't fit -- the Y motion stays
cleanly on camera X. The piezo controller and BC1 beam profiler are opened once
and reused across every sub-scan (auto-exposure disabled for the whole session,
restored at the end). Press Ctrl+C to stop early -- completed sub-scans remain
saved on disk.

Hardware:
  - Thorlabs MDT693B piezo controller, driven via ./MDT_COMMAND_LIB.py
  - Thorlabs BC1-series beam profiler, driven via ./tlbc1.py
"""

import datetime
import os
import time

from MDT_COMMAND_LIB import (
    mdtListDevices, mdtOpen, mdtClose, mdtGetLimtVoltage,
)

import tlbc1
from piezo_triangle_scan import Sensor, run_triangle_scan, _SET_AXIS

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
HELD_AXIS = "X"                 # axis parked at a DC setpoint for each sub-scan
SCAN_AXIS = "Y"                 # axis swept full-range at each held setpoint
# X setpoints spanning the full range (V). Convention: don't include V = 0.
X_SETPOINTS = [10.0, 30.0, 50.0, 70.0, 90.0, 110.0, 130.0]
# X_SETPOINTS = [30.0, 90.0]
V_SCAN_MIN = 5.0            # Y triangle low voltage (V); keep off 0 per convention
V_SCAN_MAX = 150.0         # Y triangle high voltage (V), clipped to device limit
POINTS_PER_RAMP = 40       # samples per up/down ramp -- dense so the Y fit is clean
N_CYCLES = 2               # triangle cycles per Y sub-scan (>=2 keeps hysteresis)
SETTLE_TIME = 0.02         # seconds to wait after setting Y voltage before reading
HELD_SETTLE_TIME = 1.0     # seconds to let X settle after parking it, before the Y scan
FIXED_EXPOSURE_MS = 0.5    # exposure time (ms) used for the whole session
SUBSCAN_BUFFER_TIME = 0.1
OUTPUT_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scans")


def _tag_with_held_setpoint(csv_path, held_axis, held_v):
    """Rename a finished sub-scan's CSV (and its sibling PNG) to embed the held
    axis + voltage as an `_xset<V>` token before the extension, returning the new
    CSV path. run_triangle_scan names files only by the *scanned* axis/range, so
    this is how the held X setpoint is recorded for the analysis notebook."""
    if not csv_path:
        return csv_path
    token = f"_{held_axis.lower()}set{held_v:g}"
    new_csv = csv_path[:-len(".csv")] + token + ".csv"
    os.rename(csv_path, new_csv)
    old_png = csv_path[:-len(".csv")] + ".png"
    if os.path.exists(old_png):
        os.rename(old_png, new_csv[:-len(".csv")] + ".png")
    return new_csv


def main():
    held_axis = HELD_AXIS.upper()
    scan_axis = SCAN_AXIS.upper()
    if held_axis == scan_axis:
        print("HELD_AXIS and SCAN_AXIS must differ.")
        return

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

    # clip the Y sweep to the device limit
    v_scan_max = min(V_SCAN_MAX, v_limit)
    if v_scan_max < V_SCAN_MAX:
        print(f"Requested Y max {V_SCAN_MAX}V exceeds device limit {v_limit}V, clipping to {v_scan_max}V.")

    # one output folder per cross run
    run_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    cross_dir = os.path.join(OUTPUT_ROOT, f"cross_{run_stamp}")
    os.makedirs(cross_dir, exist_ok=True)
    print(f"Cross run output: {cross_dir}")

    # only park X at setpoints that fit under the device limit
    setpoints = [x for x in X_SETPOINTS if x <= v_limit]
    skipped = [x for x in X_SETPOINTS if x > v_limit]
    if skipped:
        print(f"Skipping {held_axis} setpoints above limit {v_limit}V: {skipped}")
    print(f"{len(setpoints)} Y sub-scans planned "
          f"(one full-range {scan_axis} scan at each of {held_axis}={setpoints} V). "
          f"Camera frame latency dominates -- press Ctrl+C to stop early.")

    completed = []
    try:
        for x_set in setpoints:
            # park the held axis at its DC setpoint and let it settle
            _SET_AXIS[held_axis](piezo_hdl, float(x_set))
            if HELD_SETTLE_TIME:
                time.sleep(HELD_SETTLE_TIME)
            print(f"\n--- {held_axis} parked at {x_set:.1f} V; "
                  f"scanning {scan_axis} {V_SCAN_MIN:.1f}-{v_scan_max:.1f} V ---")

            csv_path, _ = run_triangle_scan(
                piezo_hdl, scan_axis, bc1_vi, sensor, V_SCAN_MIN, v_scan_max,
                POINTS_PER_RAMP, N_CYCLES, SETTLE_TIME, SUBSCAN_BUFFER_TIME, cross_dir,
                live=False,
            )
            csv_path = _tag_with_held_setpoint(csv_path, held_axis, x_set)
            if csv_path:
                completed.append(csv_path)

    except KeyboardInterrupt:
        print(f"\nCross run interrupted -- {len(completed)} sub-scans saved.")

    finally:
        # return every axis to 0V, restore exposure, release both devices
        for setter in _SET_AXIS.values():
            setter(piezo_hdl, 0.0)
        mdtClose(piezo_hdl)
        tlbc1.set_exposure_time(bc1_vi, prev_exposure_time)
        tlbc1.set_auto_exposure(bc1_vi, prev_auto_exposure)
        tlbc1.close(bc1_vi)
        print("Devices closed.")

    print(f"\nCross run complete: {len(completed)} sub-scans in {cross_dir}")
    for path in completed:
        print("  ", os.path.basename(path))


if __name__ == "__main__":
    main()
