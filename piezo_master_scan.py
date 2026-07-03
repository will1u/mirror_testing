"""
Master testing interface for the piezo steering mirror.

Characterizes the mirror at several operating setpoints by running a small
triangle sub-scan around each setpoint, on each selected piezo axis. Every
sub-scan produces its own CSV in the same format as piezo_triangle_scan.py,
all collected under scans/master_<timestamp>/ so a whole session can be
analyzed together (see the "Master run" section of analysis.ipynb).

The piezo controller and BC1 beam profiler are opened once and reused across
every sub-scan (auto-exposure is disabled for the whole session and restored at
the end), which avoids repeated multi-second device open/close overhead. Press
Ctrl+C to stop early -- sub-scans already completed remain saved on disk.

Hardware:
  - Thorlabs MDT693B piezo controller, driven via ./MDT_COMMAND_LIB.py
  - Thorlabs BC1-series beam profiler, driven via ./tlbc1.py
"""

import datetime
import os

from MDT_COMMAND_LIB import (
    mdtListDevices, mdtOpen, mdtClose, mdtGetLimtVoltage,
    mdtSetXAxisVoltage, mdtSetYAxisVoltage,
)

import tlbc1
from piezo_triangle_scan import Sensor, run_triangle_scan

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
# AXES = ["X", "Y"]                               # piezo axes to characterize
AXES = ["Z"]
# CENTER_SETPOINTS = [20.0, 40.0, 60.0, 80.0, 100.0, 120.0]  # V, center of each sub-scan
CENTER_SETPOINTS = [20.0, 40.0]
SUB_SPAN = 5.0            # +/- V swept around each center setpoint
POINTS_PER_RAMP = 5         # samples per up/down ramp of each sub-scan
N_CYCLES = 1        # triangle cycles per sub-scan (needed for hysteresis)
SETTLE_TIME = 0.02       # seconds to wait after setting voltage before reading
FIXED_EXPOSURE_MS = 0.5   # exposure time (ms) used for the whole session
SUBSCAN_BUFFER_TIME = 0.1
OUTPUT_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scans")


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

    # one output folder per master run
    run_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    master_dir = os.path.join(OUTPUT_ROOT, f"master_{run_stamp}")
    os.makedirs(master_dir, exist_ok=True)
    print(f"Master run output: {master_dir}")

    plan = [(axis, center) for axis in AXES for center in CENTER_SETPOINTS]
    print(f"{len(plan)} sub-scans planned "
          f"({len(AXES)} axes x {len(CENTER_SETPOINTS)} setpoints). "
          f"Camera frame latency dominates -- press Ctrl+C to stop early.")

    completed = []
    try:
        axis_prev = "X"
        axis_change = False

        for axis, center in plan:
            if axis != axis_prev:
                axis_change = True
            
            if axis_change:
                mdtSetXAxisVoltage(piezo_hdl, 0.0)
                axis_change = False
            v_min = max(0.0, center - SUB_SPAN)
            v_max = min(v_limit, center + SUB_SPAN)
            if v_max <= v_min:
                print(f"Skipping {axis} @ {center} V: range {v_min}-{v_max} V "
                      f"collapses against device limit {v_limit} V.")
                continue
            csv_path, _ = run_triangle_scan(
                piezo_hdl, axis, bc1_vi, sensor, v_min, v_max,
                POINTS_PER_RAMP, N_CYCLES, SETTLE_TIME, SUBSCAN_BUFFER_TIME, master_dir, live=False,
            )
            if csv_path:
                completed.append(csv_path)
            
            axis_prev = axis

    except KeyboardInterrupt:
        print(f"\nMaster run interrupted -- {len(completed)} sub-scans saved.")

    finally:
        # return both axes to 0V, restore exposure, release both devices
        mdtSetXAxisVoltage(piezo_hdl, 0.0)
        mdtSetYAxisVoltage(piezo_hdl, 0.0)
        mdtClose(piezo_hdl)
        tlbc1.set_exposure_time(bc1_vi, prev_exposure_time)
        tlbc1.set_auto_exposure(bc1_vi, prev_auto_exposure)
        tlbc1.close(bc1_vi)
        print("Devices closed.")

    print(f"\nMaster run complete: {len(completed)} sub-scans in {master_dir}")
    for path in completed:
        print("  ", os.path.basename(path))


if __name__ == "__main__":
    main()
