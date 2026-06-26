"""
Drive the MDT693B piezo controller (steering mirror) through several cycles
of a triangle wave on one axis, while tracking the beam position on the
Thorlabs BC1 beam profiler in real time.

Saves a CSV with the piezo setpoint/readback voltage and the beam centroid
and peak position (in um, relative to the sensor center) for every sample,
and shows a live plot while the scan runs. Press Ctrl+C at any time to stop
early -- the data collected so far is still saved and plotted.

Hardware:
  - Thorlabs MDT693B piezo controller, driven via ./MDT_COMMAND_LIB.py
  - Thorlabs BC1-series beam profiler, driven via ./tlbc1.py

Performance note: profiling shows TLBC1_get_scan_data() takes ~2.5-4s per
call no matter which getter functions are used (the official Thorlabs C
sample exhibits the same per-call latency) -- the bottleneck is the camera
driver's frame-acquisition round trip, not Python/plotting overhead. The
single lever that meaningfully helps is disabling auto-exposure, which cuts
~35-40% off each call (auto-exposure re-evaluates exposure every frame).
This script does that automatically for the duration of the scan and
restores the previous auto-exposure setting afterwards.
"""

import csv
import datetime
import os
import time

import matplotlib.pyplot as plt
import numpy as np

from MDT_COMMAND_LIB import (
    mdtListDevices, mdtOpen, mdtClose,
    mdtGetLimtVoltage, mdtSetXAxisVoltage, mdtGetXAxisVoltage,
    mdtSetYAxisVoltage, mdtGetYAxisVoltage,
)

import tlbc1

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
AXIS = "Y"              # which piezo axis to scan: "X" or "Y"
V_MIN = 0.0             # triangle wave low voltage (V)
V_MAX = 150.0            # triangle wave high voltage (V), clipped to device limit
POINTS_PER_RAMP = 40    # samples per up/down ramp
N_CYCLES = 3            # number of full triangle cycles
SETTLE_TIME = 0.02      # seconds to wait after setting voltage before reading

FIXED_EXPOSURE_MS = 0.5  # exposure time (ms) used during the scan; auto-exposure
                          # is disabled for the scan and restored afterwards
PLOT_UPDATE_EVERY = 10     # redraw the live plot every N samples instead of every sample

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))


def triangle_wave(v_min, v_max, points_per_ramp, n_cycles):
    up = np.linspace(v_min, v_max, points_per_ramp, endpoint=False)
    down = np.linspace(v_max, v_min, points_per_ramp, endpoint=False)
    one_cycle = np.concatenate([up, down])
    return np.tile(one_cycle, n_cycles)


def save_results(rows, fig):
    run_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    csv_path = os.path.join(OUTPUT_DIR, f"piezo_scan_{run_stamp}.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "sample", "timestamp", "voltage_setpoint_V", "voltage_readback_V",
            "centroid_x_um", "centroid_y_um", "peak_x_um", "peak_y_um",
            "peak_intensity_counts", "total_power_mW",
        ])
        writer.writerows(rows)
    print(f"Saved {len(rows)} samples to {csv_path}")

    plot_path = os.path.join(OUTPUT_DIR, f"piezo_scan_{run_stamp}.png")
    fig.savefig(plot_path, dpi=150)
    print(f"Saved plot to {plot_path}")


def main():
    axis = AXIS.upper()
    set_axis_voltage = {"X": mdtSetXAxisVoltage, "Y": mdtSetYAxisVoltage}[axis]
    get_axis_voltage = {"X": mdtGetXAxisVoltage, "Y": mdtGetYAxisVoltage}[axis]

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

    voltages = triangle_wave(V_MIN, v_max, POINTS_PER_RAMP, N_CYCLES)

    # --- connect to the beam profiler ---
    try:
        bc1_vi, bc1_info = tlbc1.open_first_device()
    except RuntimeError as e:
        print(e)
        mdtClose(piezo_hdl)
        return
    print("Connected to beam profiler", bc1_info["model_name"], bc1_info["serial_number"])

    pixel_count_x, pixel_count_y, pixel_pitch_h, pixel_pitch_v = tlbc1.get_sensor_information(bc1_vi)
    center_x = pixel_count_x / 2.0
    center_y = pixel_count_y / 2.0

    # disabling auto-exposure cuts ~35-40% off each get_scan_data() call;
    # remember the previous settings so they can be restored afterwards
    prev_auto_exposure = tlbc1.get_auto_exposure(bc1_vi)
    prev_exposure_time = tlbc1.get_exposure_time(bc1_vi)
    tlbc1.set_auto_exposure(bc1_vi, False)
    tlbc1.set_exposure_time(bc1_vi, FIXED_EXPOSURE_MS)

    print(f"~{len(voltages)} samples queued. Camera frame latency dominates scan "
          f"time (seconds/sample) -- press Ctrl+C any time to stop early and save.")

    # --- live plot setup ---
    plt.ion()
    fig, (ax_time, ax_xy) = plt.subplots(1, 2, figsize=(11, 5))

    ax_time.set_xlabel("Sample")
    ax_time.set_ylabel("Voltage (V)", color="tab:blue")
    ax_time.tick_params(axis="y", labelcolor="tab:blue")
    line_voltage, = ax_time.plot([], [], "-", color="tab:blue", label="Piezo voltage")

    ax_pos = ax_time.twinx()
    ax_pos.set_ylabel("Centroid displacement (um)", color="tab:red")
    ax_pos.tick_params(axis="y", labelcolor="tab:red")
    line_cx, = ax_pos.plot([], [], "-", color="tab:red", label="Centroid X")
    line_cy, = ax_pos.plot([], [], "-", color="tab:orange", label="Centroid Y")

    ax_xy.set_xlabel("Centroid X (um)")
    ax_xy.set_ylabel("Centroid Y (um)")
    ax_xy.set_title("Beam position trajectory")
    scatter_xy = ax_xy.scatter([], [], c=[], cmap="viridis", s=15)

    fig.suptitle(f"Piezo {axis}-axis triangle scan vs beam position")
    fig.tight_layout()

    def redraw(rows, cx_hist, cy_hist):
        xs = list(range(len(rows)))
        line_voltage.set_data(xs, [r[2] for r in rows])
        line_cx.set_data(xs, cx_hist)
        line_cy.set_data(xs, cy_hist)
        ax_time.relim(); ax_time.autoscale_view()
        ax_pos.relim(); ax_pos.autoscale_view()

        scatter_xy.set_offsets(np.column_stack([cx_hist, cy_hist]))
        scatter_xy.set_array(np.array(xs))
        ax_xy.relim(); ax_xy.autoscale_view()

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    # --- scan loop ---
    rows = []
    cx_hist, cy_hist = [], []
    interrupted = False

    try:
        for i, v_set in enumerate(voltages):
            set_axis_voltage(piezo_hdl, float(v_set))
            if SETTLE_TIME:
                time.sleep(SETTLE_TIME)

            v_readback = [0.0]
            get_axis_voltage(piezo_hdl, v_readback)

            scan = tlbc1.get_scan_data(bc1_vi)

            centroid_x_um = (scan.centroidPositionX - center_x) * pixel_pitch_h
            centroid_y_um = (scan.centroidPositionY - center_y) * pixel_pitch_v
            peak_x_um = (scan.profilePeakPosX - center_x) * pixel_pitch_h
            peak_y_um = (scan.profilePeakPosY - center_y) * pixel_pitch_v
            total_power_mw = 10.0 ** (scan.totalPower / 10.0)

            timestamp = datetime.datetime.now()
            rows.append([
                i, timestamp.isoformat(), v_set, v_readback[0],
                centroid_x_um, centroid_y_um, peak_x_um, peak_y_um,
                scan.peakIntensity, total_power_mw,
            ])

            cx_hist.append(centroid_x_um)
            cy_hist.append(centroid_y_um)

            print(f"[{timestamp:%H:%M:%S.%f}] {i + 1}/{len(voltages)} "
                  f"V_set={v_set:6.2f}V V_read={v_readback[0]:6.2f}V "
                  f"Centroid=({centroid_x_um:7.2f}, {centroid_y_um:7.2f}) um")

            if i % PLOT_UPDATE_EVERY == 0 or i == len(voltages) - 1:
                redraw(rows, cx_hist, cy_hist)

    except KeyboardInterrupt:
        interrupted = True
        print(f"\nInterrupted by user after {len(rows)} samples -- saving collected data.")

    finally:
        # return piezo to 0V, restore exposure settings, release both devices
        set_axis_voltage(piezo_hdl, 0.0)
        mdtClose(piezo_hdl)
        tlbc1.set_exposure_time(bc1_vi, prev_exposure_time)
        tlbc1.set_auto_exposure(bc1_vi, prev_auto_exposure)
        tlbc1.close(bc1_vi)
        print("Devices closed.")

    if not rows:
        return

    redraw(rows, cx_hist, cy_hist)
    save_results(rows, fig)

    plt.ioff()
    if not interrupted:
        plt.show()
    else:
        plt.show(block=False)
        plt.pause(0.1)


if __name__ == "__main__":
    main()
