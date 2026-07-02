"""
Beam-pointing stability test: hold the piezo steering mirror at a fixed
voltage and sample the beam's peak (and centroid) position on the Thorlabs
BC1 beam profiler over a long duration (hours-scale), tracking drift.

Data is written to the CSV incrementally (one flush per sample), so a power
loss or hard kill mid-run still leaves everything collected up to that
point on disk. Press Ctrl+C at any time to stop early -- the run still ends
with the usual final plot.

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
    mdtSetXAxisVoltage, mdtGetXAxisVoltage,
    mdtSetYAxisVoltage, mdtGetYAxisVoltage,
)

import tlbc1
import bc1_stability_common as bc1c

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
SET_PIEZO = True        # hold the piezo at a fixed voltage for the duration of the test;
                         # set False to just monitor the beam without touching the piezo
FIXED_VOLTAGE_X = 0.0   # V, only used if SET_PIEZO
FIXED_VOLTAGE_Y = 40.0   # V, only used if SET_PIEZO

DURATION_HOURS = 4.0    # total run time; set to None to run until Ctrl+C
SAMPLE_INTERVAL_S = 5.0 # target time between samples -- camera latency (~2-4s/call) may
                         # dominate and make the actual cadence slower than this on its own

FIXED_EXPOSURE_MS = 0.5  # auto-exposure is disabled for the whole run: it would actively
                          # compensate for intensity drift, masking the very thing we're
                          # trying to measure
WAVELENGTH_NM = 635      # set to your laser's wavelength (nm) for accurate power readings;
                           # None leaves the camera's current calibration untouched

PLOT_UPDATE_EVERY = 10   # redraw (and snapshot-save) the live plot every N samples

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))


def main():
    piezo_hdl = None
    if SET_PIEZO:
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
        vx = min(FIXED_VOLTAGE_X, limit_voltage[0])
        vy = min(FIXED_VOLTAGE_Y, limit_voltage[0])
        mdtSetXAxisVoltage(piezo_hdl, float(vx))
        mdtSetYAxisVoltage(piezo_hdl, float(vy))
        print(f"Piezo held at X={vx:.2f}V, Y={vy:.2f}V")

    try:
        session = bc1c.connect(fixed_exposure_ms=FIXED_EXPOSURE_MS, wavelength_nm=WAVELENGTH_NM)
    except RuntimeError as e:
        print(e)
        if piezo_hdl is not None:
            mdtClose(piezo_hdl)
        return
    print("Connected to beam profiler", session.info["model_name"], session.info["serial_number"])

    duration_s = DURATION_HOURS * 3600.0 if DURATION_HOURS is not None else None
    if duration_s is not None:
        print(f"Running for {DURATION_HOURS:.2f} hours, ~{SAMPLE_INTERVAL_S:.1f}s/sample "
              f"(camera latency may dominate). Press Ctrl+C to stop early.")
    else:
        print("Running until Ctrl+C is pressed.")

    run_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(OUTPUT_DIR, f"stability_test_{run_stamp}.csv")
    plot_path = os.path.join(OUTPUT_DIR, f"stability_test_{run_stamp}.png")

    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow([
        "sample", "timestamp", "elapsed_s",
        "voltage_x_readback_V", "voltage_y_readback_V",
        "peak_x_um", "peak_y_um", "centroid_x_um", "centroid_y_um",
        "peak_intensity_counts", "total_power_mW",
    ])
    csv_file.flush()

    # --- live plot setup ---
    plt.ion()
    fig, ax = plt.subplots(figsize=(9, 5))
    line_px, = ax.plot([], [], "-", color="tab:red", label="Peak X")
    line_py, = ax.plot([], [], "-", color="tab:orange", label="Peak Y")
    ax.set_xlabel("Elapsed time (s)")
    ax.set_ylabel("Peak position displacement (um)")
    ax.set_title("Beam pointing stability")
    ax.legend()
    fig.tight_layout()

    def redraw(t_hist, px_hist, py_hist):
        line_px.set_data(t_hist, px_hist)
        line_py.set_data(t_hist, py_hist)
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    start_time = time.time()
    t_hist, px_hist, py_hist = [], [], []
    sample = 0
    interrupted = False

    try:
        while True:
            elapsed = time.time() - start_time
            if duration_s is not None and elapsed >= duration_s:
                break

            iter_start = time.perf_counter()

            scan = tlbc1.get_scan_data(session.vi)

            bc1c.check_saturation(scan)

            peak_x_um = (scan.profilePeakPosX - session.center_x) * session.pixel_pitch_h
            peak_y_um = (scan.profilePeakPosY - session.center_y) * session.pixel_pitch_v
            centroid_x_um = (scan.centroidPositionX - session.center_x) * session.pixel_pitch_h
            centroid_y_um = (scan.centroidPositionY - session.center_y) * session.pixel_pitch_v
            total_power_mw = 10.0 ** (scan.totalPower / 10.0)

            if SET_PIEZO:
                vx_read = [0.0]; vy_read = [0.0]
                mdtGetXAxisVoltage(piezo_hdl, vx_read)
                mdtGetYAxisVoltage(piezo_hdl, vy_read)
                vx_read, vy_read = vx_read[0], vy_read[0]
            else:
                vx_read, vy_read = None, None

            timestamp = datetime.datetime.now()
            writer.writerow([
                sample, timestamp.isoformat(), f"{elapsed:.3f}",
                vx_read, vy_read,
                peak_x_um, peak_y_um, centroid_x_um, centroid_y_um,
                scan.peakIntensity, total_power_mw,
            ])
            csv_file.flush()

            t_hist.append(elapsed)
            px_hist.append(peak_x_um)
            py_hist.append(peak_y_um)

            print(f"[{timestamp:%H:%M:%S}] t={elapsed/3600:6.2f}h sample={sample} "
                  f"Peak=({peak_x_um:7.2f}, {peak_y_um:7.2f}) um  Power={total_power_mw:.4f} mW")

            if sample % PLOT_UPDATE_EVERY == 0:
                redraw(t_hist, px_hist, py_hist)
                fig.savefig(plot_path, dpi=150)

            sample += 1

            remaining = SAMPLE_INTERVAL_S - (time.perf_counter() - iter_start)
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        interrupted = True
        print(f"\nInterrupted by user after {sample} samples -- data already saved to {csv_path}.")

    finally:
        csv_file.close()
        if piezo_hdl is not None:
            mdtClose(piezo_hdl)
        session.restore_and_close()
        print("Devices closed.")

    if sample > 0:
        redraw(t_hist, px_hist, py_hist)
        fig.savefig(plot_path, dpi=150)
    print(f"Saved {sample} samples to {csv_path}")
    print(f"Saved plot to {plot_path}")

    plt.ioff()
    if not interrupted:
        plt.show()
    else:
        plt.show(block=False)
        plt.pause(0.1)


if __name__ == "__main__":
    main()
