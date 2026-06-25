"""
Polarization stability test: track the beam's total power (and peak
intensity) on the Thorlabs BC1 beam profiler over a long duration
(hours-scale), e.g. to monitor power drift caused by slow polarization
drift through a polarization-sensitive element upstream.

No piezo control is involved -- this only watches the camera. Data is
written to the CSV incrementally (one flush per sample), so a power loss
or hard kill mid-run still leaves everything collected up to that point on
disk. Press Ctrl+C at any time to stop early -- the run still ends with the
usual final plot.

Hardware:
  - Thorlabs BC1-series beam profiler, driven via ./tlbc1.py
"""

import csv
import datetime
import os
import time

import matplotlib.pyplot as plt

import tlbc1
import bc1_stability_common as bc1c

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
DURATION_HOURS = 4.0     # total run time; set to None to run until Ctrl+C
SAMPLE_INTERVAL_S = 5.0  # target time between samples -- camera latency (~2-4s/call) may
                          # dominate and make the actual cadence slower than this on its own

FIXED_EXPOSURE_MS = 0.5   # auto-exposure is disabled for the whole run: it would actively
                           # compensate for power drift, masking the very thing we're
                           # trying to measure
WAVELENGTH_NM = None       # set to your laser's wavelength (nm) for accurate power readings;
                            # None leaves the camera's current calibration untouched

PLOT_UPDATE_EVERY = 10    # redraw (and snapshot-save) the live plot every N samples

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))


def main():
    try:
        session = bc1c.connect(fixed_exposure_ms=FIXED_EXPOSURE_MS, wavelength_nm=WAVELENGTH_NM)
    except RuntimeError as e:
        print(e)
        return
    print("Connected to beam profiler", session.info["model_name"], session.info["serial_number"])

    duration_s = DURATION_HOURS * 3600.0 if DURATION_HOURS is not None else None
    if duration_s is not None:
        print(f"Running for {DURATION_HOURS:.2f} hours, ~{SAMPLE_INTERVAL_S:.1f}s/sample "
              f"(camera latency may dominate). Press Ctrl+C to stop early.")
    else:
        print("Running until Ctrl+C is pressed.")

    run_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(OUTPUT_DIR, f"polarization_stability_{run_stamp}.csv")
    plot_path = os.path.join(OUTPUT_DIR, f"polarization_stability_{run_stamp}.png")

    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow([
        "sample", "timestamp", "elapsed_s",
        "total_power_mW", "peak_intensity_counts", "peak_power_density_mW_per_um2",
        "centroid_x_um", "centroid_y_um",
    ])
    csv_file.flush()

    # --- live plot setup ---
    plt.ion()
    fig, ax_power = plt.subplots(figsize=(9, 5))
    line_power, = ax_power.plot([], [], "-", color="tab:green", label="Total power")
    ax_power.set_xlabel("Elapsed time (s)")
    ax_power.set_ylabel("Total power (mW)", color="tab:green")
    ax_power.tick_params(axis="y", labelcolor="tab:green")

    ax_peak = ax_power.twinx()
    line_peak, = ax_peak.plot([], [], "-", color="tab:purple", alpha=0.5, label="Peak intensity")
    ax_peak.set_ylabel("Peak intensity (counts)", color="tab:purple")
    ax_peak.tick_params(axis="y", labelcolor="tab:purple")

    ax_power.set_title("Power stability")
    fig.tight_layout()

    def redraw(t_hist, power_hist, peak_hist):
        line_power.set_data(t_hist, power_hist)
        line_peak.set_data(t_hist, peak_hist)
        ax_power.relim(); ax_power.autoscale_view()
        ax_peak.relim(); ax_peak.autoscale_view()
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    start_time = time.time()
    t_hist, power_hist, peak_hist = [], [], []
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

            total_power_mw = 10.0 ** (scan.totalPower / 10.0)
            centroid_x_um = (scan.centroidPositionX - session.center_x) * session.pixel_pitch_h
            centroid_y_um = (scan.centroidPositionY - session.center_y) * session.pixel_pitch_v

            timestamp = datetime.datetime.now()
            writer.writerow([
                sample, timestamp.isoformat(), f"{elapsed:.3f}",
                total_power_mw, scan.peakIntensity, scan.peakPowerDensity,
                centroid_x_um, centroid_y_um,
            ])
            csv_file.flush()

            t_hist.append(elapsed)
            power_hist.append(total_power_mw)
            peak_hist.append(scan.peakIntensity)

            print(f"[{timestamp:%H:%M:%S}] t={elapsed/3600:6.2f}h sample={sample} "
                  f"Power={total_power_mw:.4f} mW  PeakIntensity={scan.peakIntensity:.0f}")

            if sample % PLOT_UPDATE_EVERY == 0:
                redraw(t_hist, power_hist, peak_hist)
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
        session.restore_and_close()
        print("Device closed.")

    if sample > 0:
        redraw(t_hist, power_hist, peak_hist)
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
