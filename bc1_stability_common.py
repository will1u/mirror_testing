"""
Shared setup/teardown helper for the long-running (hours-scale) stability
tests (stability_test.py, polarization_stability_test.py).

Both scripts want the same thing from the camera: a fixed (non-auto)
exposure so the measurement itself doesn't mask real drift, and optionally
a calibrated wavelength so power readings are meaningful -- then a clean
restore of whatever was configured before, on exit.
"""

import tlbc1


class BC1Session:
    def __init__(self, vi, info, pixel_count_x, pixel_count_y, pixel_pitch_h, pixel_pitch_v):
        self.vi = vi
        self.info = info
        self.pixel_count_x = pixel_count_x
        self.pixel_count_y = pixel_count_y
        self.pixel_pitch_h = pixel_pitch_h
        self.pixel_pitch_v = pixel_pitch_v
        self.center_x = pixel_count_x / 2.0
        self.center_y = pixel_count_y / 2.0

        self._prev_auto_exposure = None
        self._prev_exposure_time = None
        self._prev_wavelength = None

    def configure(self, fixed_exposure_ms=None, wavelength_nm=None):
        """Disable auto-exposure (it actively compensates for power/intensity
        changes, which would mask the real drift a stability test is trying
        to measure) and optionally fix the wavelength calibration. Remembers
        the previous settings so restore() can put them back."""
        self._prev_auto_exposure = tlbc1.get_auto_exposure(self.vi)
        self._prev_exposure_time = tlbc1.get_exposure_time(self.vi)
        tlbc1.set_auto_exposure(self.vi, False)
        if fixed_exposure_ms is not None:
            tlbc1.set_exposure_time(self.vi, fixed_exposure_ms)

        if wavelength_nm is not None:
            self._prev_wavelength = tlbc1.get_wavelength(self.vi)
            wl_min, wl_max = tlbc1.get_wavelength_range(self.vi)
            # the driver rejects values exactly at the range boundary
            # (VI_ERROR_INV_SPACE), so nudge in from the edge if needed
            wl = min(max(wavelength_nm, wl_min + 0.5), wl_max - 0.5)
            tlbc1.set_wavelength(self.vi, wl)

    def restore_and_close(self):
        try:
            if self._prev_exposure_time is not None:
                tlbc1.set_exposure_time(self.vi, self._prev_exposure_time)
            if self._prev_auto_exposure is not None:
                tlbc1.set_auto_exposure(self.vi, self._prev_auto_exposure)
            if self._prev_wavelength is not None:
                tlbc1.set_wavelength(self.vi, self._prev_wavelength)
        finally:
            tlbc1.close(self.vi)


def connect(fixed_exposure_ms=None, wavelength_nm=None):
    """Connect to the first available BC1 camera and configure it for a
    stability measurement. Returns a BC1Session; call .restore_and_close()
    when done."""
    vi, info = tlbc1.open_first_device()
    pixel_count_x, pixel_count_y, pixel_pitch_h, pixel_pitch_v = tlbc1.get_sensor_information(vi)
    session = BC1Session(vi, info, pixel_count_x, pixel_count_y, pixel_pitch_h, pixel_pitch_v)
    session.configure(fixed_exposure_ms=fixed_exposure_ms, wavelength_nm=wavelength_nm)
    return session


def check_saturation(scan, precision_mode_fast=True):
    """Warn if the sensor is saturated -- once clipped, peak/power readings
    flatline at the ceiling and no longer reflect real fluctuations."""
    ceiling = 255.0 if precision_mode_fast else 4095.0
    if scan.peakIntensity >= ceiling:
        print(f"WARNING: sensor saturated (peak intensity {scan.peakIntensity:.0f} >= {ceiling:.0f}). "
              f"Reduce laser power / add an ND filter, or lower the fixed exposure time.")
