"""
Minimal ctypes wrapper around the Thorlabs TLBC1 VISA instrument driver
(the BC1-series beam profiler camera SDK), mirroring the calling style of
MDT_COMMAND_LIB.py so it can be combined easily with that piezo driver.

DLL: C:\\Program Files\\IVI Foundation\\VISA\\Win64\\Bin\\TLBC1_64.dll
     (64-bit build of the same driver used by bc1_example.c, which links
     against the 32-bit TLBC1_32.dll instead.)
Headers (for reference): C:\\Program Files (x86)\\IVI Foundation\\VISA\\WinNT\\Include\\TLBC1*.h
"""

from ctypes import (
    WinDLL, Structure, POINTER, create_string_buffer,
    c_uint16, c_uint32, c_int32, c_float, c_double, byref,
)

_DLL_PATH = r"C:\Program Files\IVI Foundation\VISA\Win64\Bin\TLBC1_64.dll"

_lib = WinDLL(_DLL_PATH)

TLBC1_MAX_COLUMNS = 4096
TLBC1_MAX_ROWS = 3000
TLBC1_ERR_DESCR_BUFFER_SIZE = 256

VI_NULL = 0
VI_TRUE = 1
VI_FALSE = 0


class TLBC1_Calculations(Structure):
    """Mirrors TLBC1_Calculations in TLBC1_Calculations.h field-for-field."""
    _fields_ = [
        ("isValid", c_uint16),

        ("baseLevel", c_double),
        ("lightShieldedPixelMeanIntensity", c_double),
        ("minIntensity", c_double),
        ("maxIntensity", c_double),
        ("saturation", c_double),
        ("saturatedPixel", c_double),

        ("imageWidth", c_uint16),
        ("imageHeight", c_uint16),

        ("peakPositionX", c_uint16),
        ("peakPositionY", c_uint16),
        ("peakIntensity", c_double),
        ("centroidPositionX", c_float),
        ("centroidPositionY", c_float),
        ("fourSigmaX", c_float),
        ("fourSigmaY", c_float),
        ("fourSigmaR", c_float),
        ("fourSigmaXY", c_float),

        ("beamWidthIsoX", c_double),
        ("beamWidthIsoY", c_double),
        ("beamWidthIsoXSimple", c_double),
        ("beamWidthIsoYSimple", c_double),
        ("ellipticityIso", c_double),
        ("azimuthAngle", c_double),

        ("ellipseDiaMin", c_float),
        ("ellipseDiaMax", c_float),
        ("ellipseDiaMean", c_float),
        ("ellipseOrientation", c_float),
        ("ellipseEllipticity", c_float),
        ("ellipseEccentricity", c_float),
        ("ellipseCenterX", c_float),
        ("ellipseCenterY", c_float),
        ("ellipseFitAmplitude", c_float),
        ("rotAngleEllipseX", c_float),
        ("rotAngleEllipseY", c_float),
        ("ellipseWidthIsoX", c_float),
        ("ellipseWidthIsoY", c_float),

        ("totalPower", c_float),
        ("peakPowerDensity", c_float),

        ("beamWidthClipX", c_float),
        ("beamWidthClipY", c_float),

        ("gaussianFitCentroidPositionX", c_float),
        ("gaussianFitCentroidPositionY", c_float),
        ("gaussianFitRatingX", c_float),
        ("gaussianFitRatingY", c_float),
        ("gaussianFitDiameterX", c_float),
        ("gaussianFitDiameterY", c_float),

        ("calcAreaCenterX", c_float),
        ("calcAreaCenterY", c_float),
        ("calcAreaWidth", c_float),
        ("calcAreaHeight", c_float),
        ("calcAreaAngle", c_double),
        ("calcAreaLineOffset", c_double),

        ("profileValuesX", c_float * TLBC1_MAX_COLUMNS),
        ("profileValuesY", c_float * TLBC1_MAX_ROWS),
        ("profilePositionsX", c_float * TLBC1_MAX_COLUMNS),
        ("profilePositionsY", c_float * TLBC1_MAX_ROWS),
        ("profilePeakValueX", c_float),
        ("profilePeakValueY", c_float),
        ("profilePeakPosX", c_uint16),
        ("profilePeakPosY", c_uint16),

        ("effectiveArea", c_double),
        ("effectiveBeamDiameter", c_double),

        ("temperature", c_double),

        ("gaussianValuesX", c_float * TLBC1_MAX_COLUMNS),
        ("gaussianValuesY", c_float * TLBC1_MAX_ROWS),

        ("besselFitValuesX", c_float * TLBC1_MAX_COLUMNS),
        ("besselFitValuesY", c_float * TLBC1_MAX_ROWS),

        ("besselFitRatingX", c_float),
        ("besselFitRatingY", c_float),
    ]


_lib.TLBC1_get_device_count.argtypes = [c_uint32, POINTER(c_uint32)]
_lib.TLBC1_get_device_count.restype = c_int32

# manufacturer/model_name/serial_number/resource_name are passed as
# create_string_buffer() instances at call time, which are accepted by
# ctypes without declaring argtypes.
_lib.TLBC1_get_device_information.restype = c_int32

_lib.TLBC1_init.restype = c_int32
_lib.TLBC1_close.argtypes = [c_uint32]
_lib.TLBC1_close.restype = c_int32

_lib.TLBC1_identification_query.restype = c_int32
_lib.TLBC1_revision_query.restype = c_int32

_lib.TLBC1_get_sensor_information.argtypes = [
    c_uint32, POINTER(c_uint16), POINTER(c_uint16), POINTER(c_double), POINTER(c_double),
]
_lib.TLBC1_get_sensor_information.restype = c_int32

_lib.TLBC1_get_scan_data.argtypes = [c_uint32, POINTER(TLBC1_Calculations)]
_lib.TLBC1_get_scan_data.restype = c_int32

_lib.TLBC1_get_auto_exposure.argtypes = [c_uint32, POINTER(c_uint16)]
_lib.TLBC1_get_auto_exposure.restype = c_int32

_lib.TLBC1_set_auto_exposure.argtypes = [c_uint32, c_uint16]
_lib.TLBC1_set_auto_exposure.restype = c_int32

_lib.TLBC1_get_exposure_time.argtypes = [c_uint32, POINTER(c_double)]
_lib.TLBC1_get_exposure_time.restype = c_int32

_lib.TLBC1_set_exposure_time.argtypes = [c_uint32, c_double]
_lib.TLBC1_set_exposure_time.restype = c_int32

_lib.TLBC1_get_wavelength.argtypes = [c_uint32, POINTER(c_double)]
_lib.TLBC1_get_wavelength.restype = c_int32

_lib.TLBC1_set_wavelength.argtypes = [c_uint32, c_double]
_lib.TLBC1_set_wavelength.restype = c_int32

_lib.TLBC1_get_wavelength_range.argtypes = [c_uint32, POINTER(c_double), POINTER(c_double)]
_lib.TLBC1_get_wavelength_range.restype = c_int32

_lib.TLBC1_error_message.restype = c_int32


def error_message(vi, status_code):
    buf = create_string_buffer(TLBC1_ERR_DESCR_BUFFER_SIZE)
    _lib.TLBC1_error_message(vi, status_code, buf)
    return buf.value.decode(errors="replace")


class TLBC1Error(RuntimeError):
    def __init__(self, vi, status_code):
        self.status_code = status_code
        super().__init__(f"TLBC1 error {status_code}: {error_message(vi, status_code)}")


def _check(vi, status):
    if status != 0:
        raise TLBC1Error(vi, status)
    return status


def get_device_count():
    count = c_uint32(0)
    _check(VI_NULL, _lib.TLBC1_get_device_count(VI_NULL, byref(count)))
    return count.value


def get_device_information(device_index):
    manufacturer = create_string_buffer(256)
    model_name = create_string_buffer(256)
    serial_number = create_string_buffer(256)
    available = c_uint16(0)
    resource_name = create_string_buffer(256)

    status = _lib.TLBC1_get_device_information(
        c_uint32(VI_NULL), c_uint32(device_index),
        manufacturer, model_name, serial_number,
        byref(available), resource_name,
    )
    _check(VI_NULL, status)
    return {
        "manufacturer": manufacturer.value.decode(errors="replace"),
        "model_name": model_name.value.decode(errors="replace"),
        "serial_number": serial_number.value.decode(errors="replace"),
        "available": bool(available.value),
        "resource_name": resource_name.value.decode(errors="replace"),
    }


def init(resource_name, id_query=True, reset=False):
    vi = c_uint32(0)
    status = _lib.TLBC1_init(
        resource_name.encode(), c_uint16(VI_TRUE if id_query else VI_FALSE),
        c_uint16(VI_TRUE if reset else VI_FALSE), byref(vi),
    )
    _check(VI_NULL, status)
    return vi.value


def close(vi):
    return _lib.TLBC1_close(c_uint32(vi))


def identification_query(vi):
    instr_name = create_string_buffer(256)
    serial_number = create_string_buffer(256)
    _check(vi, _lib.TLBC1_identification_query(c_uint32(vi), instr_name, serial_number))
    return instr_name.value.decode(errors="replace"), serial_number.value.decode(errors="replace")


def revision_query(vi):
    driver_rev = create_string_buffer(256)
    firmware_rev = create_string_buffer(256)
    _check(vi, _lib.TLBC1_revision_query(c_uint32(vi), driver_rev, firmware_rev))
    return driver_rev.value.decode(errors="replace"), firmware_rev.value.decode(errors="replace")


def get_sensor_information(vi):
    pixel_count_x = c_uint16(0)
    pixel_count_y = c_uint16(0)
    pixel_pitch_h = c_double(0)
    pixel_pitch_v = c_double(0)
    _check(vi, _lib.TLBC1_get_sensor_information(
        c_uint32(vi), byref(pixel_count_x), byref(pixel_count_y),
        byref(pixel_pitch_h), byref(pixel_pitch_v),
    ))
    return pixel_count_x.value, pixel_count_y.value, pixel_pitch_h.value, pixel_pitch_v.value


def get_scan_data(vi):
    data = TLBC1_Calculations()
    _check(vi, _lib.TLBC1_get_scan_data(c_uint32(vi), byref(data)))
    return data


def get_auto_exposure(vi):
    auto_exposure = c_uint16(0)
    _check(vi, _lib.TLBC1_get_auto_exposure(c_uint32(vi), byref(auto_exposure)))
    return bool(auto_exposure.value)


def set_auto_exposure(vi, enabled):
    return _check(vi, _lib.TLBC1_set_auto_exposure(c_uint32(vi), c_uint16(VI_TRUE if enabled else VI_FALSE)))


def get_exposure_time(vi):
    exposure_time = c_double(0)
    _check(vi, _lib.TLBC1_get_exposure_time(c_uint32(vi), byref(exposure_time)))
    return exposure_time.value


def set_exposure_time(vi, exposure_time_ms):
    return _check(vi, _lib.TLBC1_set_exposure_time(c_uint32(vi), c_double(exposure_time_ms)))


def get_wavelength(vi):
    wavelength = c_double(0)
    _check(vi, _lib.TLBC1_get_wavelength(c_uint32(vi), byref(wavelength)))
    return wavelength.value


def set_wavelength(vi, wavelength_nm):
    return _check(vi, _lib.TLBC1_set_wavelength(c_uint32(vi), c_double(wavelength_nm)))


def get_wavelength_range(vi):
    min_wavelength = c_double(0)
    max_wavelength = c_double(0)
    _check(vi, _lib.TLBC1_get_wavelength_range(c_uint32(vi), byref(min_wavelength), byref(max_wavelength)))
    return min_wavelength.value, max_wavelength.value


def open_first_device():
    """Convenience helper: find and initialize the first available BC1 camera."""
    count = get_device_count()
    if count == 0:
        raise RuntimeError("No BC1 instrument found.")

    for index in range(count):
        info = get_device_information(index)
        if info["available"]:
            vi = init(info["resource_name"], id_query=True, reset=False)
            return vi, info

    raise RuntimeError("No BC1 instrument available (already open elsewhere?).")
