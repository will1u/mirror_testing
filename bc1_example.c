/*
 * Simple example for the Thorlabs BC1 beam profiler camera.
 *
 * Uses the TLBC1 VISA instrument driver (VXIpnp driver) that ships with
 * the Thorlabs Beam software / NI-VISA install:
 *   Headers: C:\Program Files (x86)\IVI Foundation\VISA\WinNT\Include\TLBC1.h
 *   Lib:     C:\Program Files (x86)\IVI Foundation\VISA\WinNT\lib\msc\TLBC1_32.lib
 *            C:\Program Files (x86)\IVI Foundation\VISA\WinNT\Lib_x64\msc\TLBC1_64.lib
 *   DLL:     C:\Program Files (x86)\IVI Foundation\VISA\WinNT\Bin\TLBC1_32.dll
 *
 * Connects to the first available BC1, prints identification info, and
 * reads a few scans of beam data (peak position, beam width, total power).
 *
 * Build: run build.bat (uses the MSVC compiler, cl.exe) from a
 * "Developer Command Prompt for VS".
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Windows.h>

#include <visa.h>
#include "TLBC1.h"

static void print_timestamp(void)
{
    SYSTEMTIME t;
    GetLocalTime(&t);
    printf("[%02u:%02u:%02u.%03u] ", t.wHour, t.wMinute, t.wSecond, t.wMilliseconds);
}

static ViSession instr = (ViSession)0xFF;

static void error_exit(ViStatus err)
{
    ViChar ebuf[TLBC1_ERR_DESCR_BUFFER_SIZE];

    TLBC1_error_message(instr, err, ebuf);
    fprintf(stderr, "ERROR: %s\n", ebuf);

    if (instr != (ViSession)0xFF)
        TLBC1_close(instr);

    printf("Press <ENTER> to exit\n");
    while (getchar() == EOF);

    exit((int)err);
}

int main(void)
{
    ViStatus err;
    ViUInt32 deviceCount = 0;
    ViChar resourceName[256];
    ViChar modelName[16];
    ViChar serialNumber[16];
    ViChar driverRev[16];
    ViChar firmwareRev[16];
    ViUInt16 pixelCountX, pixelCountY;
    ViReal64 pixelPitchH, pixelPitchV;
    TLBC1_Calculations scanData;
    int i;

    printf("Thorlabs BC1 beam profiler - simple C example\n\n");

    printf("Scanning for Thorlabs BC1 instruments ...\n");
    err = TLBC1_get_device_count(VI_NULL, &deviceCount);
    if (err != VI_SUCCESS)
        error_exit(err);

    if (deviceCount == 0)
    {
        printf("No BC1 instrument found.\n");
        return 1;
    }

    printf("Found %u instrument(s).\n", deviceCount);

    {
        ViUInt32 k;
        ViBoolean available = VI_FALSE;

        for (k = 0; k < deviceCount; k++)
        {
            err = TLBC1_get_device_information(VI_NULL, k, VI_NULL, VI_NULL,
                                                VI_NULL, &available, resourceName);
            if (err == VI_SUCCESS && available)
                break;
        }

        if (k >= deviceCount)
        {
            printf("No instrument available (is it already open elsewhere?).\n");
            return 1;
        }
    }

    printf("Initializing the device...\n");
    err = TLBC1_init(resourceName, VI_TRUE, VI_FALSE, &instr);
    if (err != VI_SUCCESS)
        error_exit(err);

    err = TLBC1_identification_query(instr, modelName, serialNumber);
    if (err != VI_SUCCESS)
        error_exit(err);
    printf("Model:          %s\n", modelName);
    printf("Serial number:  %s\n", serialNumber);

    err = TLBC1_revision_query(instr, driverRev, firmwareRev);
    if (err != VI_SUCCESS)
        error_exit(err);
    printf("Driver rev.:    %s\n", driverRev);
    printf("Firmware rev.:  %s\n", firmwareRev);

    err = TLBC1_get_sensor_information(instr, &pixelCountX, &pixelCountY,
                                        &pixelPitchH, &pixelPitchV);
    if (err != VI_SUCCESS)
        error_exit(err);
    printf("Sensor:         %u x %u px, pitch %.2f x %.2f um\n\n",
           pixelCountX, pixelCountY, pixelPitchH, pixelPitchV);

    for (i = 0; i < 5; i++)
    {
        double beamWidthClipXUm, beamWidthClipYUm, totalPowerMw;
        double peakPosXUm, peakPosYUm;

        err = TLBC1_get_scan_data(instr, &scanData);
        if (err != VI_SUCCESS)
            error_exit(err);

        /* beam widths are reported in pixels; scale by the sensor's pixel pitch to get um */
        beamWidthClipXUm = scanData.beamWidthClipX * pixelPitchH;
        beamWidthClipYUm = scanData.beamWidthClipY * pixelPitchV;

        /* totalPower is reported in dBm; convert to mW */
        totalPowerMw = pow(10.0, scanData.totalPower / 10.0);

        /* peak position is a raw pixel coordinate (origin at top-left); convert to
           um displacement from the sensor center */
        peakPosXUm = (scanData.profilePeakPosX - pixelCountX / 2.0) * pixelPitchH;
        peakPosYUm = (scanData.profilePeakPosY - pixelCountY / 2.0) * pixelPitchV;

        print_timestamp();
        printf("Scan %d: Peak=%.2f @ (%.2f, %.2f) um  BeamWidth=(%.2f, %.2f) um  TotalPower=%.3f mW\n",
               i + 1, scanData.peakIntensity, peakPosXUm, peakPosYUm,
               beamWidthClipXUm, beamWidthClipYUm, totalPowerMw);

        Sleep(200);
    }

    TLBC1_close(instr);
    instr = (ViSession)0xFF;

    printf("\nDone. Press <ENTER> to exit\n");
    while (getchar() == EOF);

    return 0;
}
