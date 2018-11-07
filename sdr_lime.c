// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sdr_lime.c: LimeSDR support
//
// Copyright (c) 2016-2017 Oliver Jowett <oliver@mutability.co.uk>
// Copyright (c) 2017 FlightAware LLC
// Copyright (C) 2018 Paul Ciarlo <paul.ciarlo@gmail.com>
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "sdr_lime.h"
#include "dump1090.h"

#include <lime/LimeSuite.h>

static struct {
    unsigned decimation;
    unsigned lpf_bandwidth;

    lms_device_t *device;
    lms_info_str_t device_list[10];

    iq_convert_fn converter;
    struct converter_state *converter_state;
} LimeSDR;

void limesdrInitConfig()
{
    LimeSDR.decimation = 1;
    LimeSDR.lpf_bandwidth = 1750000;
    LimeSDR.device = NULL;
    LimeSDR.converter = NULL;
    LimeSDR.converter_state = NULL;
}

void limesdrShowHelp()
{
    printf("      LimeSDR-specific options (use with --device-type limesdr)\n");
    printf("\n");
    //printf("--device <ident>         select device by bladeRF 'device identifier'\n");
    //printf("--bladerf-fpga <path>    use alternative FPGA bitstream ('' to disable FPGA load)\n");
    printf("--limesdr-decimation <N> assume FPGA decimates by a factor of N\n");
    printf("--limesdr-bandwidth <hz> set LPF bandwidth ('bypass' to bypass the LPF)\n");
    printf("\n");
}

bool limesdrHandleOption(int argc, char **argv, int *jptr)
{
    int j = *jptr;
    bool more = (j+1 < argc);
    if (!strcmp(argv[j], "--limesdr-decimation") && more) {
        LimeSDR.decimation = atoi(argv[++j]);
    } else if (!strcmp(argv[j], "--limesdr-bandwidth") && more) {
        ++j;
        LimeSDR.lpf_bandwidth = atoi(argv[j]);
    } else {
        return false;
    }

    *jptr = j;
    return true;
}

bool limesdrOpen()
{
    if (LimeSDR.device != NULL) {
        return false;
    }
    int count = LMS_GetDeviceList(LimeSDR.device_list), status;
    if (count == -1) {
        fprintf(stderr, "Some error calling LMS_GetDeviceList() in limesdrOpen(): %s\n", LMS_GetLastErrorMessage());
        return false;
    } else if (count == 0) {
        fprintf(stderr, "No devices found by LMS_GetDeviceList() in limesdrOpen()\n");
        return false;
    } else {
        int devIndex = 0;
        status = LMS_Open(&LimeSDR.device, LimeSDR.device_list[devIndex], NULL);
        if (status == -1) {
            fprintf(stderr, "Some error calling LMS_Open() in limesdrOpen(): %s\n", LMS_GetLastErrorMessage());
            return false;
        }
    }
    if ((status = LMS_SetSampleRate(LimeSDR.device, Modes.sample_rate, LimeSDR.decimation)) < 0) {
        fprintf(stderr, "LMS_SetSampleRate failed: %s\n", LMS_GetLastErrorMessage());
        goto error;
    }

    if ((status = LMS_SetLOFrequency(LimeSDR.device, LMS_CH_RX, 0, Modes.freq)) < 0) {
        fprintf(stderr, "LMS_SetLOFrequency failed: %s\n", LMS_GetLastErrorMessage());
        goto error;
    }

    if ((status = LMS_SetAntenna(LimeSDR.device, LMS_CH_RX, 0, LMS_PATH_LNAW)) < 0) {
        fprintf(stderr, "LMS_SetAntenna failed: %s\n", LMS_GetLastErrorMessage());
        goto error;
    }

    if ((status = LMS_SetLPF(LimeSDR.device, LMS_CH_RX, 0, 1)) < 0) {
        fprintf(stderr, "LMS_SetLPF failed: %s\n", LMS_GetLastErrorMessage());
        goto error;
    }

    if ((status = LMS_SetLPFBW(LimeSDR.device, LMS_CH_RX, 0, LimeSDR.lpf_bandwidth)) < 0) {
        fprintf(stderr, "LMS_SetLPFBW failed: %s\n", LMS_GetLastErrorMessage());
        goto error;
    }

    /* turn the tx gain right off, just in case */
    if ((status = LMS_EnableChannel(LimeSDR.device, LMS_CH_TX, 0, 0)) < 0) {
        fprintf(stderr, "LMS_EnableChannel(TX) failed: %s\n", LMS_GetLastErrorMessage());
        goto error;
    }

    if ((status = LMS_SetGaindB(LimeSDR.device, LMS_CH_RX, 0, Modes.gain)) < 0) {
        fprintf(stderr, "LMS_SetGaindB(RX) failed: %s\n", LMS_GetLastErrorMessage());
        goto error;
    }

    if ((status = LMS_Calibrate(LimeSDR.device, LMS_CH_RX, 0, LimeSDR.lpf_bandwidth, 0)) < 0) {
        fprintf(stderr, "LMS_Calibrate failed: %s\n", LMS_GetLastErrorMessage());
        goto error;
    }

    //show_config();

    LimeSDR.converter = init_converter(INPUT_SC16,
                                       Modes.sample_rate,
                                       Modes.dc_filter,
                                       &LimeSDR.converter_state);
    if (!LimeSDR.converter) {
        fprintf(stderr, "can't initialize sample converter\n");
        goto error;
    }

    return true;

error:
    if (LimeSDR.device != NULL) {
        LMS_Close(LimeSDR.device);
        LimeSDR.device = NULL;
    }
    return false;
}

void limesdrRun()
{

}

void limesdrClose()
{
    if (LimeSDR.converter != NULL) {
        cleanup_converter(LimeSDR.converter_state);
        LimeSDR.converter = NULL;
        LimeSDR.converter_state = NULL;
    }

    if (LimeSDR.device != NULL) {
        int res = LMS_Close(LimeSDR.device);
        if (res == -1) {
            fprintf(stderr, "Some error calling LMS_Close() in limesdrClose()\n");
        }
        LimeSDR.device = NULL;
    }
}
