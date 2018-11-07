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

#include "dump1090.h"
#include "sdr_lime.h"

#include <lime/LimeSuite.h>

typedef int16_t sc16_t[2];

static struct {
    unsigned decimation;
    unsigned lpf_bandwidth;

    lms_device_t *device;
    lms_info_str_t device_list[10];

    lms_stream_t stream;
    lms_stream_status_t stream_status;

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
        if (!strcasecmp(argv[j], "bypass")) {
            LimeSDR.lpf_bandwidth = 0;
        } else {
            LimeSDR.lpf_bandwidth = atoi(argv[j]);
        }
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

    if ((status = LMS_SetLPF(LimeSDR.device, LMS_CH_RX, 0, LimeSDR.lpf_bandwidth > 0 ? 1 : 0)) < 0) {
        fprintf(stderr, "LMS_SetLPF failed: %s\n", LMS_GetLastErrorMessage());
        goto error;
    }

    if (LimeSDR.lpf_bandwidth > 0) {
        if ((status = LMS_SetLPFBW(LimeSDR.device, LMS_CH_RX, 0, LimeSDR.lpf_bandwidth)) < 0) {
            fprintf(stderr, "LMS_SetLPFBW failed: %s\n", LMS_GetLastErrorMessage());
            goto error;
        }
    }

    /* disable tx channel, just in case */
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
    sc16_t samples[MODES_MAG_BUF_SAMPLES];
    lms_stream_meta_t meta;
    static uint64_t nextTimestamp = 0;
    static bool dropping = false;
    static struct timespec thread_cpu;
    static unsigned timeouts = 0;

    if (LimeSDR.device == NULL) {
        return;
    }

    int status;
    if ((status = LMS_SetupStream(LimeSDR.device, &LimeSDR.stream)) < 0) {
        fprintf(stderr, "LMS_SetupStream failed: %s\n", LMS_GetLastErrorMessage());
        return;
    }

    if ((status = LMS_StartStream(&LimeSDR.stream)) < 0) {
        fprintf(stderr, "LMS_StartStream failed: %s\n", LMS_GetLastErrorMessage());
        LMS_DestroyStream(LimeSDR.device, &LimeSDR.stream);
        return;
    }

    start_cpu_timing(&thread_cpu);

    timeouts = 0; // reset to zero when we get a callback with some data
    // record initial time for later sys timestamp calculation
    uint64_t entryTimestamp = mstime();

    pthread_mutex_lock(&Modes.data_mutex);

    while (!Modes.exit) {
        pthread_mutex_unlock(&Modes.data_mutex);

        int nSamples = LMS_RecvStream(&LimeSDR.stream, samples, MODES_MAG_BUF_SAMPLES, &meta, 5000);

        pthread_mutex_lock(&Modes.data_mutex);

        if (nSamples == -1) {
            fprintf(stderr, "LMS_RecvStream failed: %s\n", LMS_GetLastErrorMessage());

            // could be timeout? or another error? I don't know how to tell so lets just
            // quit after receiving too many errors but assume they are timeouts otherwise
            if (++timeouts > 100) {
                break;
            } else {
                continue;
            }
        }

        unsigned next_free_buffer = (Modes.first_free_buffer + 1) % MODES_MAG_BUFFERS;
        struct mag_buf *outbuf = &Modes.mag_buffers[Modes.first_free_buffer];
        struct mag_buf *lastbuf = &Modes.mag_buffers[(Modes.first_free_buffer + MODES_MAG_BUFFERS - 1) % MODES_MAG_BUFFERS];
        unsigned free_bufs = (Modes.first_filled_buffer - next_free_buffer + MODES_MAG_BUFFERS) % MODES_MAG_BUFFERS;

        if (free_bufs == 0 || (dropping && free_bufs < MODES_MAG_BUFFERS/2)) {
            // FIFO is full. Drop this block.
            dropping = true;
            continue;
        }

        dropping = false;
        pthread_mutex_unlock(&Modes.data_mutex);

        // Copy trailing data from last block (or reset if not valid)
        if (outbuf->dropped == 0) {
            memcpy(outbuf->data, lastbuf->data + lastbuf->length, Modes.trailing_samples * sizeof(uint16_t));
        } else {
            memset(outbuf->data, 0, Modes.trailing_samples * sizeof(uint16_t));
        }

        // start handling metadata blocks
        outbuf->dropped = 0;
        outbuf->length = 0;
        outbuf->mean_level = outbuf->mean_power = 0;

        // Compute the sample timestamp for the start of the block
        outbuf->sampleTimestamp = nextTimestamp * 12e6 / Modes.sample_rate / LimeSDR.decimation;

        // Convert a block of data
        double mean_level, mean_power;
        LimeSDR.converter(samples, &outbuf->data[Modes.trailing_samples + outbuf->length], nSamples, LimeSDR.converter_state, &mean_level, &mean_power);
        outbuf->length += nSamples;
        outbuf->mean_level += mean_level;
        outbuf->mean_power += mean_power;
        nextTimestamp += nSamples * LimeSDR.decimation;
        timeouts = 0;

        // Get the approx system time for the start of this block
        unsigned block_duration = 1e3 * outbuf->length / Modes.sample_rate;
        outbuf->sysTimestamp = entryTimestamp - block_duration;

        // Push the new data to the demodulation thread
        pthread_mutex_lock(&Modes.data_mutex);

        // accumulate CPU while holding the mutex, and restart measurement
        end_cpu_timing(&thread_cpu, &Modes.reader_cpu_accumulator);
        start_cpu_timing(&thread_cpu);

        Modes.mag_buffers[next_free_buffer].dropped = 0;
        Modes.mag_buffers[next_free_buffer].length = 0;  // just in case
        Modes.first_free_buffer = next_free_buffer;

        pthread_cond_signal(&Modes.data_cond);
    }

    pthread_mutex_unlock(&Modes.data_mutex);

    LMS_StopStream(&LimeSDR.stream);
    LMS_DestroyStream(LimeSDR.device, &LimeSDR.stream);
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
