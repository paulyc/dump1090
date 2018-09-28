// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// lib1090.c: utility functions for using lib1090 independently
//
// Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
// Copyright (c) 2014-2016 Oliver Jowett <oliver@mutability.co.uk>
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
// This file incorporates work covered by the following copyright and
// permission notice:
//
//   Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
//   Copyright (c) 2014-2016 Oliver Jowett <oliver@mutability.co.uk>
//   Copyright (c) 2017 FlightAware LLC
//   Copyright (C) 2018 Paul Ciarlo <paul.ciarlo@gmail.com>
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//    *  Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//    *  Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "lib1090.h"
#include <pthread.h>
#include <math.h>

static struct lib1090Config_t lib1090Config = {
    0.0f, 0.0f, 0.0f,
    PTHREAD_ONCE_INIT, 0,
    { 0, 0 }, "/tmp/lib1090BeastOutput", 0, 0
};

static void signalHandler(int dummy) {
    MODES_NOTUSED(dummy);
    lib1090Uninit();
}

static void __lib1090Init() {
    modesInitConfig();
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    Modes.net = 1;
    Modes.sdr_type = SDR_NONE;
    Modes.fUserLat = lib1090Config.userLat;
    Modes.fUserLon = lib1090Config.userLon;
    Modes.fUserAltM = lib1090Config.userAltMeters;
    Modes.json_dir = "/tmp/piaware";
    Modes.mode_ac = 1;
    Modes.mode_ac_auto = 0;
    Modes.dc_filter = 1;
    Modes.check_crc = 1;
    Modes.nfix_crc = MODES_MAX_BITERRORS;
    Modes.net_output_raw_ports = NULL;
    Modes.net_output_sbs_ports = NULL;
    Modes.net_input_raw_ports = NULL;
    Modes.net_input_beast_ports = NULL;
    Modes.net_output_beast_ports = NULL;
    Modes.net_verbatim = 0;
    Modes.sample_rate = 4000000.0;

    modesInit();
    modesInitNet();
    modesInitStats();

    Modes.exit = 0;

    int err = pipe(lib1090Config.pipedes);
    if (err != 0) {
        switch (err) {
            //More than {OPEN_MAX} minus two file descriptors are already in use by this process.
            case EMFILE:
            // The number of simultaneously open files in the system would exceed a system-imposed limit.
            case ENFILE:
            default:
                break;
        }
    }

    unlink(lib1090Config.beastOutPipeName);
    err = mkfifo(lib1090Config.beastOutPipeName, 0666);
    if (err != 0) {
        //return errno;
    }

    lib1090Config.pipefd = open(lib1090Config.beastOutPipeName, O_WRONLY);
}

static int doInitOnce() {
    return pthread_once(&lib1090Config.initOnce, __lib1090Init);
}

int lib1090Init(float userLat, float userLon, float userAltMeters) {
    lib1090Config.userLat = userLat;
    lib1090Config.userLon = userLon;
    lib1090Config.userAltMeters = userAltMeters;
    return doInitOnce();
}

int lib1090Uninit() {
    int status = lib1090JoinThread(NULL);
    pthread_cond_destroy(&Modes.data_cond);
    pthread_mutex_destroy(&Modes.data_mutex);
    close(lib1090Config.pipedes[0]);
    close(lib1090Config.pipedes[1]);
    close(lib1090Config.pipefd);
    return status;
}

static void lib1090MainLoop() {
    pthread_mutex_lock(&Modes.data_mutex);

    while (!Modes.exit) {
        struct timespec start_time, ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 100000000;
        normalize_timespec(&ts);

        pthread_cond_timedwait(&Modes.data_cond, &Modes.data_mutex, &ts);

        // copy out reader CPU time and reset it
        add_timespecs(&Modes.reader_cpu_accumulator, &Modes.stats_current.reader_cpu, &Modes.stats_current.reader_cpu);
        Modes.reader_cpu_accumulator.tv_sec = 0;
        Modes.reader_cpu_accumulator.tv_nsec = 0;

        //pthread_mutex_unlock(&Modes.data_mutex);

        start_cpu_timing(&start_time);
        backgroundTasks();
        end_cpu_timing(&start_time, &Modes.stats_current.background_cpu);

        //pthread_mutex_lock(&Modes.data_mutex);
    }

    pthread_mutex_unlock(&Modes.data_mutex);
}

ssize_t lib1090HandleFrame(struct modesMessage *mm, uint8_t *frm, uint64_t timestamp) {
    uint8_t frameOut[MAX_BEAST_MSG_LEN];
    ssize_t res = lib1090FixupFrame(frm, frameOut);
    if (res == -1) {
        return -1;
    }

    res = lib1090DecodeFrame(mm, frameOut, timestamp, _3dBFS);
    switch (res) {
        case 0:
        case -1:
        default:
            break;
        case -2:
            return -2;
            break;
    }

    uint8_t beastbufferOut[MAX_BEAST_MSG_LEN];
    return lib1090FormatBeast(mm, beastbufferOut, sizeof(beastbufferOut), true);
}

static void* __lib1090RunThread(void* pparam) {
    lib1090MainLoop();
    return NULL;
}

// will return -EBUSY if the thread is already running
// ( <0 indicating it was an error returned by ME and not the pthread library)
int lib1090RunThread(void *udata) {
    int err = doInitOnce();
    if (err != 0) {
        return err;
    }
    if (lib1090Config.libThread != 0) {
        return -EBUSY;
    }
    if (udata == NULL) {
        udata = __builtin_return_address(0);
    }
    return pthread_create(&lib1090Config.libThread, NULL, __lib1090RunThread, udata);
}

int lib1090JoinThread(void **retptr) {
    // thread already joined or never created
    if (lib1090Config.libThread == 0) {
        return -EINVAL;
    }
    void *dummy;
    if (retptr == NULL) {
        retptr = &dummy;
    }
    pthread_mutex_lock(&Modes.data_mutex);
    Modes.exit = 1;
    pthread_cond_signal(&Modes.data_cond);
    pthread_mutex_unlock(&Modes.data_mutex);
    int status = pthread_join(lib1090Config.libThread, retptr);
    if (status == 0) {
        lib1090Config.libThread = 0;
    }
    return status;
}

// returns number of errors fixed, or -1 if it was unfixable
int lib1090FixupFrame(uint8_t *frameIn, uint8_t *frameOut) { // check crc, fix if possible
    const int msglen = modesMessageLenByType(frameIn[0] >> 3);
    memcpy(frameOut, frameIn, msglen / 8);
    const uint32_t syndrome = modesChecksum(frameIn, msglen);
    struct errorinfo * info = modesChecksumDiagnose(syndrome, msglen);
    if (info != NULL) {
        modesChecksumFix(frameOut, info);
        return info->errors;
    } else {
        return -1;
    }
}

ssize_t lib1090DecodeFrame(struct modesMessage *mm, uint8_t *frame, uint64_t timestamp, double signalLevel) {
    // Decode the received message
    memset(mm, '\0', sizeof(struct modesMessage));

    // For consistency with how the Beast / Radarcape does it,
    // we report the timestamp at the end of bit 56 (even if
    // the frame is a 112-bit frame)
    struct timespec tp;
    clock_gettime(CLOCK_REALTIME, &tp);
    /*mm->timestampMsg = tp.tv_nsec + receiveclock_ns_elapsed(0, 56); // + j*5 + (8 + 56) * 12 + bestphase; /// these magic numbers//// 12 because of 12MHz... i dunno wtf

     // compute message receive time as block-start-time + difference in the 12MHz clock
     mm->sysTimestampMsg = mm->timestampMsg + receiveclock_ms_elapsed(timestamp, mm->timestampMsg); // idk
     */

    mm->timestampMsg = timestamp + 56*12; //j*5 + (8 + 56) * 12 + bestphase;

    // compute message receive time as block-start-time + difference in the 12MHz clock
    mm->sysTimestampMsg = timestamp + receiveclock_ms_elapsed(timestamp, mm->timestampMsg);
    mm->score = 100;

    // measure signal power
    {
        const int signal_len = mm->msgbits*12/5;
        const double signal_power = signalLevel * signal_len;
        mm->signalLevel = signalLevel;
        Modes.stats_current.signal_power_sum += signal_power;
        Modes.stats_current.signal_power_count += signal_len;

        if (mm->signalLevel > Modes.stats_current.peak_signal_power)
            Modes.stats_current.peak_signal_power = mm->signalLevel;
        if (mm->signalLevel > _3dBFS)
            Modes.stats_current.strong_signal_count++; // signal power above -3dBFS
    }

    // return 0 if all OK
    //   -1: message might be valid, but we couldn't validate the CRC against a known ICAO
    //   -2: bad message or unrepairable CRC error
    const int result = decodeModesMessage(mm, frame);
    switch (result) {
        case 0:
            Modes.stats_current.demod_accepted[mm->correctedbits]++;
            break;
        case -1:
            Modes.stats_current.demod_rejected_unknown_icao++;
            return -1;
            break;
        case -2:
            Modes.stats_current.demod_rejected_bad++;
            return -2;
            break;
        default:
            break;
    }

    struct aircraft *a = trackUpdateFromMessage(mm);
    ++Modes.stats_current.messages_total;
    if (a != NULL) {
        // log something?
    }

    displayModesMessage(mm);
    modesQueueOutput(mm, a);

    return 0;
}

ssize_t lib1090FormatBeast(struct modesMessage *mm, uint8_t *beastBufferOut, size_t beastBufferLen, bool writeToPipe) {
    const ssize_t msgLen = formatBeastMessage(mm, beastBufferOut, beastBufferLen);
    if (msgLen <= 0) {
        fprintf(stderr, "formatBeastMessage returned error %zd\n", msgLen);
        return msgLen;
    } else if (writeToPipe) {
        const size_t written = write(lib1090Config.pipefd, beastBufferOut, msgLen);
        if (written != msgLen) {
            fprintf(stderr, "fwrite returned error %d while writing beast message to pipe\n", errno);
            return errno;
        }
    }
    return msgLen;
}

void lib1090GetModes(struct modes_t** modesOut, struct lib1090Config_t** lib1090ConfigOut) {
    if (modesOut != NULL) {
        *modesOut = &Modes;
    }
    if (lib1090ConfigOut != NULL) {
        *lib1090ConfigOut = &lib1090Config;
    }
}
