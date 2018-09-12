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

static struct {
    float userLat;
    float userLon;
    float userAltMeters;
    pthread_once_t initOnce;
    pthread_t libThread;
} lib1090Config = { 0.0f, 0.0f, 0.0f, PTHREAD_ONCE_INIT, 0 };

static void __lib1090Init() {
    modesInitConfig();
    install_signal_handlers(false);

    Modes.sdr_type = SDR_NONE;
    Modes.fUserLat = lib1090Config.userLat;
    Modes.fUserLon = lib1090Config.userLon;
    Modes.fUserAltM = lib1090Config.userAltMeters;
    Modes.json_dir = "/var/cache/piaware";
    Modes.mode_ac = 1;

    modesInit();
    modesInitNet();
    modesInitStats();
}

int doInitOnce() {
    return pthread_once(&lib1090Config.initOnce, __lib1090Init);
}

int lib1090Init(float userLat, float userLon, float userAltMeters) {
    lib1090Config.userLat = userLat;
    lib1090Config.userLon = userLon;
    lib1090Config.userAltMeters = userAltMeters;
    return doInitOnce();
}

static void lib1090MainLoop() {
    pthread_mutex_lock(&Modes.data_mutex);
    while (Modes.exit == 0) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 100000000;
        normalize_timespec(&ts);

        pthread_cond_timedwait(&Modes.data_cond, &Modes.data_mutex, &ts);

    }
    pthread_mutex_unlock(&Modes.data_mutex);
}

int lib1090HandleFrame(struct modesMessage *mm, uint8_t *frm, uint64_t timestamp) {
    memset(mm, '\0', sizeof(struct modesMessage));

    // For consistency with how the Beast / Radarcape does it,
    // we report the timestamp at the end of bit 56 (even if
    // the frame is a 112-bit frame)
    mm->timestampMsg = timestamp; // + j*5 + (8 + 56) * 12 + bestphase; /// these magic numbers//// 12 because of 12MHz... i dunno wtf

    // compute message receive time as block-start-time + difference in the 12MHz clock
    mm->sysTimestampMsg = timestamp; // + receiveclock_ms_elapsed(mag->sampleTimestamp, mm.timestampMsg); // idk

    //mm.score = bestscore;

    // Decode the received message
    {
        // return 0 if all OK
        //   -1: message might be valid, but we couldn't validate the CRC against a known ICAO
        //   -2: bad message or unrepairable CRC error
        int result = decodeModesMessage(mm, frm);
        if (result < 0) {
            if (result == -1)
                Modes.stats_current.demod_rejected_unknown_icao++;
            else
                Modes.stats_current.demod_rejected_bad++;
            return result;
        } else {
            Modes.stats_current.demod_accepted[mm->correctedbits]++;
        }
    }

    /*
        * // measure signal power
    {
        double signal_power;
        uint64_t scaled_signal_power = 0;
        int signal_len = msglen*12/5;
        int k;

        for (k = 0; k < signal_len; ++k) {
            uint32_t mag = m[j+19+k];
            scaled_signal_power += mag * mag;
        }

        signal_power = scaled_signal_power / 65535.0 / 65535.0;
        mm.signalLevel = signal_power / signal_len;
        Modes.stats_current.signal_power_sum += signal_power;
        Modes.stats_current.signal_power_count += signal_len;
        sum_scaled_signal_power += scaled_signal_power;

        if (mm.signalLevel > Modes.stats_current.peak_signal_power)
            Modes.stats_current.peak_signal_power = mm.signalLevel;
        if (mm.signalLevel > 0.50119)
            Modes.stats_current.strong_signal_count++; // signal power above -3dBFS
    } */

    // Pass data to the next layer
    useModesMessage(mm);
    return 0;
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
    void *dummy;
    if (retptr == NULL) {
        retptr = &dummy;
    }
    Modes.exit = 1;
    int status = pthread_join(lib1090Config.libThread, retptr);
    if (status == 0) {
        lib1090Config.libThread = 0;
    }
    return status;
}
