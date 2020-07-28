// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// lib1090.h: main library header
// includes all other dump1090 headers except system headers
// include this to use lib1090 exports
//
// Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
// Copyright (c) 2014-2016 Oliver Jowett <oliver@mutability.co.uk>
// Copyright (c) 2017 FlightAware LLC
// Copyright (C) 2018-2019 Paul Ciarlo <paul.ciarlo@gmail.com>
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
//   Copyright (C) 2018-2019 Paul Ciarlo <paul.ciarlo@gmail.com>
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

#ifndef DUMP1090_LIB1090_H
#define DUMP1090_LIB1090_H

#include "dump1090.h"

// from lib1090.c
static const double _3dBFS = 0.501187234; //pow(10.0, -3.0/10.0);

struct lib1090Config_t {
    const char *userLat;
    const char *userLon;
    const char *userAltMeters;
    int pipedes[2];
    const char *beastOutPipeName;
    int pipefd;
    pid_t childPid;
    double sample_rate;
    const char *jsonDir;
};

void lib1090GetConfig(struct lib1090Config_t **configOut);
void lib1090GetModes(struct _Modes** modesOut);

int lib1090Init();
int lib1090Uninit();

int lib1090RunThread(void *udata);
int lib1090JoinThread(void **retptr);

ssize_t lib1090HandleFrame(struct modesMessage *mm, uint8_t *frm, uint64_t timestamp);
int lib1090FixupFrame(uint8_t *frameIn, uint8_t *frameOut); // check crc, fix if possible
ssize_t lib1090DecodeFrame(struct modesMessage *mm, uint8_t *frame, uint64_t timestamp, double signalLevel);
ssize_t lib1090FormatBeast(struct modesMessage *mm, uint8_t *beastBufferOut, size_t beastBufferLen, bool writeToPipe);

struct dump1090Fork_t {
    const char *userLat;
    const char *userLon;
    int pipedes[2];
    pid_t childPid;
    double sample_rate;
    const char *jsonDir;
    char scratch[128];
};

int lib1090InitDump1090(struct dump1090Fork_t **forkInfoOut);
int lib1090ForkDump1090(struct dump1090Fork_t *forkInfo);
int lib1090KillDump1090(struct dump1090Fork_t *forkInfo);
int lib1090FreeDump1090(struct dump1090Fork_t **pForkInfo);

#ifdef __cplusplus
}
#endif

#endif // DUMP1090_LIB1090_H
