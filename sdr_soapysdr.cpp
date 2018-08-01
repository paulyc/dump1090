// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sdr_soapysdr.c: SoapySDR support
//
// Copyright (c) 2014-2017 Oliver Jowett <oliver@mutability.co.uk>
// Copyright (c) 2017 FlightAware LLC
// Copyright (c) 2018 Paul Ciarlo <paul.ciarlo@gmail.com
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

#include "dump1090.h"
#include "convert.h"
#include "sdr_soapysdr.h"

#include <SoapySDR/Device.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <memory>
#include <mutex>
#include <queue>
#include <climits>

using std::ostream;
using namespace SoapySDR;

template <typename T>
ostream &operator<<(ostream &lhs, const std::vector<T> &rhs);

ostream &operator<<(ostream &lhs, const Range &rhs) {
    lhs << "[" << rhs.minimum() << ", " << rhs.maximum() << "]";
    return lhs;
}

ostream &operator<<(ostream &lhs, const Kwargs &rhs) {
    lhs << KwargsToString(rhs);
    return lhs;
}

ostream &operator<<(ostream &lhs, const ArgInfo &rhs) {
    lhs << "ArgInfo(" << rhs.key << ", " << rhs.value << ", " << rhs.description << ", " << rhs.units << ", " <<
    rhs.type << ", " << rhs.options << ", " << rhs.optionNames << ", " << rhs.range << ")";
    return lhs;
}

template <typename T>
ostream &operator<<(ostream &lhs, const std::vector<T> &rhs) {
    lhs << "[";
    for (const T &a : rhs) {
        lhs << a << ", ";
    }
    lhs << "]";
    return lhs;
}

struct SoapyGain {
    std::string name;
    Range range;
    double current;
};

struct soapy_converter_state {
    soapy_converter_state() : dc_a(0.0),dc_b(0.0),z1_I(0.0),z1_Q(0.0){}
    double dc_a;
    double dc_b;
    double z1_I;
    double z1_Q;
};

typedef int16_t cs16_t[2];
static struct {
    Device *dev;
    int ppm_error;
    std::vector<std::string> antennas;
    std::vector<SoapyGain> gains;
    RangeList sampleRateRanges;
    double fullScale;
    uint16_t magicConstant;
    Stream *stream;
 //   cs16_t *copyOutBuffers;
   // iq_convert_fn converter;
   // soapy_converter_state converter_state;
} Soapy;


    
   /* int fd;
    unsigned bytes_per_sample;
    void *readbuf;
    iq_convert_fn converter;
    struct converter_state *converter_state;*/

//
// =============================== RTLSDR handling ==========================
//

void soapyInitConfig()
{
    Soapy.dev = nullptr;
    Soapy.ppm_error = 0;
    Soapy.stream = nullptr;
  //  Soapy.copyOutBuffers = nullptr;
}

static void show_devices()
{
    std::cout << Device::enumerate() << std::endl;
}

static int find_device_index(char *s)
{
    return 0;
}

extern "C" void soapyShowHelp()
{
    printf("      soapysdr-specific options (use with --device-type soapysdr)\n");
    printf("\n");
    printf("--device <index|serial>  select device by index or serial number\n");
    printf("--ppm <correction>       set oscillator frequency correction in PPM\n");
    printf("--antenna <antenna>       set oscillator frequency correction in PPM\n");
    printf("--lna <lnaGain>       set oscillator frequency correction in PPM\n");
    printf("--tna <tnaGain>       set oscillator frequency correction in PPM\n");
    printf("--antenna <antenna>       set oscillator frequency correction in PPM\n");
    printf("\n");
}

extern "C" bool soapyHandleOption(int argc, char **argv, int *jptr)
{
    int j = *jptr;
    bool more = (j +1  < argc);
    
    if (!strcmp(argv[j], "--ppm") && more) {
        Soapy.ppm_error = atoi(argv[++j]);
    } else {
        return false;
    }
    
    *jptr = j;
    return true;
}

extern "C" bool soapyOpen(void) {
    KwargsList argsList = Device::enumerate();
    std::cout << argsList << std::endl;
    Soapy.dev = Device::make(argsList[0]);
    Soapy.sampleRateRanges= Soapy.dev->getSampleRateRange(SOAPY_SDR_RX, 0);
    std::cout << "SampleRateRanges: " << Soapy.sampleRateRanges << std::endl;
    Soapy.antennas = Soapy.dev->listAntennas(SOAPY_SDR_RX, 0);
    std::cout << "Antennas: " << Soapy.antennas << std::endl;
    Soapy.dev->setAntenna(SOAPY_SDR_RX, 0, "LNAW");
    std::vector<std::string> gainsStr = Soapy.dev->listGains(SOAPY_SDR_RX, 0);
    std::cout << "Gains: " << gainsStr << std::endl;
    for (std::string &name : gainsStr) {
        Range r = Soapy.dev->getGainRange(SOAPY_SDR_RX, 0, name);
        if (name == "LNA") {
            Soapy.dev->setGain(SOAPY_SDR_RX, 0, name, r.maximum());
        } else if (name == "PGA") {
            Soapy.dev->setGain(SOAPY_SDR_RX, 0, name, 0);
        } else if (name == "TNA") {
            Soapy.dev->setGain(SOAPY_SDR_RX, 0, name, 4);
        }
        
        std::cout << name << ": " << r << " current: ";
        double d = Soapy.dev->getGain(SOAPY_SDR_RX, 0, name);
        std::cout << d << std::endl;
        Soapy.gains.push_back({name, r, d});
    }
    
    std::string fmt = Soapy.dev->getNativeStreamFormat(SOAPY_SDR_RX, 0, Soapy.fullScale);
    uint16_t fullScaleInt = (uint16_t)Soapy.fullScale;
    printf("%f %d\n", Soapy.fullScale, fullScaleInt);
    // i think these are all powers of 2, iirc fullScale is 2048 (2**11) or 4096 (2**12) for the Limesdr-mini
    // so this magic constant is either 2**22 or 2**24 / 2**16, or 2**6 or 2**8
    Soapy.magicConstant = (fullScaleInt * fullScaleInt) / USHRT_MAX;
    printf("magic %d\n", Soapy.magicConstant);
    
    // complex int16 (4 bytes per element)
    Modes.sample_rate = 2.4e6;
    Modes.freq = 1090000000;
    printf("set sample rate %f\n", 2.4e6);
    Soapy.dev->setSampleRate(SOAPY_SDR_RX, 0, Modes.sample_rate);
    printf("set frequency %f\n", (const double)Modes.freq);
    Soapy.dev->setFrequency(SOAPY_SDR_RX, 0, (const double)Modes.freq);
    
    bool hasDcOfsMode = Soapy.dev->getDCOffsetMode(SOAPY_SDR_RX, 0);
    printf(hasDcOfsMode ? "Device has dc offset mode\n" : "Device has no dc offset mode\n");
    if (hasDcOfsMode) {
        Soapy.dev->setDCOffsetMode(SOAPY_SDR_RX, 0, true);
    }
    
    std::cout << Soapy.dev->getStreamArgsInfo(SOAPY_SDR_RX, 0) << std::endl;
    
    printf("call setup\n");
    Soapy.stream = Soapy.dev->setupStream(SOAPY_SDR_RX, "CS16", std::vector<size_t>(), KwargsFromString("skipCal=true"));
    printf("stream %p\n", Soapy.stream);

    return true;
}

class StreamInfo;

struct BufEnt {
    BufEnt(std::vector<cs16_t*> &buffs, size_t h, bool r) : buffs(buffs), handle(h), releaseAfterRead(r) {
        rd = wr = buf = buffs[handle];
    }
    ~BufEnt() {
        if (releaseAfterRead) {
            Soapy.dev->releaseReadBuffer(Soapy.stream, handle);
            buffs[handle] = nullptr;
        }
    }
    void wrote(size_t elems) { wr += elems; }
    size_t elems() const { return wr - rd; }
    void read(size_t elems) { rd += elems; }
    std::vector<cs16_t*> &buffs;
    cs16_t *buf, *rd, *wr;
    size_t handle;
    bool releaseAfterRead;
};

class StreamInfo{
public:
    StreamInfo(size_t nBuffers, size_t nElems) :
        numBuffers(nBuffers),
        mtuNumElements(nElems),
        buffs(nBuffers, nullptr),
        buffsWrIndx(0),
        dropping(0),
        sampleCounter(0)
    {
        
    }
    
    size_t numBuffers;
    size_t mtuNumElements;
    std::vector<cs16_t*> buffs;
    size_t buffsWrIndx;
    int flags;
    long long timeNs;
    std::queue<BufEnt*> readme;
    int dropping;
    uint64_t sampleCounter;
    struct timespec thread_cpu;
};

static void readOnceNoDma(StreamInfo &si) {
    int elems = Soapy.dev->readStream(Soapy.stream, (void*const*)&si.buffs[si.buffsWrIndx], si.mtuNumElements, si.flags, si.timeNs);
    if (elems < 0) {
        printf("readStream returned error %d\n", elems);
    } else {
        //printf("readStream returned elems %d\n", elems);
        BufEnt *bufEnt = new BufEnt(si.buffs, si.buffsWrIndx++, false);
        bufEnt->wrote(elems);
        si.readme.push(bufEnt);
        if (si.buffsWrIndx >= si.buffs.size()) {
            si.buffsWrIndx = 0;
        }
    }
}

static void readOnceDma(StreamInfo &si)
{
    if (si.buffs[si.buffsWrIndx] != nullptr) {
        printf("overflow!!\n");
        return;
    }
    int elems = Soapy.dev->acquireReadBuffer(Soapy.stream, si.buffsWrIndx, (const void**)&si.buffs[si.buffsWrIndx], si.flags, si.timeNs);
    if (elems == 0) return;
    BufEnt *bufEnt = new BufEnt(si.buffs, si.buffsWrIndx++, true);
    bufEnt->wrote(elems);
    si.readme.push(bufEnt);
    if (si.buffsWrIndx == si.numBuffers) {
        si.buffsWrIndx = 0;
    }
}

//go in locked
static void mainLoop(StreamInfo &si)
{
    struct mag_buf *outbuf;
    struct mag_buf *lastbuf;
    unsigned next_free_buffer;
    unsigned free_bufs;
    const unsigned block_duration = 1e9 * si.mtuNumElements / Modes.sample_rate;
    
    pthread_mutex_lock(&Modes.data_mutex);
    next_free_buffer = (Modes.first_free_buffer + 1) % MODES_MAG_BUFFERS;
    outbuf = &Modes.mag_buffers[Modes.first_free_buffer];
    lastbuf = &Modes.mag_buffers[(Modes.first_free_buffer + MODES_MAG_BUFFERS - 1) % MODES_MAG_BUFFERS];
    free_bufs = (Modes.first_filled_buffer - next_free_buffer + MODES_MAG_BUFFERS) % MODES_MAG_BUFFERS;
    pthread_mutex_unlock(&Modes.data_mutex);
    
    start_cpu_timing(&si.thread_cpu);
    
    while (si.readme.empty()) {
        readOnceNoDma(si);
    }
    BufEnt *bufEnt = si.readme.front();
    
    /*if (free_bufs == 0 || (si.dropping && free_bufs < MODES_MAG_BUFFERS/2)) {
        // FIFO is full. Drop this block.
        //printf("dropping\n");
        si.dropping = 1;
        outbuf->dropped += bufEnt->elems();
        si.sampleCounter += bufEnt->elems();
        bufEnt->read(bufEnt->elems());
        delete bufEnt;
        si.readme.pop();
        return;
    }*/
    
    si.dropping = 0;
    
    /*
     // Structure representing one magnitude buffer
     struct mag_buf {
     uint16_t       *data;            // Magnitude data. Starts with Modes.trailing_samples worth of overlap from the previous block
     unsigned        length;          // Number of valid samples _after_ overlap. Total buffer length is buf->length + Modes.trailing_samples.
     uint64_t        sampleTimestamp; // Clock timestamp of the start of this block, 12MHz clock
     struct timespec sysTimestamp;    // Estimated system time at start of block
     uint32_t        dropped;         // Number of dropped samples preceding this buffer
     double          mean_level;      // Mean of normalized (0..1) signal level
     double          mean_power;      // Mean of normalized (0..1) power level
     };*/
    //while (si.readme.size() > 0 && free_bufs > 0) {
    //next_free_buffer = (Modes.first_free_buffer + 1) % MODES_MAG_BUFFERS;
    //outbuf = &Modes.mag_buffers[Modes.first_free_buffer];
    //lastbuf = &Modes.mag_buffers[(Modes.first_free_buffer + MODES_MAG_BUFFERS - 1) % MODES_MAG_BUFFERS];
    //free_bufs = (Modes.first_filled_buffer - next_free_buffer + MODES_MAG_BUFFERS) % MODES_MAG_BUFFERS;
    
    // Compute the sample timestamp and system timestamp for the start of the block
    outbuf->sampleTimestamp = si.sampleCounter * 12e6 / Modes.sample_rate;
    
    // Get the approx system time for the start of this block
    clock_gettime(CLOCK_REALTIME, &outbuf->sysTimestamp);
    outbuf->sysTimestamp.tv_nsec -= block_duration;
    normalize_timespec(&outbuf->sysTimestamp);
    
    // Copy trailing data from last block (or reset if not valid)
    if (outbuf->dropped == 0) {
        memcpy(outbuf->data, lastbuf->data + lastbuf->length, Modes.trailing_samples * sizeof(uint16_t));
    } else {
        memset(outbuf->data, 0, Modes.trailing_samples * sizeof(uint16_t));
    }
    
    // Convert the new data
    //size_t elems = bufEnt->elems();
    //outbuf->length = elems;
    
    //size_t toCopy = std::min<size_t>(elems, outbuf->length);
    size_t toCopy = MODES_MAG_BUF_SAMPLES - Modes.trailing_samples;
    size_t copied = 0;
    uint16_t *to = outbuf->data + Modes.trailing_samples;
    printf("copying %d\n", toCopy);
    while (toCopy-- > 0) {
        // add squared components and convert magnitude scales
        // i think this whole thing can be turned into a bitshift
        // since the magicConstant is a power of two divided by 2**16
        // if it is 2**6 or 2**8 as I suspect, then this can be changed from a division
        // to a (unsigned) shift right 6 or 8 bits
        // extend to 32 bits to do the math and truncate the result to 16bits
        const int32_t I_t = *(bufEnt->rd)[0];
        const int32_t Q_t = *(bufEnt->rd++)[1];
        const uint32_t mag = (I_t * I_t + Q_t * Q_t) >> 6;
        *to++ = uint16_t(mag);/// Soapy.magicConstant;
        ++copied;
        if (bufEnt->rd == bufEnt->wr) {
            si.readme.pop();
            delete bufEnt;
            bufEnt = nullptr;
            while (si.readme.empty()) {
                readOnceNoDma(si);
            }
            bufEnt = si.readme.front();
        }
    }
    si.sampleCounter += copied;
    outbuf->length = MODES_MAG_BUF_SAMPLES;

    pthread_mutex_lock(&Modes.data_mutex);
    Modes.mag_buffers[next_free_buffer].dropped = 0;
    Modes.mag_buffers[next_free_buffer].length = 0;  // just in case
    Modes.first_free_buffer = next_free_buffer;
    
    // accumulate CPU while holding the mutex, and restart measurement
    end_cpu_timing(&si.thread_cpu, &Modes.reader_cpu_accumulator);
    
    //printf("bye\n");
    pthread_cond_signal(&Modes.data_cond);
    pthread_mutex_unlock(&Modes.data_mutex);
}

extern "C" void soapyRun()
{
    printf("run\n");
    if (Soapy.dev == nullptr || Soapy.stream == nullptr) {
        return;
    }
    
    size_t nBuffers = Soapy.dev->getNumDirectAccessBuffers(Soapy.stream);
    size_t mtuNumElements = Soapy.dev->getStreamMTU(Soapy.stream);
    
    StreamInfo si(nBuffers == 0 ? MODES_RTL_BUFFERS : nBuffers, mtuNumElements);
    if (nBuffers == 0) {
        // dma not supported
        printf("Dma not supported, creating buffers\n");
        for (int i = 0; i < MODES_RTL_BUFFERS; ++i) {
            si.buffs[i] = new cs16_t[si.mtuNumElements];
        }
    } else {
        printf("Dma supported, grabbing each buffer\n");
        for (size_t i = 0; i < si.numBuffers; ++i) {
            Soapy.dev->getDirectAccessBufferAddrs(Soapy.stream, i, (void**)&si.buffs[i]);
            Soapy.dev->releaseReadBuffer(Soapy.stream, i);
        }
    }
    
    //Soapy.dev->
    Soapy.dev->activateStream(Soapy.stream);
    
    pthread_mutex_lock(&Modes.data_mutex);
    while (!Modes.exit) {
        pthread_cond_wait(&Modes.data_cond, &Modes.data_mutex);
        pthread_mutex_unlock(&Modes.data_mutex);
        mainLoop(si);
        pthread_mutex_lock(&Modes.data_mutex);
    }
    pthread_mutex_unlock(&Modes.data_mutex);
    
    
    //start_cpu_timing(&rtlsdr_thread_cpu);
    
    //while (!Modes.exit) {
    //    rtlsdr_read_async(RTLSDR.dev, rtlsdrCallback, NULL,
    //                      /* MODES_RTL_BUFFERS */ 4,
    //                      MODES_RTL_BUF_SIZE);
    //}
}

extern "C" void soapyClose()
{
    Soapy.dev->deactivateStream(Soapy.stream);
    Soapy.dev->closeStream(Soapy.stream);
    Soapy.stream = nullptr;
    Device::unmake(Soapy.dev);
    Soapy.dev = nullptr;
}
