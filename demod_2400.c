// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// demod_2400.c: 2.4MHz Mode S demodulator.
//
// Copyright (c) 2014,2015 Oliver Jowett <oliver@mutability.co.uk>
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

#include <assert.h>

#ifdef MODEAC_DEBUG
#include <gd.h>
#endif

// 2.4MHz sampling rate version
//
// When sampling at 2.4MHz we have exactly 6 samples per 5 symbols.
// Each symbol is 500ns wide, each sample is 416.7ns wide
//
// We maintain a phase offset that is expressed in units of 1/5 of a sample i.e. 1/6 of a symbol, 83.333ns
// Each symbol we process advances the phase offset by 6 i.e. 6/5 of a sample, 500ns
//
// The correlation functions below correlate a 1-0 pair of symbols (i.e. manchester encoded 1 bit)
// starting at the given sample, and assuming that the symbol starts at a fixed 0-5 phase offset within
// m[0]. They return a correlation value, generally interpreted as >0 = 1 bit, <0 = 0 bit

// TODO check if there are better (or more balanced) correlation functions to use here

// nb: the correlation functions sum to zero, so we do not need to adjust for the DC offset in the input signal
// (adding any constant value to all of m[0..3] does not change the result)

static inline bool slice_phase0(const mag_data_t *const m) {
    return (5.0 * m[0] - 3.0 * m[1] - 2.0 * m[2]) > 0.0;
}
static inline bool slice_phase1(const mag_data_t *const m) {
    return (4.0 * m[0] - m[1] - 3.0 * m[2]) > 0.0;
}
static inline bool slice_phase2(const mag_data_t *const m) {
    return (3.0 * m[0] + m[1] - 4.0 * m[2]) > 0.0;
}
static inline bool slice_phase3(const mag_data_t *const m) {
    return (2.0 * m[0] + 3.0 * m[1] - 5.0 * m[2]) > 0.0;
}
static inline bool slice_phase4(const mag_data_t *const m) {
    return (m[0] + 5.0 * m[1] - 5.0 * m[2] - m[3]) > 0.0;
}

//
// Given 'mlen' magnitude samples in 'm', sampled at 2.4MHz,
// try to demodulate some Mode S messages.
//
void demodulate2400(struct mag_buf *mag)
{
    static struct modesMessage zeroMessage;
    struct modesMessage mm;
    uint8_t msg1[MODES_LONG_MSG_BYTES], msg2[MODES_LONG_MSG_BYTES], *msg;

    uint8_t *bestmsg;
    int bestscore, bestphase;

    const mag_data_t *const m = mag->data;
    const uint32_t mlen = mag->length;

    internal_float_t sum_signal_power = 0.0;

    msg = msg1;

    for (uint32_t j = 0; j < mlen; j++) {
        const mag_data_t *preamble = &m[j];
        internal_float_t high, base_signal, base_noise;
        int try_phase;
        int msglen;

        // Look for a message starting at around sample 0 with phase offset 3..7

        // Ideal sample values for preambles with different phase
        // Xn is the first data symbol with phase offset N
        //
        // sample#: 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0
        // phase 3: 2/4\0/5\1 0 0 0 0/5\1/3 3\0 0 0 0 0 0 X4
        // phase 4: 1/5\0/4\2 0 0 0 0/4\2 2/4\0 0 0 0 0 0 0 X0
        // phase 5: 0/5\1/3 3\0 0 0 0/3 3\1/5\0 0 0 0 0 0 0 X1
        // phase 6: 0/4\2 2/4\0 0 0 0 2/4\0/5\1 0 0 0 0 0 0 X2
        // phase 7: 0/3 3\1/5\0 0 0 0 1/5\0/4\2 0 0 0 0 0 0 X3
        //

        // quick check: we must have a rising edge 0->1 and a falling edge 12->13
        if (! (preamble[0] < preamble[1] && preamble[12] > preamble[13]) )
           continue;

        if (preamble[1] > preamble[2] &&                                       // 1
            preamble[2] < preamble[3] && preamble[3] > preamble[4] &&          // 3
            preamble[8] < preamble[9] && preamble[9] > preamble[10] &&         // 9
            preamble[10] < preamble[11]) {                                     // 11-12
            // peaks at 1,3,9,11-12: phase 3
            high = (preamble[1] + preamble[3] + preamble[9] + preamble[11] + preamble[12]) / 4;
            base_signal = preamble[1] + preamble[3] + preamble[9];
            base_noise = preamble[5] + preamble[6] + preamble[7];
        } else if (preamble[1] > preamble[2] &&                                // 1
                   preamble[2] < preamble[3] && preamble[3] > preamble[4] &&   // 3
                   preamble[8] < preamble[9] && preamble[9] > preamble[10] &&  // 9
                   preamble[11] < preamble[12]) {                              // 12
            // peaks at 1,3,9,12: phase 4
            high = (preamble[1] + preamble[3] + preamble[9] + preamble[12]) / 4;
            base_signal = preamble[1] + preamble[3] + preamble[9] + preamble[12];
            base_noise = preamble[5] + preamble[6] + preamble[7] + preamble[8];
        } else if (preamble[1] > preamble[2] &&                                // 1
                   preamble[2] < preamble[3] && preamble[4] > preamble[5] &&   // 3-4
                   preamble[8] < preamble[9] && preamble[10] > preamble[11] && // 9-10
                   preamble[11] < preamble[12]) {                              // 12
            // peaks at 1,3-4,9-10,12: phase 5
            high = (preamble[1] + preamble[3] + preamble[4] + preamble[9] + preamble[10] + preamble[12]) / 4;
            base_signal = preamble[1] + preamble[12];
            base_noise = preamble[6] + preamble[7];
        } else if (preamble[1] > preamble[2] &&                                 // 1
                   preamble[3] < preamble[4] && preamble[4] > preamble[5] &&    // 4
                   preamble[9] < preamble[10] && preamble[10] > preamble[11] && // 10
                   preamble[11] < preamble[12]) {                               // 12
            // peaks at 1,4,10,12: phase 6
            high = (preamble[1] + preamble[4] + preamble[10] + preamble[12]) / 4;
            base_signal = preamble[1] + preamble[4] + preamble[10] + preamble[12];
            base_noise = preamble[5] + preamble[6] + preamble[7] + preamble[8];
        } else if (preamble[2] > preamble[3] &&                                 // 1-2
                   preamble[3] < preamble[4] && preamble[4] > preamble[5] &&    // 4
                   preamble[9] < preamble[10] && preamble[10] > preamble[11] && // 10
                   preamble[11] < preamble[12]) {                               // 12
            // peaks at 1-2,4,10,12: phase 7
            high = (preamble[1] + preamble[2] + preamble[4] + preamble[10] + preamble[12]) / 4;
            base_signal = preamble[4] + preamble[10] + preamble[12];
            base_noise = preamble[6] + preamble[7] + preamble[8];
        } else {
            // no suitable peaks
            continue;
        }

        // Check for enough signal
        if (base_signal * 2.0 < 3.0 * base_noise) // about 3.5dB SNR
            continue;

        // Check that the "quiet" bits 6,7,15,16,17 are actually quiet
        if (preamble[5] >= high ||
            preamble[6] >= high ||
            preamble[7] >= high ||
            preamble[8] >= high ||
            preamble[14] >= high ||
            preamble[15] >= high ||
            preamble[16] >= high ||
            preamble[17] >= high ||
            preamble[18] >= high) {
            continue;
        }

        // try all phases
        Modes.stats_current.demod_preambles++;
        bestmsg = NULL; bestscore = -2; bestphase = -1;
        for (try_phase = 4; try_phase <= 8; ++try_phase) {
            const mag_data_t *pPtr;
            int phase, i, score, bytelen;

            // Decode all the next 112 bits, regardless of the actual message
            // size. We'll check the actual message type later

            pPtr = &m[j+19] + (try_phase/5);
            phase = try_phase % 5;

            bytelen = MODES_LONG_MSG_BYTES;
            for (i = 0; i < bytelen; ++i) {
                uint8_t theByte = 0;

                switch (phase) {
                case 0:
                    theByte =
                        (slice_phase0(pPtr) ? 0x80 : 0) |
                        (slice_phase2(pPtr+2) ? 0x40 : 0) |
                        (slice_phase4(pPtr+4) ? 0x20 : 0) |
                        (slice_phase1(pPtr+7) ? 0x10 : 0) |
                        (slice_phase3(pPtr+9) ? 0x08 : 0) |
                        (slice_phase0(pPtr+12) ? 0x04 : 0) |
                        (slice_phase2(pPtr+14) ? 0x02 : 0) |
                        (slice_phase4(pPtr+16) ? 0x01 : 0);


                    phase = 1;
                    pPtr += 19;
                    break;

                case 1:
                    theByte =
                        (slice_phase1(pPtr) ? 0x80 : 0) |
                        (slice_phase3(pPtr+2) ? 0x40 : 0) |
                        (slice_phase0(pPtr+5) ? 0x20 : 0) |
                        (slice_phase2(pPtr+7) ? 0x10 : 0) |
                        (slice_phase4(pPtr+9) ? 0x08 : 0) |
                        (slice_phase1(pPtr+12) ? 0x04 : 0) |
                        (slice_phase3(pPtr+14) ? 0x02 : 0) |
                        (slice_phase0(pPtr+17) ? 0x01 : 0);

                    phase = 2;
                    pPtr += 19;
                    break;

                case 2:
                    theByte =
                        (slice_phase2(pPtr) ? 0x80 : 0) |
                        (slice_phase4(pPtr+2) ? 0x40 : 0) |
                        (slice_phase1(pPtr+5) ? 0x20 : 0) |
                        (slice_phase3(pPtr+7) ? 0x10 : 0) |
                        (slice_phase0(pPtr+10) ? 0x08 : 0) |
                        (slice_phase2(pPtr+12) ? 0x04 : 0) |
                        (slice_phase4(pPtr+14) ? 0x02 : 0) |
                        (slice_phase1(pPtr+17) ? 0x01 : 0);

                    phase = 3;
                    pPtr += 19;
                    break;

                case 3:
                    theByte =
                        (slice_phase3(pPtr) ? 0x80 : 0) |
                        (slice_phase0(pPtr+3) ? 0x40 : 0) |
                        (slice_phase2(pPtr+5) ? 0x20 : 0) |
                        (slice_phase4(pPtr+7) ? 0x10 : 0) |
                        (slice_phase1(pPtr+10) ? 0x08 : 0) |
                        (slice_phase3(pPtr+12) ? 0x04 : 0) |
                        (slice_phase0(pPtr+15) ? 0x02 : 0) |
                        (slice_phase2(pPtr+17) ? 0x01 : 0);

                    phase = 4;
                    pPtr += 19;
                    break;

                case 4:
                    theByte =
                        (slice_phase4(pPtr) ? 0x80 : 0) |
                        (slice_phase1(pPtr+3) ? 0x40 : 0) |
                        (slice_phase3(pPtr+5) ? 0x20 : 0) |
                        (slice_phase0(pPtr+8) ? 0x10 : 0) |
                        (slice_phase2(pPtr+10) ? 0x08 : 0) |
                        (slice_phase4(pPtr+12) ? 0x04 : 0) |
                        (slice_phase1(pPtr+15) ? 0x02 : 0) |
                        (slice_phase3(pPtr+17) ? 0x01 : 0);

                    phase = 0;
                    pPtr += 20;
                    break;
                }

                msg[i] = theByte;
                if (i == 0) {
                    switch (msg[0] >> 3) {
                    case 0: case 4: case 5: case 11:
                        bytelen = MODES_SHORT_MSG_BYTES; break;

                    case 16: case 17: case 18: case 19: case 20: case 21: case 24:
                        break;

                    default:
                        bytelen = 1; // unknown DF, give up immediately
                        break;
                    }
                }
            }

            // Score the mode S message and see if it's any good.
            score = scoreModesMessage(msg, i*8);
            if (score > bestscore) {
                // new high score!
                bestmsg = msg;
                bestscore = score;
                bestphase = try_phase;

                // swap to using the other buffer so we don't clobber our demodulated data
                // (if we find a better result then we'll swap back, but that's OK because
                // we no longer need this copy if we found a better one)
                msg = (msg == msg1) ? msg2 : msg1;
            }
        }

        // Do we have a candidate?
        if (bestscore < 0) {
            if (bestscore == -1)
                Modes.stats_current.demod_rejected_unknown_icao++;
            else
                Modes.stats_current.demod_rejected_bad++;
            continue; // nope.
        }

        msglen = modesMessageLenByType(bestmsg[0] >> 3);

        // Set initial mm structure details
        mm = zeroMessage;

        // For consistency with how the Beast / Radarcape does it,
        // we report the timestamp at the end of bit 56 (even if
        // the frame is a 112-bit frame)
        mm.timestampMsg = mag->sampleTimestamp + j*5 + (8 + 56) * 12 + bestphase;

        // compute message receive time as block-start-time + difference in the 12MHz clock
        mm.sysTimestampMsg = mag->sysTimestamp + receiveclock_ms_elapsed(mag->sampleTimestamp, mm.timestampMsg);

        mm.score = bestscore;

        // Decode the received message
        {
            int result = decodeModesMessage(&mm, bestmsg);
            if (result < 0) {
                if (result == -1)
                    Modes.stats_current.demod_rejected_unknown_icao++;
                else
                    Modes.stats_current.demod_rejected_bad++;
                continue;
            } else {
                Modes.stats_current.demod_accepted[mm.correctedbits]++;
            }
        }

        // measure signal power
        {
            internal_float_t signal_power = 0.0;
            int signal_len = msglen*12/5;

            for (int k = 0; k < signal_len; ++k) {
                const mag_data_t mag = m[j+19+k];
                signal_power += mag * mag;
            }

            mm.signalLevel = signal_power / signal_len;
            Modes.stats_current.signal_power_sum += signal_power;
            Modes.stats_current.signal_power_count += signal_len;
            sum_signal_power += signal_power;

            if (mm.signalLevel > Modes.stats_current.peak_signal_power)
                Modes.stats_current.peak_signal_power = mm.signalLevel;
            if (mm.signalLevel > 0.50119)
                Modes.stats_current.strong_signal_count++; // signal power above -3dBFS
        }

        // Skip over the message:
        // (we actually skip to 8 bits before the end of the message,
        //  because we can often decode two messages that *almost* collide,
        //  where the preamble of the second message clobbered the last
        //  few bits of the first message, but the message bits didn't
        //  overlap)
        j += msglen*12/5;

        // Pass data to the next layer
        useModesMessage(&mm);
    }

    /* update noise power */
    {
        Modes.stats_current.noise_power_sum += (mag->mean_power * mag->length - sum_signal_power);
        Modes.stats_current.noise_power_count += mag->length;
    }
}

#ifdef MODEAC_DEBUG

static int yscale(unsigned signal)
{
    return (int) (299 - 299.0 * signal / 65536.0);
}

static void draw_modeac(uint16_t *m, unsigned modeac, unsigned f1_clock, unsigned noise_threshold, unsigned signal_threshold, unsigned bits, unsigned noisy_bits, unsigned uncertain_bits)
{
    // 25 bits at 87*60MHz
    // use 1 pixel = 30MHz = 1087 pixels

    gdImagePtr im = gdImageCreate(1088, 300);
    int red = gdImageColorAllocate(im, 255, 0, 0);
    int brightgreen = gdImageColorAllocate(im, 0, 255, 0);
    int darkgreen = gdImageColorAllocate(im, 0, 180, 0);
    int blue = gdImageColorAllocate(im, 0, 0, 255);
    int grey = gdImageColorAllocate(im, 200, 200, 200);
    int white = gdImageColorAllocate(im, 255, 255, 255);
    int black = gdImageColorAllocate(im, 0, 0, 0);

    gdImageFilledRectangle(im, 0, 0, 1087, 299, white);

    // draw samples
    for (unsigned pixel = 0; pixel < 1088; ++pixel) {
        int clock_offset = (pixel - 150) * 2;
        int bit = clock_offset / 87;
        int sample = (f1_clock + clock_offset) / 25;
        int bitoffset = clock_offset % 87;
        int color;

        if (sample < 0)
            continue;

        if (clock_offset < 0 || bit >= 20) {
            color = grey;
        } else if (bitoffset < 27 && (uncertain_bits & (1 << (19-bit)))) {
            color = red;
        } else if (bitoffset >= 27 && (noisy_bits & (1 << (19-bit)))) {
            color = red;
        } else if (bitoffset >= 27) {
            color = grey;
        } else if (bits & (1 << (19-bit))) {
            color = brightgreen;
        } else {
            color = darkgreen;
        }

        gdImageLine(im, pixel, 299, pixel, yscale(m[sample]), color);
    }

    // draw bit boundaries
    for (unsigned bit = 0; bit < 20; ++bit) {
        unsigned clock = 87 * bit;
        unsigned pixel0 = clock / 2 + 150;
        unsigned pixel1 = (clock + 27) / 2 + 150;

        gdImageLine(im, pixel0, 0, pixel0, 299, (bit == 0 || bit == 14) ? black : grey);
        gdImageLine(im, pixel1, 0, pixel1, 299, (bit == 0 || bit == 14) ? black : grey);
    }

    // draw thresholds
    gdImageLine(im, 0, yscale(noise_threshold), 1087, yscale(noise_threshold), blue);
    gdImageLine(im, 0, yscale(signal_threshold), 1087, yscale(signal_threshold), blue);

    // save it

    static int file_counter;
    char filename[PATH_MAX];
    sprintf(filename, "modeac_%04X_%04d.png", modeac, ++file_counter);
    fprintf(stderr, "writing %s\n", filename);

    FILE *pngout = fopen(filename, "wb");
    gdImagePng(im, pngout);
    fclose(pngout);
    gdImageDestroy(im);
}

#endif

//////////
////////// MODE A/C
//////////

//static const double virtualClockFrequency = 60e6;
//static const double virtualClockPeriod = 1.0 / virtualClockFrequency;
//static const double bitClockFrequency = 12e6;
//static const double bitClockPeriod = 1.0 / bitClockFrequency;
static const int cyclesPerModeACBitPeriod = 87;
static const int cyclesPerSample = 25;

// Mode A/C bits are 1.45us wide, consisting of 0.45us on and 1.0us off
// We track this in terms of a (virtual) 60MHz clock, which is the lowest common multiple
// of the bit frequency and the 2.4MHz sampling frequency
//
//            0.45us = 27 cycles }
//            1.00us = 60 cycles } one bit period = 1.45us = 87 cycles
//
// one 2.4MHz sample = 25 cycles
void demodulate2400AC(struct mag_buf *mag)
{
    struct modesMessage mm;
    const mag_data_t *const m = mag->data;
    const uint32_t mlen = mag->length;

    memset(&mm, 0, sizeof(mm));

    const internal_float_t noise_stddev = sqrt(mag->mean_power - mag->mean_level * mag->mean_level); // Var(X) = E[(X-E[X])^2] = E[X^2] - (E[X])^2
    const internal_float_t noise_level = mag->mean_power + noise_stddev;
    const internal_float_t noise_level_plus_6dB = noise_level * 2.0;

    for (uint32_t f1_sample = 1; f1_sample < mlen; ++f1_sample) {
        // Mode A/C messages should match this bit sequence:

        // bit #     value
        //   -1       0    quiet zone
        //    0       1    framing pulse (F1)
        //    1      C1
        //    2      A1
        //    3      C2
        //    4      A2
        //    5      C4
        //    6      A4
        //    7       0    quiet zone (X1)
        //    8      B1
        //    9      D1
        //   10      B2
        //   11      D2
        //   12      B4
        //   13      D4
        //   14       1    framing pulse (F2)
        //   15       0    quiet zone (X2)
        //   16       0    quiet zone (X3)
        //   17     SPI
        //   18       0    quiet zone (X4)
        //   19       0    quiet zone (X5)

        // Look for a F1 and F2 pair,
        // with F1 starting at offset f1_sample.

        // the first framing pulse covers 3.5 samples:
        //
        // |----|        |----|
        // | F1 |________| C1 |_
        //
        // | 0 | 1 | 2 | 3 | 4 |
        //
        // and there is some unknown phase offset of the
        // leading edge e.g.:
        //
        //   |----|        |----|
        // __| F1 |________| C1 |_
        //
        // | 0 | 1 | 2 | 3 | 4 |
        //
        // in theory the "on" period can straddle 3 samples
        // but it's not a big deal as at most 4% of the power
        // is in the third sample.

        if (!(m[f1_sample-1] < m[f1_sample+0]))
            continue;      // not a rising edge

        if (m[f1_sample+2] > m[f1_sample+0] || m[f1_sample+2] > m[f1_sample+1])
            continue;      // quiet part of bit wasn't sufficiently quiet

        const internal_float_t f1_level = (m[f1_sample+0] + m[f1_sample+1]) * 0.5;

        if (noise_level_plus_6dB > f1_level) {
            // require 6dB above noise
            continue;
        }

        // estimate initial clock phase based on the amount of power
        // that ended up in the second sample

        const internal_float_t f1a_power = m[f1_sample] * m[f1_sample];
        const internal_float_t f1b_power = m[f1_sample+1] * m[f1_sample+1];
        const internal_float_t fraction = f1b_power / (f1a_power + f1b_power);
        const long f1_clock = lround(cyclesPerSample * (f1_sample + fraction * fraction));

        // same again for F2
        // F2 is 20.3us / 14 bit periods after F1
        const long f2_clock = f1_clock + (cyclesPerModeACBitPeriod * 14L);
        const long f2_sample = f2_clock / cyclesPerSample;
        assert(f2_sample < mlen + MODES_TRAILING_SAMPLES);

        if (!(m[f2_sample-1] < m[f2_sample+0]))
            continue;

        if (m[f2_sample+2] > m[f2_sample+0] || m[f2_sample+2] > m[f2_sample+1])
            continue;      // quiet part of bit wasn't sufficiently quiet

        internal_float_t f2_level = (m[f2_sample+0] + m[f2_sample+1]) * 0.5;

        if (noise_level_plus_6dB > f2_level) {
            // require 6dB above noise
            continue;
        }

        const internal_float_t f1f2_level = (f1_level > f2_level ? f1_level : f2_level);

        const internal_float_t midpoint = sqrt(noise_level * f1f2_level); // geometric mean of the two levels
        const internal_float_t signal_threshold = midpoint * M_SQRT2; // +3dB
        const internal_float_t noise_threshold = midpoint * M_SQRT1_2;  // -3dB

        // Looks like a real signal. Demodulate all the bits.
        long uncertain_bits = 0;
        long noisy_bits = 0;
        long bits = 0;

        for (long bit = 0, clock = f1_clock; bit < 20L; ++bit, clock += cyclesPerModeACBitPeriod) {
            long sample = clock / cyclesPerSample;

            bits <<= 1;
            noisy_bits <<= 1;
            uncertain_bits <<= 1;

            // check for excessive noise in the quiet period
            if (m[sample+2] >= signal_threshold) {
                noisy_bits |= 1;
            }

            // decide if this bit is on or off
            if (m[sample+0] >= signal_threshold || m[sample+1] >= signal_threshold) {
                bits |= 1;
            } else if (m[sample+0] > noise_threshold && m[sample+1] > noise_threshold) {
                /* not certain about this bit */
                uncertain_bits |= 1;
            } else {
                /* this bit is off */
            }
        }

        // framing bits must be on
        if ((bits & 0x80020) != 0x80020) {
            continue;
        }

        // quiet bits must be off
        if ((bits & 0x0101B) != 0) {
            continue;
        }

        if (noisy_bits || uncertain_bits) {
            continue;
        }

        // Convert to the form that we use elsewhere:
        //  00 A4 A2 A1  00 B4 B2 B1  SPI C4 C2 C1  00 D4 D2 D1
        unsigned modeac =
            ((bits & 0x40000) ? 0x0010 : 0) |  // C1
            ((bits & 0x20000) ? 0x1000 : 0) |  // A1
            ((bits & 0x10000) ? 0x0020 : 0) |  // C2
            ((bits & 0x08000) ? 0x2000 : 0) |  // A2
            ((bits & 0x04000) ? 0x0040 : 0) |  // C4
            ((bits & 0x02000) ? 0x4000 : 0) |  // A4
            ((bits & 0x00800) ? 0x0100 : 0) |  // B1
            ((bits & 0x00400) ? 0x0001 : 0) |  // D1
            ((bits & 0x00200) ? 0x0200 : 0) |  // B2
            ((bits & 0x00100) ? 0x0002 : 0) |  // D2
            ((bits & 0x00080) ? 0x0400 : 0) |  // B4
            ((bits & 0x00040) ? 0x0004 : 0) |  // D4
            ((bits & 0x00004) ? 0x0080 : 0);   // SPI

#ifdef MODEAC_DEBUG
        draw_modeac(m, modeac, f1_clock, noise_threshold, signal_threshold, bits, noisy_bits, uncertain_bits);
#endif

        // This message looks good, submit it

        // For consistency with how the Beast / Radarcape does it,
        // we report the timestamp at the second framing pulse (F2)
        mm.timestampMsg = mag->sampleTimestamp + f2_clock / 5;  // 60MHz -> 12MHz

        // compute message receive time as block-start-time + difference in the 12MHz clock
        mm.sysTimestampMsg = mag->sysTimestamp + receiveclock_ms_elapsed(mag->sampleTimestamp, mm.timestampMsg);

        decodeModeAMessage(&mm, modeac);

        // Pass data to the next layer
        useModesMessage(&mm);

        f1_sample += (20*cyclesPerModeACBitPeriod / cyclesPerSample);
        Modes.stats_current.demod_modeac++;
    }
}
