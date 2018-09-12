// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// lib1090.h: main library header
// includes all other dump1090 headers except system headers
// include this to use lib1090 exports
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

#ifndef DUMP1090_LIB1090_H
#define DUMP1090_LIB1090_H

#define __DUMP1090_H

// ============================= Include files ==========================

#ifndef _WIN32
    #include <assert.h>
    #include <stdio.h>
    #include <string.h>
    #include <stdlib.h>
    #include <stdbool.h>
    #include <pthread.h>
    #include <stdint.h>
    #include <errno.h>
    #include <unistd.h>
    #include <math.h>
    #include <sys/time.h>
    #include <signal.h>
    #include <fcntl.h>
    #include <ctype.h>
    #include <sys/stat.h>
    #include <sys/ioctl.h>
    #include <time.h>
    #include <limits.h>
#else
    #include "winstubs.h" //Put everything Windows specific in here
#endif

/*
 * Platform-specific bits
 */

#if defined(__APPLE__)

/*
 * Mach endian conversion
 */
# include <libkern/OSByteOrder.h>
# define bswap_16 OSSwapInt16
# define bswap_32 OSSwapInt32
# define bswap_64 OSSwapInt64
# include <machine/endian.h>
# define le16toh(x) OSSwapLittleToHostInt16(x)
# define le32toh(x) OSSwapLittleToHostInt32(x)

#else

# include <endian.h>

#endif

#ifdef MISSING_NANOSLEEP
#define CLOCK_NANOSLEEP_H

#ifdef _CLOCKID_T_DEFINED_
#define CLOCKID_T
#endif

#ifndef CLOCKID_T
#define CLOCKID_T
typedef enum
{
    CLOCK_REALTIME,
    CLOCK_MONOTONIC,
    CLOCK_PROCESS_CPUTIME_ID,
    CLOCK_THREAD_CPUTIME_ID
} clockid_t;
#endif // ifndef CLOCKID_T


#ifndef TIMER_ABSTIME
#define TIMER_ABSTIME 1
#endif // TIMER_ABSTIME

struct timespec;

int clock_nanosleep (clockid_t id, int flags, const struct timespec *ts,
                     struct timespec *ots);
#endif // MISSING_NANOSLEEP

#ifdef MISSING_GETTIME
#define CLOCK_GETTIME_H

#include <mach/mach_time.h> // Apple-only, but this isn't inclued on other BSDs

#ifdef _CLOCKID_T_DEFINED_
#define CLOCKID_T
#endif

#ifndef CLOCKID_T
#define CLOCKID_T
typedef enum
{
    CLOCK_REALTIME,
    CLOCK_MONOTONIC,
    CLOCK_PROCESS_CPUTIME_ID,
    CLOCK_THREAD_CPUTIME_ID
} clockid_t;
#endif // ifndef CLOCKID_T

struct timespec;

static mach_timebase_info_data_t __clock_gettime_inf;

int clock_gettime(clockid_t clk_id, struct timespec *tp);
#endif // MISSING_GETTIME

// ============================= #defines ===============================

// Default version number, if not overriden by the Makefile
#ifndef MODES_DUMP1090_VERSION
# define MODES_DUMP1090_VERSION     "v1.13-custom"
#endif

#ifndef MODES_DUMP1090_VARIANT
# define MODES_DUMP1090_VARIANT     "dump1090-paulyc"
#endif

#define HAVE_RTL_BIAST             1
#define MODES_DEFAULT_PPM          0
#define MODES_DEFAULT_FREQ         1090000000
#define MODES_DEFAULT_WIDTH        1000
#define MODES_DEFAULT_HEIGHT       700
#define MODES_RTL_BUFFERS          15                         // Number of RTL buffers
#define MODES_RTL_BUF_SIZE         (16*16384)                 // 256k
#define MODES_MAG_BUF_SAMPLES      (MODES_RTL_BUF_SIZE / 2)   // Each sample is 2 bytes
#define MODES_MAG_BUFFERS          12                         // Number of magnitude buffers (should be smaller than RTL_BUFFERS for flowcontrol to work)
#define MODES_AUTO_GAIN            -100                       // Use automatic gain
#define MODES_MAX_GAIN             999999                     // Use max available gain
#define MODES_MSG_SQUELCH_DB       4.0                        // Minimum SNR, in dB
#define MODES_MSG_ENCODER_ERRS     3                          // Maximum number of encoding errors

#define MODEAC_MSG_SAMPLES       (25 * 2)                     // include up to the SPI bit
#define MODEAC_MSG_BYTES          2
#define MODEAC_MSG_SQUELCH_LEVEL  0x07FF                      // Average signal strength limit

#define MODES_PREAMBLE_US        8              // microseconds = bits
#define MODES_PREAMBLE_SAMPLES  (MODES_PREAMBLE_US       * 2)
#define MODES_PREAMBLE_SIZE     (MODES_PREAMBLE_SAMPLES  * sizeof(uint16_t))
#define MODES_LONG_MSG_BYTES     14
#define MODES_SHORT_MSG_BYTES    7
#define MODES_LONG_MSG_BITS     (MODES_LONG_MSG_BYTES    * 8)
#define MODES_SHORT_MSG_BITS    (MODES_SHORT_MSG_BYTES   * 8)
#define MODES_LONG_MSG_SAMPLES  (MODES_LONG_MSG_BITS     * 2)
#define MODES_SHORT_MSG_SAMPLES (MODES_SHORT_MSG_BITS    * 2)
#define MODES_LONG_MSG_SIZE     (MODES_LONG_MSG_SAMPLES  * sizeof(uint16_t))
#define MODES_SHORT_MSG_SIZE    (MODES_SHORT_MSG_SAMPLES * sizeof(uint16_t))

#define MODES_OS_PREAMBLE_SAMPLES  (20)
#define MODES_OS_PREAMBLE_SIZE     (MODES_OS_PREAMBLE_SAMPLES  * sizeof(uint16_t))
#define MODES_OS_LONG_MSG_SAMPLES  (268)
#define MODES_OS_SHORT_MSG_SAMPLES (135)
#define MODES_OS_LONG_MSG_SIZE     (MODES_LONG_MSG_SAMPLES  * sizeof(uint16_t))
#define MODES_OS_SHORT_MSG_SIZE    (MODES_SHORT_MSG_SAMPLES * sizeof(uint16_t))

#define MODES_OUT_BUF_SIZE         (1500)
#define MODES_OUT_FLUSH_SIZE       (MODES_OUT_BUF_SIZE - 256)
#define MODES_OUT_FLUSH_INTERVAL   (60000)

#define MODES_USER_LATLON_VALID (1<<0)

#define INVALID_ALTITUDE (-9999)

/* Where did a bit of data arrive from? In order of increasing priority */
typedef enum {
    SOURCE_INVALID,        /* data is not valid */
    SOURCE_MODE_AC,        /* A/C message */
    SOURCE_MLAT,           /* derived from mlat */
    SOURCE_MODE_S,         /* data from a Mode S message, no full CRC */
    SOURCE_MODE_S_CHECKED, /* data from a Mode S message with full CRC */
    SOURCE_TISB,           /* data from a TIS-B extended squitter message */
    SOURCE_ADSB,           /* data from a ADS-B extended squitter message */
} datasource_t;

/* What sort of address is this and who sent it?
 * (Earlier values are higher priority)
 */
typedef enum {
    ADDR_ADSB_ICAO,       /* Mode S or ADS-B, ICAO address, transponder sourced */
    ADDR_ADSB_ICAO_NT,    /* ADS-B, ICAO address, non-transponder */
    ADDR_ADSR_ICAO,       /* ADS-R, ICAO address */
    ADDR_TISB_ICAO,       /* TIS-B, ICAO address */

    ADDR_ADSB_OTHER,      /* ADS-B, other address format */
    ADDR_ADSR_OTHER,      /* ADS-R, other address format */
    ADDR_TISB_TRACKFILE,  /* TIS-B, Mode A code + track file number */
    ADDR_TISB_OTHER,      /* TIS-B, other address format */

    ADDR_MODE_A,          /* Mode A */

    ADDR_UNKNOWN          /* unknown address format */
} addrtype_t;

typedef enum {
    UNIT_FEET,
    UNIT_METERS
} altitude_unit_t;

typedef enum {
    ALTITUDE_BARO,
    ALTITUDE_GEOM
} altitude_source_t;

typedef enum {
    AG_INVALID,
    AG_GROUND,
    AG_AIRBORNE,
    AG_UNCERTAIN
} airground_t;

typedef enum {
    SIL_INVALID, SIL_UNKNOWN, SIL_PER_SAMPLE, SIL_PER_HOUR
} sil_type_t;

typedef enum {
    CPR_SURFACE, CPR_AIRBORNE, CPR_COARSE
} cpr_type_t;

typedef enum {
    HEADING_INVALID,          // Not set
    HEADING_GROUND_TRACK,     // Direction of track over ground, degrees clockwise from true north
    HEADING_TRUE,             // Heading, degrees clockwise from true north
    HEADING_MAGNETIC,         // Heading, degrees clockwise from magnetic north
    HEADING_MAGNETIC_OR_TRUE, // HEADING_MAGNETIC or HEADING_TRUE depending on the HRD bit in opstatus
    HEADING_TRACK_OR_HEADING  // GROUND_TRACK / MAGNETIC / TRUE depending on the TAH bit in opstatus
} heading_type_t;

typedef enum {
    COMMB_UNKNOWN,
    COMMB_EMPTY_RESPONSE,
    COMMB_DATALINK_CAPS,
    COMMB_GICB_CAPS,
    COMMB_AIRCRAFT_IDENT,
    COMMB_ACAS_RA,
    COMMB_VERTICAL_INTENT,
    COMMB_TRACK_TURN,
    COMMB_HEADING_SPEED
} commb_format_t;

typedef enum {
    NAV_MODE_AUTOPILOT = 1,
    NAV_MODE_VNAV = 2,
    NAV_MODE_ALT_HOLD = 4,
    NAV_MODE_APPROACH = 8,
    NAV_MODE_LNAV = 16,
    NAV_MODE_TCAS = 32
} nav_modes_t;

// Matches encoding of the ES type 28/1 emergency/priority status subfield
typedef enum {
    EMERGENCY_NONE = 0,
    EMERGENCY_GENERAL = 1,
    EMERGENCY_LIFEGUARD = 2,
    EMERGENCY_MINFUEL = 3,
    EMERGENCY_NORDO = 4,
    EMERGENCY_UNLAWFUL = 5,
    EMERGENCY_DOWNED = 6,
    EMERGENCY_RESERVED = 7
} emergency_t;

#define MODES_NON_ICAO_ADDRESS       (1<<24) // Set on addresses to indicate they are not ICAO addresses

#define MODES_DEBUG_DEMOD (1<<0)
#define MODES_DEBUG_DEMODERR (1<<1)
#define MODES_DEBUG_BADCRC (1<<2)
#define MODES_DEBUG_GOODCRC (1<<3)
#define MODES_DEBUG_NOPREAMBLE (1<<4)
#define MODES_DEBUG_NET (1<<5)
#define MODES_DEBUG_JS (1<<6)

// When debug is set to MODES_DEBUG_NOPREAMBLE, the first sample must be
// at least greater than a given level for us to dump the signal.
#define MODES_DEBUG_NOPREAMBLE_LEVEL 25

#define MODES_INTERACTIVE_REFRESH_TIME 250      // Milliseconds
#define MODES_INTERACTIVE_DISPLAY_TTL 60000     // Delete from display after 60 seconds

#define MODES_NET_HEARTBEAT_INTERVAL 60000      // milliseconds

#define MODES_CLIENT_BUF_SIZE  1024
#define MODES_NET_SNDBUF_SIZE (1024*64)
#define MODES_NET_SNDBUF_MAX  (7)

#define HISTORY_SIZE 120
#define HISTORY_INTERVAL 30000

#define MODES_NOTUSED(V) ((void) V)

#define MAX_AMPLITUDE 65535.0
#define MAX_POWER (MAX_AMPLITUDE * MAX_AMPLITUDE)

// Include subheaders after all the #defines are in place
#define DUMP1090_UTIL_H

/* Returns system time in milliseconds */
uint64_t mstime(void);

/* Returns the time for the current message we're dealing with */
extern uint64_t _messageNow;
static inline uint64_t messageNow() {
    return _messageNow;
}

/* Returns the time elapsed, in nanoseconds, from t1 to t2,
 * where t1 and t2 are 12MHz counters.
 */
int64_t receiveclock_ns_elapsed(uint64_t t1, uint64_t t2);

/* Same, in milliseconds */
int64_t receiveclock_ms_elapsed(uint64_t t1, uint64_t t2);

/* Normalize the value in ts so that ts->nsec lies in
 * [0,999999999]
 */
struct timespec;
void normalize_timespec(struct timespec *ts);

/* record current CPU time in start_time */
void start_cpu_timing(struct timespec *start_time);

/* add difference between start_time and the current CPU time to add_to */
void end_cpu_timing(const struct timespec *start_time, struct timespec *add_to);

#define ANET_H

#define ANET_OK 0
#define ANET_ERR -1
#define ANET_ERR_LEN 256

#if defined(__sun)
#define AF_LOCAL AF_UNIX
#endif

int anetTcpConnect(char *err, char *addr, char *service);
int anetTcpNonBlockConnect(char *err, char *addr, char *service);
int anetRead(int fd, char *buf, int count);
int anetTcpServer(char *err, char *service, char *bindaddr, int *fds, int nfds);
int anetTcpAccept(char *err, int serversock);
int anetWrite(int fd, char *buf, int count);
int anetNonBlock(char *err, int fd);
int anetTcpNoDelay(char *err, int fd);
int anetTcpKeepAlive(char *err, int fd);
int anetSetSendBuffer(char *err, int fd, int buffsize);

#define DUMP1090_NETIO_H

// Describes a networking service (group of connections)

struct aircraft;
struct modesMessage;
struct client;
struct net_service;
typedef int (*read_fn)(struct client *, char *);
typedef void (*heartbeat_fn)(struct net_service *);

typedef enum {
    READ_MODE_IGNORE,
    READ_MODE_BEAST,
    READ_MODE_BEAST_COMMAND,
    READ_MODE_ASCII
} read_mode_t;

// Describes one network service (a group of clients with common behaviour)
struct net_service {
    struct net_service* next;
    const char *descr;
    int listener_count;  // number of listeners
    int *listener_fds;   // listening FDs

    int connections;     // number of active clients

    struct net_writer *writer; // shared writer state

    const char *read_sep;      // hander details for input data
    read_mode_t read_mode;
    read_fn read_handler;
};

// Structure used to describe a networking client
struct client {
    struct client*  next;                // Pointer to next client
    int    fd;                           // File descriptor
    struct net_service *service;         // Service this client is part of
    int    buflen;                       // Amount of data on buffer
    char   buf[MODES_CLIENT_BUF_SIZE+1]; // Read buffer
    int    modeac_requested;             // 1 if this Beast output connection has asked for A/C
};

// Common writer state for all output sockets of one type
struct net_writer {
    struct net_service *service; // owning service
    void *data;          // shared write buffer, sized MODES_OUT_BUF_SIZE
    int dataUsed;        // number of bytes of write buffer currently used
    uint64_t lastWrite;  // time of last write to clients
    heartbeat_fn send_heartbeat; // function that queues a heartbeat if needed
};

struct net_service *serviceInit(const char *descr, struct net_writer *writer, heartbeat_fn hb_handler, read_mode_t mode, const char *sep, read_fn read_handler);
struct client *serviceConnect(struct net_service *service, char *addr, int port);
void serviceListen(struct net_service *service, char *bind_addr, char *bind_ports);
struct client *createSocketClient(struct net_service *service, int fd);
struct client *createGenericClient(struct net_service *service, int fd);

// view1090 / faup1090 want to create these themselves:
struct net_service *makeBeastInputService(void);
struct net_service *makeFatsvOutputService(void);

void sendBeastSettings(struct client *c, const char *settings);

void modesInitNet(void);
void modesQueueOutput(struct modesMessage *mm, struct aircraft *a);
void modesNetPeriodicWork(void);

void writeFATSVHeader();

// TODO: move these somewhere else
char *generateAircraftJson(const char *url_path, int *len);
char *generateStatsJson(const char *url_path, int *len);
char *generateReceiverJson(const char *url_path, int *len);
char *generateHistoryJson(const char *url_path, int *len);
void writeJsonToFile(const char *file, char * (*generator) (const char *,int*));

#define DUMP1090_CRC_H

// Global max for fixable bit erros
#define MODES_MAX_BITERRORS 2

struct errorinfo {
    uint32_t syndrome;                 // CRC syndrome
    int      errors;                   // number of errors
    int8_t   bit[MODES_MAX_BITERRORS]; // bit positions to fix (-1 = no bit)
};

void modesChecksumInit(int fixBits);
uint32_t modesChecksum(uint8_t *msg, int bitlen);
struct errorinfo *modesChecksumDiagnose(uint32_t syndrome, int bitlen);
void modesChecksumFix(uint8_t *msg, struct errorinfo *info);

#define DUMP1090_DEMOD_2400_H

struct mag_buf;

void demodulate2400(struct mag_buf *mag);
void demodulate2400AC(struct mag_buf *mag);

#define DUMP1090_STATS_H

struct stats {
    uint64_t start;
    uint64_t end;

    // Mode S demodulator counts:
    uint32_t demod_preambles;
    uint32_t demod_rejected_bad;
    uint32_t demod_rejected_unknown_icao;
    uint32_t demod_accepted[MODES_MAX_BITERRORS+1];

    // Mode A/C demodulator counts:
    uint32_t demod_modeac;

    uint64_t samples_processed;
    uint64_t samples_dropped;

    // timing:
    struct timespec demod_cpu;
    struct timespec reader_cpu;
    struct timespec background_cpu;

    // noise floor:
    double noise_power_sum;
    uint64_t noise_power_count;

    // mean signal power:
    double signal_power_sum;
    uint64_t signal_power_count;

    // peak signal power seen
    double peak_signal_power;

    // number of signals with power > -3dBFS
    uint32_t strong_signal_count;

    // remote messages:
    uint32_t remote_received_modeac;
    uint32_t remote_received_modes;
    uint32_t remote_rejected_bad;
    uint32_t remote_rejected_unknown_icao;
    uint32_t remote_accepted[MODES_MAX_BITERRORS+1];

    // total messages:
    uint32_t messages_total;

    // CPR decoding:
    unsigned int cpr_surface;
    unsigned int cpr_airborne;
    unsigned int cpr_global_ok;
    unsigned int cpr_global_bad;
    unsigned int cpr_global_skipped;
    unsigned int cpr_global_range_checks;
    unsigned int cpr_global_speed_checks;
    unsigned int cpr_local_ok;
    unsigned int cpr_local_skipped;
    unsigned int cpr_local_range_checks;
    unsigned int cpr_local_speed_checks;
    unsigned int cpr_local_aircraft_relative;
    unsigned int cpr_local_receiver_relative;
    unsigned int cpr_filtered;

    // number of altitude messages ignored because
    // we had a recent DF17/18 altitude
    unsigned int suppressed_altitude_messages;

    // aircraft:
    // total "new" aircraft (i.e. not seen in the last 30 or 300s)
    unsigned int unique_aircraft;
    // we saw only a single message
    unsigned int single_message_aircraft;

    // range histogram
#define RANGE_BUCKET_COUNT 76
    uint32_t range_histogram[RANGE_BUCKET_COUNT];
};

void add_stats(const struct stats *st1, const struct stats *st2, struct stats *target);
void display_stats(struct stats *st);
void reset_stats(struct stats *st);

void add_timespecs(const struct timespec *x, const struct timespec *y, struct timespec *z);

#define DUMP1090_CPR_H

int decodeCPRairborne(int even_cprlat, int even_cprlon,
                      int odd_cprlat, int odd_cprlon,
                      int fflag,
                      double *out_lat, double *out_lon);

int decodeCPRsurface(double reflat, double reflon,
                     int even_cprlat, int even_cprlon,
                     int odd_cprlat, int odd_cprlon,
                     int fflag,
                     double *out_lat, double *out_lon);

int decodeCPRrelative(double reflat, double reflon,
                      int cprlat, int cprlon,
                      int fflag, int surface,
                      double *out_lat, double *out_lon);

#define DUMP1090_ICAO_FILTER_H

// Call once:
void icaoFilterInit();

// Add an address to the filter
void icaoFilterAdd(uint32_t addr);

// Test if the given address matches the filter
int icaoFilterTest(uint32_t addr);

// Test if the top 16 bits match any previously added address.
// If they do, returns an arbitrary one of the matched
// addresses. Returns 0 on failure.
uint32_t icaoFilterTestFuzzy(uint32_t partial);

// Call this periodically to allow the filter to expire
// old entries.
void icaoFilterExpire();

#define DUMP1090_CONVERT_H

struct converter_state;
typedef enum { INPUT_UC8=0, INPUT_SC16, INPUT_SC16Q11 } input_format_t;

typedef void (*iq_convert_fn)(void *iq_data,
                              uint16_t *mag_data,
                              unsigned nsamples,
                              struct converter_state *state,
                              double *out_mean_level,
                              double *out_mean_power);

iq_convert_fn init_converter(input_format_t format,
                             double sample_rate,
                             int filter_dc,
                             struct converter_state **out_state);

void cleanup_converter(struct converter_state *state);

#define SDR_H

// Common interface to different SDR inputs.

void sdrInitConfig();
void sdrShowHelp();
bool sdrHandleOption(int argc, char **argv, int *jptr);
bool sdrOpen();
void sdrRun();
void sdrClose();

//======================== structure declarations =========================

typedef enum {
    SDR_NONE, SDR_IFILE, SDR_RTLSDR, SDR_BLADERF
} sdr_type_t;

// Structure representing one magnitude buffer
struct mag_buf {
    uint16_t       *data;            // Magnitude data. Starts with Modes.trailing_samples worth of overlap from the previous block
    unsigned        length;          // Number of valid samples _after_ overlap. Total buffer length is buf->length + Modes.trailing_samples.
    uint64_t        sampleTimestamp; // Clock timestamp of the start of this block, 12MHz clock
    uint64_t        sysTimestamp;    // Estimated system time at start of block
    uint32_t        dropped;         // Number of dropped samples preceding this buffer
    double          mean_level;      // Mean of normalized (0..1) signal level
    double          mean_power;      // Mean of normalized (0..1) power level
};

// Program global state
struct {                             // Internal state
    pthread_t       reader_thread;

    pthread_mutex_t data_mutex;      // Mutex to synchronize buffer access
    pthread_cond_t  data_cond;       // Conditional variable associated

    struct mag_buf  mag_buffers[MODES_MAG_BUFFERS];       // Converted magnitude buffers from RTL or file input
    unsigned        first_free_buffer;                    // Entry in mag_buffers that will next be filled with input.
    unsigned        first_filled_buffer;                  // Entry in mag_buffers that has valid data and will be demodulated next. If equal to next_free_buffer, there is no unprocessed data.
    struct timespec reader_cpu_accumulator;               // CPU time used by the reader thread, copied out and reset by the main thread under the mutex

    unsigned        trailing_samples;                     // extra trailing samples in magnitude buffers
    double          sample_rate;                          // actual sample rate in use (in hz)

    uint16_t       *log10lut;        // Magnitude -> log10 lookup table
    int             exit;            // Exit from the main loop when true

    // Sample conversion
    int            dc_filter;        // should we apply a DC filter?

    // RTLSDR
    char *        dev_name;
    int           gain;
    int           freq;
    int           ppm_error;
#ifdef HAVE_RTL_BIAST
    int           enable_rtlsdr_biast;
#endif

    // Networking
    char           aneterr[ANET_ERR_LEN];
    struct net_service *services;    // Active services
    struct client *clients;          // Our clients

    struct net_writer raw_out;       // Raw output
    struct net_writer beast_out;     // Beast-format output
    struct net_writer sbs_out;       // SBS-format output
    struct net_writer fatsv_out;     // FATSV-format output

#ifdef _WIN32
    WSADATA        wsaData;          // Windows socket initialisation
#endif

    // Configuration
    sdr_type_t sdr_type;             // where are we getting data from?
    int   nfix_crc;                  // Number of crc bit error(s) to correct
    int   check_crc;                 // Only display messages with good CRC
    int   raw;                       // Raw output format
    int   mode_ac;                   // Enable decoding of SSR Modes A & C
    int   mode_ac_auto;              // allow toggling of A/C by Beast commands
    int   debug;                     // Debugging mode
    int   net;                       // Enable networking
    int   net_only;                  // Enable just networking
    uint64_t net_heartbeat_interval; // TCP heartbeat interval (milliseconds)
    int   net_output_flush_size;     // Minimum Size of output data
    uint64_t net_output_flush_interval; // Maximum interval (in milliseconds) between outputwrites
    char *net_output_raw_ports;      // List of raw output TCP ports
    char *net_input_raw_ports;       // List of raw input TCP ports
    char *net_output_sbs_ports;      // List of SBS output TCP ports
    char *net_input_beast_ports;     // List of Beast input TCP ports
    char *net_output_beast_ports;    // List of Beast output TCP ports
    char *net_bind_address;          // Bind address
    int   net_sndbuf_size;           // TCP output buffer size (64Kb * 2^n)
    int   net_verbatim;              // if true, send the original message, not the CRC-corrected one
    int   forward_mlat;              // allow forwarding of mlat messages to output ports
    int   quiet;                     // Suppress stdout
    uint32_t show_only;              // Only show messages from this ICAO
    int   interactive;               // Interactive mode
    uint64_t interactive_display_ttl;// Interactive mode: TTL display
    uint64_t stats;                  // Interval (millis) between stats dumps,
    int   stats_range_histo;         // Collect/show a range histogram?
    int   onlyaddr;                  // Print only ICAO addresses
    int   metric;                    // Use metric units
    int   use_gnss;                  // Use GNSS altitudes with H suffix ("HAE", though it isn't always) when available
    int   mlat;                      // Use Beast ascii format for raw data output, i.e. @...; iso *...;
    char *json_dir;                  // Path to json base directory, or NULL not to write json.
    uint64_t json_interval;          // Interval between rewriting the json aircraft file, in milliseconds; also the advertised map refresh interval
    int   json_location_accuracy;    // Accuracy of location metadata: 0=none, 1=approx, 2=exact

    int   json_aircraft_history_next;
    struct {
        char *content;
        int clen;
    } json_aircraft_history[HISTORY_SIZE];

    // User details
    double fUserLat;                // Users receiver/antenna lat/lon needed for initial surface location
    double fUserLon;                // Users receiver/antenna lat/lon needed for initial surface location
    double fUserAltM;               // Users receiver/antenna altitude in meters
    int    bUserFlags;              // Flags relating to the user details
    double maxRange;                // Absolute maximum decoding range, in *metres*

    // State tracking
    struct aircraft *aircrafts;

    // Statistics
    struct stats stats_current;
    struct stats stats_alltime;
    struct stats stats_periodic;
    struct stats stats_1min[15];
    int stats_latest_1min;
    struct stats stats_5min;
    struct stats stats_15min;
} Modes;

// The struct we use to store information about a decoded message.
struct modesMessage {
    // Generic fields
    unsigned char msg[MODES_LONG_MSG_BYTES];      // Binary message.
    unsigned char verbatim[MODES_LONG_MSG_BYTES]; // Binary message, as originally received before correction
    int           msgbits;                        // Number of bits in message
    int           msgtype;                        // Downlink format #
    uint32_t      crc;                            // Message CRC
    int           correctedbits;                  // No. of bits corrected
    uint32_t      addr;                           // Address Announced
    addrtype_t    addrtype;                       // address format / source
    uint64_t      timestampMsg;                   // Timestamp of the message (12MHz clock)
    uint64_t      sysTimestampMsg;                // Timestamp of the message (system time)
    int           remote;                         // If set this message is from a remote station
    double        signalLevel;                    // RSSI, in the range [0..1], as a fraction of full-scale power
    int           score;                          // Scoring from scoreModesMessage, if used

    datasource_t  source;                         // Characterizes the overall message source

    // Raw data, just extracted directly from the message
    // The names reflect the field names in Annex 4
    unsigned IID; // extracted from CRC of DF11s
    unsigned AA;
    unsigned AC;
    unsigned CA;
    unsigned CC;
    unsigned CF;
    unsigned DR;
    unsigned FS;
    unsigned ID;
    unsigned KE;
    unsigned ND;
    unsigned RI;
    unsigned SL;
    unsigned UM;
    unsigned VS;
    unsigned char MB[7];
    unsigned char MD[10];
    unsigned char ME[7];
    unsigned char MV[7];

    // Decoded data
    unsigned altitude_baro_valid : 1;
    unsigned altitude_geom_valid : 1;
    unsigned track_valid : 1;
    unsigned track_rate_valid : 1;
    unsigned heading_valid : 1;
    unsigned roll_valid : 1;
    unsigned gs_valid : 1;
    unsigned ias_valid : 1;
    unsigned tas_valid : 1;
    unsigned mach_valid : 1;
    unsigned baro_rate_valid : 1;
    unsigned geom_rate_valid : 1;
    unsigned squawk_valid : 1;
    unsigned callsign_valid : 1;
    unsigned cpr_valid : 1;
    unsigned cpr_odd : 1;
    unsigned cpr_decoded : 1;
    unsigned cpr_relative : 1;
    unsigned category_valid : 1;
    unsigned geom_delta_valid : 1;
    unsigned from_mlat : 1;
    unsigned from_tisb : 1;
    unsigned spi_valid : 1;
    unsigned spi : 1;
    unsigned alert_valid : 1;
    unsigned alert : 1;
    unsigned emergency_valid : 1;

    unsigned metype; // DF17/18 ME type
    unsigned mesub;  // DF17/18 ME subtype

    commb_format_t commb_format; // Inferred format of a comm-b message

    // valid if altitude_baro_valid:
    int               altitude_baro;       // Altitude in either feet or meters
    altitude_unit_t   altitude_baro_unit;  // the unit used for altitude

    // valid if altitude_geom_valid:
    int               altitude_geom;       // Altitude in either feet or meters
    altitude_unit_t   altitude_geom_unit;  // the unit used for altitude

    // following fields are valid if the corresponding _valid field is set:
    int      geom_delta;        // Difference between geometric and baro alt
    float    heading;           // ground track or heading, degrees (0-359). Reported directly or computed from from EW and NS velocity
    heading_type_t heading_type;// how to interpret 'track_or_heading'
    float    track_rate;        // Rate of change of track, degrees/second
    float    roll;              // Roll, degrees, negative is left roll
    struct {
        // Groundspeed, kts, reported directly or computed from from EW and NS velocity
        // For surface movement, this has different interpretations for v0 and v2; both
        // fields are populated. The tracking layer will update "gs.selected".
        float v0;
        float v2;
        float selected;
    } gs;
    unsigned ias;               // Indicated airspeed, kts
    unsigned tas;               // True airspeed, kts
    double   mach;              // Mach number
    int      baro_rate;         // Rate of change of barometric altitude, feet/minute
    int      geom_rate;         // Rate of change of geometric (GNSS / INS) altitude, feet/minute
    unsigned squawk;            // 13 bits identity (Squawk), encoded as 4 hex digits
    char     callsign[9];       // 8 chars flight number, NUL-terminated
    unsigned category;          // A0 - D7 encoded as a single hex byte
    emergency_t emergency;      // emergency/priority status

    // valid if cpr_valid
    cpr_type_t cpr_type;       // The encoding type used (surface, airborne, coarse TIS-B)
    unsigned   cpr_lat;        // Non decoded latitude.
    unsigned   cpr_lon;        // Non decoded longitude.
    unsigned   cpr_nucp;       // NUCp/NIC value implied by message type

    airground_t airground;     // air/ground state

    // valid if cpr_decoded:
    double decoded_lat;
    double decoded_lon;
    unsigned decoded_nic;
    unsigned decoded_rc;

    // various integrity/accuracy things
    struct {
        unsigned nic_a_valid : 1;
        unsigned nic_b_valid : 1;
        unsigned nic_c_valid : 1;
        unsigned nic_baro_valid : 1;
        unsigned nac_p_valid : 1;
        unsigned nac_v_valid : 1;
        unsigned gva_valid : 1;
        unsigned sda_valid : 1;

        unsigned nic_a : 1;        // if nic_a_valid
        unsigned nic_b : 1;        // if nic_b_valid
        unsigned nic_c : 1;        // if nic_c_valid
        unsigned nic_baro : 1;     // if nic_baro_valid

        unsigned nac_p : 4;        // if nac_p_valid
        unsigned nac_v : 3;        // if nac_v_valid

        unsigned sil : 2;          // if sil_type != SIL_INVALID
        sil_type_t sil_type;

        unsigned gva : 2;          // if gva_valid

        unsigned sda : 2;          // if sda_valid
    } accuracy;

    // Operational Status
    struct {
        unsigned valid : 1;
        unsigned version : 3;

        unsigned om_acas_ra : 1;
        unsigned om_ident : 1;
        unsigned om_atc : 1;
        unsigned om_saf : 1;

        unsigned cc_acas : 1;
        unsigned cc_cdti : 1;
        unsigned cc_1090_in : 1;
        unsigned cc_arv : 1;
        unsigned cc_ts : 1;
        unsigned cc_tc : 2;
        unsigned cc_uat_in : 1;
        unsigned cc_poa : 1;
        unsigned cc_b2_low : 1;
        unsigned cc_lw_valid : 1;

        heading_type_t tah;
        heading_type_t hrd;

        unsigned cc_lw;
        unsigned cc_antenna_offset;
    } opstatus;

    // combined:
    //   Target State & Status (ADS-B V2 only)
    //   Comm-B BDS4,0 Vertical Intent
    struct {
        unsigned heading_valid : 1;
        unsigned fms_altitude_valid : 1;
        unsigned mcp_altitude_valid : 1;
        unsigned qnh_valid : 1;
        unsigned modes_valid : 1;

        float    heading;       // heading, degrees (0-359) (could be magnetic or true heading; magnetic recommended)
        heading_type_t heading_type;
        unsigned fms_altitude;  // FMS selected altitude
        unsigned mcp_altitude;  // MCP/FCU selected altitude
        float    qnh;           // altimeter setting (QFE or QNH/QNE), millibars

        enum { NAV_ALT_INVALID, NAV_ALT_UNKNOWN, NAV_ALT_AIRCRAFT, NAV_ALT_MCP, NAV_ALT_FMS } altitude_source;

        nav_modes_t modes;
    } nav;
};

// This one needs modesMessage:
#define DUMP1090_TRACK_H

/* Maximum age of tracked aircraft in milliseconds */
#define TRACK_AIRCRAFT_TTL 300000

/* Maximum age of a tracked aircraft with only 1 message received, in milliseconds */
#define TRACK_AIRCRAFT_ONEHIT_TTL 60000

/* Maximum validity of an aircraft position */
#define TRACK_AIRCRAFT_POSITION_TTL 60000

/* Minimum number of repeated Mode A/C replies with a particular Mode A code needed in a
 * 1 second period before accepting that code.
 */
#define TRACK_MODEAC_MIN_MESSAGES 4

/* Special value for Rc unknown */
#define RC_UNKNOWN 0

// data moves through three states:
//  fresh: data is valid. Updates from a less reliable source are not accepted.
//  stale: data is valid. Updates from a less reliable source are accepted.
//  expired: data is not valid.
typedef struct {
    uint64_t stale_interval;  /* how long after an update until the data is stale */
    uint64_t expire_interval; /* how long after an update until the data expires */

    datasource_t source;     /* where the data came from */
    uint64_t updated;        /* when it arrived */
    uint64_t stale;          /* when it goes stale */
    uint64_t expires;        /* when it expires */
} data_validity;

/* Structure used to describe the state of one tracked aircraft */
struct aircraft {
    uint32_t      addr;           // ICAO address
    addrtype_t    addrtype;       // highest priority address type seen for this aircraft

    uint64_t      seen;           // Time (millis) at which the last packet was received
    long          messages;       // Number of Mode S messages received

    double        signalLevel[8]; // Last 8 Signal Amplitudes
    int           signalNext;     // next index of signalLevel to use

    data_validity callsign_valid;
    char          callsign[9];     // Flight number

    data_validity altitude_baro_valid;
    int           altitude_baro;   // Altitude (Baro)

    data_validity altitude_geom_valid;
    int           altitude_geom;   // Altitude (Geometric)

    data_validity geom_delta_valid;
    int           geom_delta;      // Difference between Geometric and Baro altitudes

    data_validity gs_valid;
    float         gs;

    data_validity ias_valid;
    unsigned      ias;

    data_validity tas_valid;
    unsigned      tas;

    data_validity mach_valid;
    float         mach;

    data_validity track_valid;
    float         track;           // Ground track

    data_validity track_rate_valid;
    float         track_rate;      // Rate of change of ground track, degrees/second

    data_validity roll_valid;
    float         roll;            // Roll angle, degrees right

    data_validity mag_heading_valid;
    float         mag_heading;     // Magnetic heading

    data_validity true_heading_valid;
    float         true_heading;    // True heading

    data_validity baro_rate_valid;
    int           baro_rate;      // Vertical rate (barometric)

    data_validity geom_rate_valid;
    int           geom_rate;      // Vertical rate (geometric)

    data_validity squawk_valid;
    unsigned      squawk;         // Squawk

    data_validity emergency_valid;
    emergency_t   emergency;      // Emergency/priority status

    unsigned      category;       // Aircraft category A0 - D7 encoded as a single hex byte. 00 = unset

    data_validity airground_valid;
    airground_t   airground;      // air/ground status

    data_validity nav_qnh_valid;
    float         nav_qnh;        // Altimeter setting (QNH/QFE), millibars

    data_validity nav_altitude_valid;
    unsigned      nav_altitude;    // FMS or FCU selected altitude

    data_validity nav_heading_valid;
    float         nav_heading; // target heading, degrees (0-359)

    data_validity nav_modes_valid;
    nav_modes_t   nav_modes;  // enabled modes (autopilot, vnav, etc)

    data_validity cpr_odd_valid;        // Last seen even CPR message
    cpr_type_t    cpr_odd_type;
    unsigned      cpr_odd_lat;
    unsigned      cpr_odd_lon;
    unsigned      cpr_odd_nic;
    unsigned      cpr_odd_rc;

    data_validity cpr_even_valid;       // Last seen odd CPR message
    cpr_type_t    cpr_even_type;
    unsigned      cpr_even_lat;
    unsigned      cpr_even_lon;
    unsigned      cpr_even_nic;
    unsigned      cpr_even_rc;

    data_validity position_valid;
    double        lat, lon;       // Coordinated obtained from CPR encoded data
    unsigned      pos_nic;        // NIC of last computed position
    unsigned      pos_rc;         // Rc of last computed position

    // data extracted from opstatus etc
    int           adsb_version;   // ADS-B version (from ADS-B operational status); -1 means no ADS-B messages seen
    heading_type_t adsb_hrd;      // Heading Reference Direction setting (from ADS-B operational status)
    heading_type_t adsb_tah;      // Track Angle / Heading setting (from ADS-B operational status)

    data_validity nic_a_valid;
    data_validity nic_c_valid;
    data_validity nic_baro_valid;
    data_validity nac_p_valid;
    data_validity nac_v_valid;
    data_validity sil_valid;
    data_validity gva_valid;
    data_validity sda_valid;

    unsigned      nic_a : 1;      // NIC supplement A from opstatus
    unsigned      nic_c : 1;      // NIC supplement C from opstatus
    unsigned      nic_baro : 1;   // NIC baro supplement from TSS or opstatus
    unsigned      nac_p : 4;      // NACp from TSS or opstatus
    unsigned      nac_v : 3;      // NACv from airborne velocity or opstatus
    unsigned      sil : 2;        // SIL from TSS or opstatus
    sil_type_t    sil_type;       // SIL supplement from TSS or opstatus
    unsigned      gva : 2;        // GVA from opstatus
    unsigned      sda : 2;        // SDA from opstatus

    int           modeA_hit;   // did our squawk match a possible mode A reply in the last check period?
    int           modeC_hit;   // did our altitude match a possible mode C reply in the last check period?

    int           fatsv_emitted_altitude_baro;    // last FA emitted altitude
    int           fatsv_emitted_altitude_geom;    //      -"-         GNSS altitude
    int           fatsv_emitted_baro_rate;        //      -"-         barometric rate
    int           fatsv_emitted_geom_rate;        //      -"-         geometric rate
    float         fatsv_emitted_track;            //      -"-         true track
    float         fatsv_emitted_track_rate;       //      -"-         track rate of change
    float         fatsv_emitted_mag_heading;      //      -"-         magnetic heading
    float         fatsv_emitted_true_heading;     //      -"-         true heading
    float         fatsv_emitted_roll;             //      -"-         roll angle
    float         fatsv_emitted_gs;               //      -"-         groundspeed
    unsigned      fatsv_emitted_ias;              //      -"-         IAS
    unsigned      fatsv_emitted_tas;              //      -"-         TAS
    float         fatsv_emitted_mach;             //      -"-         Mach number
    airground_t   fatsv_emitted_airground;        //      -"-         air/ground state
    unsigned      fatsv_emitted_nav_altitude;     //      -"-         target altitude
    float         fatsv_emitted_nav_heading;      //      -"-         target heading
    nav_modes_t   fatsv_emitted_nav_modes;        //      -"-         enabled navigation modes
    float         fatsv_emitted_nav_qnh;          //      -"-         altimeter setting
    unsigned char fatsv_emitted_bds_10[7];        //      -"-         BDS 1,0 message
    unsigned char fatsv_emitted_bds_30[7];        //      -"-         BDS 3,0 message
    unsigned char fatsv_emitted_es_status[7];     //      -"-         ES operational status message
    unsigned char fatsv_emitted_es_acas_ra[7];    //      -"-         ES ACAS RA report message
    char          fatsv_emitted_callsign[9];      //      -"-         callsign
    addrtype_t    fatsv_emitted_addrtype;         //      -"-         address type (assumed ADSB_ICAO initially)
    int           fatsv_emitted_adsb_version;     //      -"-         ADS-B version (assumed non-ADS-B initially)
    unsigned      fatsv_emitted_category;         //      -"-         ADS-B emitter category (assumed A0 initially)
    unsigned      fatsv_emitted_squawk;           //      -"-         squawk
    unsigned      fatsv_emitted_nac_p;            //      -"-         NACp
    unsigned      fatsv_emitted_nac_v;            //      -"-         NACv
    unsigned      fatsv_emitted_sil;              //      -"-         SIL
    sil_type_t    fatsv_emitted_sil_type;         //      -"-         SIL supplement
    unsigned      fatsv_emitted_nic_baro;         //      -"-         NICbaro
    emergency_t   fatsv_emitted_emergency;        //      -"-         emergency/priority status

    uint64_t      fatsv_last_emitted;             // time (millis) aircraft was last FA emitted
    uint64_t      fatsv_last_force_emit;          // time (millis) we last emitted only-on-change data

    struct aircraft *next;        // Next aircraft in our linked list

    struct modesMessage first_message;  // A copy of the first message we received for this aircraft.
};

/* Mode A/C tracking is done separately, not via the aircraft list,
 * and via a flat array rather than a list since there are only 4k possible values
 * (nb: we ignore the ident/SPI bit when tracking)
 */
extern uint32_t modeAC_count[4096];
extern uint32_t modeAC_match[4096];
extern uint32_t modeAC_age[4096];

/* is this bit of data valid? */
static inline int trackDataValid(const data_validity *v)
{
    return (v->source != SOURCE_INVALID && messageNow() < v->expires);
}

/* is this bit of data fresh? */
static inline int trackDataFresh(const data_validity *v)
{
    return (v->source != SOURCE_INVALID && messageNow() < v->stale);
}

/* what's the age of this data, in milliseconds? */
static inline uint64_t trackDataAge(const data_validity *v)
{
    if (v->source == SOURCE_INVALID)
        return ~(uint64_t)0;
    if (v->updated >= messageNow())
        return 0;
    return (messageNow() - v->updated);
}

/* Update aircraft state from data in the provided mesage.
 * Return the tracked aircraft.
 */
struct modesMessage;
struct aircraft *trackUpdateFromMessage(struct modesMessage *mm);

/* Call periodically */
void trackPeriodicUpdate();

/* Convert from a (hex) mode A value to a 0-4095 index */
static inline unsigned modeAToIndex(unsigned modeA)
{
    return (modeA & 0x0007) | ((modeA & 0x0070) >> 1) | ((modeA & 0x0700) >> 2) | ((modeA & 0x7000) >> 3);
}

/* Convert from a 0-4095 index to a (hex) mode A value */
static inline unsigned indexToModeA(unsigned index)
{
    return (index & 0007) | ((index & 0070) << 1) | ((index & 0700) << 2) | ((index & 07000) << 3);
}

#define MODE_S_H

//
// Functions exported from mode_s.c
//
int modesMessageLenByType(int type);
int scoreModesMessage(unsigned char *msg, int validbits);
int decodeModesMessage (struct modesMessage *mm, unsigned char *msg);
void displayModesMessage(struct modesMessage *mm);
void useModesMessage    (struct modesMessage *mm);

// datafield extraction helpers

// The first bit (MSB of the first byte) is numbered 1, for consistency
// with how the specs number them.

// Extract one bit from a message.
static inline  __attribute__((always_inline)) unsigned getbit(unsigned char *data, unsigned bitnum)
{
    unsigned bi = bitnum - 1;
    unsigned by = bi >> 3;
    unsigned mask = 1 << (7 - (bi & 7));

    return (data[by] & mask) != 0;
}

// Extract some bits (firstbit .. lastbit inclusive) from a message.
static inline  __attribute__((always_inline)) unsigned getbits(unsigned char *data, unsigned firstbit, unsigned lastbit)
{
    unsigned fbi = firstbit - 1;
    unsigned lbi = lastbit - 1;
    unsigned nbi = (lastbit - firstbit + 1);

    unsigned fby = fbi >> 3;
    unsigned lby = lbi >> 3;
    unsigned nby = (lby - fby) + 1;

    unsigned shift = 7 - (lbi & 7);
    unsigned topmask = 0xFF >> (fbi & 7);

    assert (fbi <= lbi);
    assert (nbi <= 32);
    assert (nby <= 5);

    if (nby == 5) {
        return
            ((data[fby] & topmask) << (32 - shift)) |
            (data[fby + 1] << (24 - shift)) |
            (data[fby + 2] << (16 - shift)) |
            (data[fby + 3] << (8 - shift)) |
            (data[fby + 4] >> shift);
    } else if (nby == 4) {
        return
            ((data[fby] & topmask) << (24 - shift)) |
            (data[fby + 1] << (16 - shift)) |
            (data[fby + 2] << (8 - shift)) |
            (data[fby + 3] >> shift);
    } else if (nby == 3) {
        return
            ((data[fby] & topmask) << (16 - shift)) |
            (data[fby + 1] << (8 - shift)) |
            (data[fby + 2] >> shift);
    } else if (nby == 2) {
        return
            ((data[fby] & topmask) << (8 - shift)) |
            (data[fby + 1] >> shift);
    } else if (nby == 1) {
        return
            (data[fby] & topmask) >> shift;
    } else {
        return 0;
    }
}

extern char ais_charset[64];

#define COMM_B_H

void decodeCommB(struct modesMessage *mm);

// ======================== function declarations =========================

#ifdef __cplusplus
extern "C" {
#endif

//
// Functions exported from mode_ac.c
//
int  detectModeA       (uint16_t *m, struct modesMessage *mm);
void decodeModeAMessage(struct modesMessage *mm, int ModeA);
void modeACInit();
int modeAToModeC (unsigned int modeA);
unsigned modeCToModeA (int modeC);

//
// Functions exported from interactive.c
//
void  interactiveInit(void);
void  interactiveShowData(void);
void  interactiveCleanup(void);

void log_with_timestamp(const char *format, ...) __attribute__((format (printf, 1, 2) ));

// Provided by dump1090.c / view1090.c / faup1090.c
void dump1090ReceiverPositionChanged(float lat, float lon, float alt);
void faup1090ReceiverPositionChanged(float lat, float lon, float alt);
void view1090ReceiverPositionChanged(float lat, float lon, float alt);
void receiverPositionChanged(float lat, float lon, float alt);

void faupInitConfig(void);
void faupInit(void);
void faupBackgroundTasks(void);

void view1090InitConfig(void);
void view1090Init(void);

void modesInitConfig(void);
void modesInit(void);
void modesInitStats(void);
void *readerThreadEntryPoint(void *arg);
void snipMode(int level);
void display_total_stats(void);
void mainLoopNetOnly(void);
void mainLoopSdr(void);
void backgroundTasks(void);
void install_signal_handlers(bool reset);

#ifdef __cplusplus
}
#endif

// from lib1090.c
int lib1090Init(float userLat, float userLon, float userAltMeters);
int lib1090RunThread(void *udata);
int lib1090JoinThread(void **retptr);
int lib1090HandleFrame(struct modesMessage *mm, uint8_t *frm, uint64_t timestamp);

#endif // DUMP1090_LIB1090_H
