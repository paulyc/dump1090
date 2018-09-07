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

// Default version number, if not overriden by the Makefile
#ifndef MODES_DUMP1090_VERSION
# define MODES_DUMP1090_VERSION     "v1.13-custom"
#endif

#ifndef MODES_DUMP1090_VARIANT
# define MODES_DUMP1090_VARIANT     "dump1090-paulyc"
#endif

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
#endif

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
#endif

#define HAVE_RTL_BIAST 1
#define MODES_DEFAULT_PPM 0
#define MODES_DEFAULT_FREQ 1090000000
#define MODES_DEFAULT_WIDTH 1000
#define MODES_DEFAULT_HEIGHT 700
#define MODES_RTL_BUFFERS 15
#define MODES_RTL_BUF_SIZE (16*16384)
#define MODES_MAG_BUF_SAMPLES (MODES_RTL_BUF_SIZE / 2)
#define MODES_MAG_BUFFERS 12
#define MODES_AUTO_GAIN -100
#define MODES_MAX_GAIN 999999
#define MODES_MSG_SQUELCH_DB 4.0
#define MODES_MSG_ENCODER_ERRS 3
#define MODEAC_MSG_SAMPLES (25 * 2)
#define MODEAC_MSG_BYTES 2
#define MODEAC_MSG_SQUELCH_LEVEL 0x07FF
#define MODES_PREAMBLE_US 8
#define MODES_PREAMBLE_SAMPLES (MODES_PREAMBLE_US * 2)
#define MODES_PREAMBLE_SIZE (MODES_PREAMBLE_SAMPLES * sizeof(uint16_t))
#define MODES_LONG_MSG_BYTES 14
#define MODES_SHORT_MSG_BYTES 7
#define MODES_LONG_MSG_BITS (MODES_LONG_MSG_BYTES * 8)
#define MODES_SHORT_MSG_BITS (MODES_SHORT_MSG_BYTES * 8)
#define MODES_LONG_MSG_SAMPLES (MODES_LONG_MSG_BITS * 2)
#define MODES_SHORT_MSG_SAMPLES (MODES_SHORT_MSG_BITS * 2)
#define MODES_LONG_MSG_SIZE (MODES_LONG_MSG_SAMPLES * sizeof(uint16_t))
#define MODES_SHORT_MSG_SIZE (MODES_SHORT_MSG_SAMPLES * sizeof(uint16_t))
#define MODES_OS_PREAMBLE_SAMPLES (20)
#define MODES_OS_PREAMBLE_SIZE (MODES_OS_PREAMBLE_SAMPLES * sizeof(uint16_t))
#define MODES_OS_LONG_MSG_SAMPLES (268)
#define MODES_OS_SHORT_MSG_SAMPLES (135)
#define MODES_OS_LONG_MSG_SIZE (MODES_LONG_MSG_SAMPLES * sizeof(uint16_t))
#define MODES_OS_SHORT_MSG_SIZE (MODES_SHORT_MSG_SAMPLES * sizeof(uint16_t))
#define MODES_OUT_BUF_SIZE (1500)
#define MODES_OUT_FLUSH_SIZE (MODES_OUT_BUF_SIZE - 256)
#define MODES_OUT_FLUSH_INTERVAL (60000)
#define MODES_USER_LATLON_VALID (1<<0)
#define INVALID_ALTITUDE (-9999)
typedef enum {
    SOURCE_INVALID,
    SOURCE_MODE_AC,
    SOURCE_MLAT,
    SOURCE_MODE_S,
    SOURCE_MODE_S_CHECKED,
    SOURCE_TISB,
    SOURCE_ADSB,
} datasource_t;
typedef enum {
    ADDR_ADSB_ICAO,
    ADDR_ADSB_ICAO_NT,
    ADDR_ADSR_ICAO,
    ADDR_TISB_ICAO,
    ADDR_ADSB_OTHER,
    ADDR_ADSR_OTHER,
    ADDR_TISB_TRACKFILE,
    ADDR_TISB_OTHER,
    ADDR_MODE_A,
    ADDR_UNKNOWN
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
    HEADING_INVALID,
    HEADING_GROUND_TRACK,
    HEADING_TRUE,
    HEADING_MAGNETIC,
    HEADING_MAGNETIC_OR_TRUE,
    HEADING_TRACK_OR_HEADING
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
#define MODES_NON_ICAO_ADDRESS (1<<24)
#define MODES_DEBUG_DEMOD (1<<0)
#define MODES_DEBUG_DEMODERR (1<<1)
#define MODES_DEBUG_BADCRC (1<<2)
#define MODES_DEBUG_GOODCRC (1<<3)
#define MODES_DEBUG_NOPREAMBLE (1<<4)
#define MODES_DEBUG_NET (1<<5)
#define MODES_DEBUG_JS (1<<6)
#define MODES_DEBUG_NOPREAMBLE_LEVEL 25
#define MODES_INTERACTIVE_REFRESH_TIME 250
#define MODES_INTERACTIVE_DISPLAY_TTL 60000
#define MODES_NET_HEARTBEAT_INTERVAL 60000
#define MODES_CLIENT_BUF_SIZE 1024
#define MODES_NET_SNDBUF_SIZE (1024*64)
#define MODES_NET_SNDBUF_MAX (7)
#define HISTORY_SIZE 120
#define HISTORY_INTERVAL 30000
#define MODES_NOTUSED(V) ((void) V)
#define MAX_AMPLITUDE 65535.0
#define MAX_POWER (MAX_AMPLITUDE * MAX_AMPLITUDE)

//#include "util.h"
#define DUMP1090_UTIL_H 
uint64_t mstime(void);
extern uint64_t _messageNow;
static inline uint64_t messageNow() {
    return _messageNow;
}
int64_t receiveclock_ns_elapsed(uint64_t t1, uint64_t t2);
int64_t receiveclock_ms_elapsed(uint64_t t1, uint64_t t2);
struct timespec;
void normalize_timespec(struct timespec *ts);
void start_cpu_timing(struct timespec *start_time);
void end_cpu_timing(const struct timespec *start_time, struct timespec *add_to);

//#include "anet.h"
#define ANET_H 
#define ANET_OK 0
#define ANET_ERR -1
#define ANET_ERR_LEN 256
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

//#include "net_io.h"
#define DUMP1090_NETIO_H 
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
struct net_service {
    struct net_service* next;
    const char *descr;
    int listener_count;
    int *listener_fds;
    int connections;
    struct net_writer *writer;
    const char *read_sep;
    read_mode_t read_mode;
    read_fn read_handler;
};
struct client {
    struct client* next;
    int fd;
    struct net_service *service;
    int buflen;
    char buf[1024 +1];
    int modeac_requested;
};
struct net_writer {
    struct net_service *service;
    void *data;
    int dataUsed;
    uint64_t lastWrite;
    heartbeat_fn send_heartbeat;
};
struct net_service *serviceInit(const char *descr, struct net_writer *writer, heartbeat_fn hb_handler, read_mode_t mode, const char *sep, read_fn read_handler);
struct client *serviceConnect(struct net_service *service, char *addr, int port);
void serviceListen(struct net_service *service, char *bind_addr, char *bind_ports);
struct client *createSocketClient(struct net_service *service, int fd);
struct client *createGenericClient(struct net_service *service, int fd);
struct net_service *makeBeastInputService(void);
struct net_service *makeFatsvOutputService(void);
void sendBeastSettings(struct client *c, const char *settings);
void modesInitNet(void);
void modesQueueOutput(struct modesMessage *mm, struct aircraft *a);
void modesNetPeriodicWork(void);
void writeFATSVHeader();
char *generateAircraftJson(const char *url_path, int *len);
char *generateStatsJson(const char *url_path, int *len);
char *generateReceiverJson(const char *url_path, int *len);
char *generateHistoryJson(const char *url_path, int *len);
void writeJsonToFile(const char *file, char * (*generator) (const char *,int*));

//#include "crc.h"
#define DUMP1090_CRC_H 
#define MODES_MAX_BITERRORS 2
struct errorinfo {
    uint32_t syndrome;
    int errors;
    int8_t bit[2];
};
void modesChecksumInit(int fixBits);
uint32_t modesChecksum(uint8_t *msg, int bitlen);
struct errorinfo *modesChecksumDiagnose(uint32_t syndrome, int bitlen);
void modesChecksumFix(uint8_t *msg, struct errorinfo *info);

//#include "demod_2400.h"
#define DUMP1090_DEMOD_2400_H 
struct mag_buf;
void demodulate2400(struct mag_buf *mag);
void demodulate2400AC(struct mag_buf *mag);

//#include "stats.h"
#define DUMP1090_STATS_H 
struct stats {
    uint64_t start;
    uint64_t end;
    uint32_t demod_preambles;
    uint32_t demod_rejected_bad;
    uint32_t demod_rejected_unknown_icao;
    uint32_t demod_accepted[2 +1];
    uint32_t demod_modeac;
    uint64_t samples_processed;
    uint64_t samples_dropped;
    struct timespec demod_cpu;
    struct timespec reader_cpu;
    struct timespec background_cpu;
    double noise_power_sum;
    uint64_t noise_power_count;
    double signal_power_sum;
    uint64_t signal_power_count;
    double peak_signal_power;
    uint32_t strong_signal_count;
    uint32_t remote_received_modeac;
    uint32_t remote_received_modes;
    uint32_t remote_rejected_bad;
    uint32_t remote_rejected_unknown_icao;
    uint32_t remote_accepted[2 +1];
    uint32_t messages_total;
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
    unsigned int suppressed_altitude_messages;
    unsigned int unique_aircraft;
    unsigned int single_message_aircraft;
#define RANGE_BUCKET_COUNT 76
    uint32_t range_histogram[76];
};
void add_stats(const struct stats *st1, const struct stats *st2, struct stats *target);
void display_stats(struct stats *st);
void reset_stats(struct stats *st);
void add_timespecs(const struct timespec *x, const struct timespec *y, struct timespec *z);

//#include "cpr.h"
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

//#include "icao_filter.h"
#define DUMP1090_ICAO_FILTER_H 
void icaoFilterInit();
void icaoFilterAdd(uint32_t addr);
int icaoFilterTest(uint32_t addr);
uint32_t icaoFilterTestFuzzy(uint32_t partial);
void icaoFilterExpire();

//#include "convert.h"
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

//#include "sdr.h"
#define SDR_H 
void sdrInitConfig();
void sdrShowHelp();
_Bool sdrHandleOption(int argc, char **argv, int *jptr);
_Bool sdrOpen();
void sdrRun();
void sdrClose();
typedef enum {
    SDR_NONE, SDR_IFILE, SDR_RTLSDR, SDR_BLADERF
} sdr_type_t;
struct mag_buf {
    uint16_t *data;
    unsigned length;
    uint64_t sampleTimestamp;
    uint64_t sysTimestamp;
    uint32_t dropped;
    double mean_level;
    double mean_power;
};
struct {
    pthread_t reader_thread;
    pthread_mutex_t data_mutex;
    pthread_cond_t data_cond;
    struct mag_buf mag_buffers[12];
    unsigned first_free_buffer;
    unsigned first_filled_buffer;
    struct timespec reader_cpu_accumulator;
    unsigned trailing_samples;
    double sample_rate;
    uint16_t *log10lut;
    int exit;
    int dc_filter;
    char * dev_name;
    int gain;
    int freq;
    int ppm_error;
    int enable_rtlsdr_biast;
    char aneterr[256];
    struct net_service *services;
    struct client *clients;
    struct net_writer raw_out;
    struct net_writer beast_out;
    struct net_writer sbs_out;
    struct net_writer fatsv_out;
    sdr_type_t sdr_type;
    int nfix_crc;
    int check_crc;
    int raw;
    int mode_ac;
    int mode_ac_auto;
    int debug;
    int net;
    int net_only;
    uint64_t net_heartbeat_interval;
    int net_output_flush_size;
    uint64_t net_output_flush_interval;
    char *net_output_raw_ports;
    char *net_input_raw_ports;
    char *net_output_sbs_ports;
    char *net_input_beast_ports;
    char *net_output_beast_ports;
    char *net_bind_address;
    int net_sndbuf_size;
    int net_verbatim;
    int forward_mlat;
    int quiet;
    uint32_t show_only;
    int interactive;
    uint64_t interactive_display_ttl;
    uint64_t stats;
    int stats_range_histo;
    int onlyaddr;
    int metric;
    int use_gnss;
    int mlat;
    char *json_dir;
    uint64_t json_interval;
    int json_location_accuracy;
    int json_aircraft_history_next;
    struct {
        char *content;
        int clen;
    } json_aircraft_history[120];
    double fUserLat;
    double fUserLon;
    int bUserFlags;
    double maxRange;
    struct aircraft *aircrafts;
    struct stats stats_current;
    struct stats stats_alltime;
    struct stats stats_periodic;
    struct stats stats_1min[15];
    int stats_latest_1min;
    struct stats stats_5min;
    struct stats stats_15min;
} Modes;
struct modesMessage {
    unsigned char msg[14];
    unsigned char verbatim[14];
    int msgbits;
    int msgtype;
    uint32_t crc;
    int correctedbits;
    uint32_t addr;
    addrtype_t addrtype;
    uint64_t timestampMsg;
    uint64_t sysTimestampMsg;
    int remote;
    double signalLevel;
    int score;
    datasource_t source;
    unsigned IID;
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
    unsigned metype;
    unsigned mesub;
    commb_format_t commb_format;
    int altitude_baro;
    altitude_unit_t altitude_baro_unit;
    int altitude_geom;
    altitude_unit_t altitude_geom_unit;
    int geom_delta;
    float heading;
    heading_type_t heading_type;
    float track_rate;
    float roll;
    struct {
        float v0;
        float v2;
        float selected;
    } gs;
    unsigned ias;
    unsigned tas;
    double mach;
    int baro_rate;
    int geom_rate;
    unsigned squawk;
    char callsign[9];
    unsigned category;
    emergency_t emergency;
    cpr_type_t cpr_type;
    unsigned cpr_lat;
    unsigned cpr_lon;
    unsigned cpr_nucp;
    airground_t airground;
    double decoded_lat;
    double decoded_lon;
    unsigned decoded_nic;
    unsigned decoded_rc;
    struct {
        unsigned nic_a_valid : 1;
        unsigned nic_b_valid : 1;
        unsigned nic_c_valid : 1;
        unsigned nic_baro_valid : 1;
        unsigned nac_p_valid : 1;
        unsigned nac_v_valid : 1;
        unsigned gva_valid : 1;
        unsigned sda_valid : 1;
        unsigned nic_a : 1;
        unsigned nic_b : 1;
        unsigned nic_c : 1;
        unsigned nic_baro : 1;
        unsigned nac_p : 4;
        unsigned nac_v : 3;
        unsigned sil : 2;
        sil_type_t sil_type;
        unsigned gva : 2;
        unsigned sda : 2;
    } accuracy;
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
    struct {
        unsigned heading_valid : 1;
        unsigned fms_altitude_valid : 1;
        unsigned mcp_altitude_valid : 1;
        unsigned qnh_valid : 1;
        unsigned modes_valid : 1;
        float heading;
        heading_type_t heading_type;
        unsigned fms_altitude;
        unsigned mcp_altitude;
        float qnh;
        enum { NAV_ALT_INVALID, NAV_ALT_UNKNOWN, NAV_ALT_AIRCRAFT, NAV_ALT_MCP, NAV_ALT_FMS } altitude_source;
        nav_modes_t modes;
    } nav;
};

//#include "track.h"
#define DUMP1090_TRACK_H 
#define TRACK_AIRCRAFT_TTL 300000
#define TRACK_AIRCRAFT_ONEHIT_TTL 60000
#define TRACK_AIRCRAFT_POSITION_TTL 60000
#define TRACK_MODEAC_MIN_MESSAGES 4
#define RC_UNKNOWN 0
typedef struct {
    uint64_t stale_interval;
    uint64_t expire_interval;
    datasource_t source;
    uint64_t updated;
    uint64_t stale;
    uint64_t expires;
} data_validity;
struct aircraft {
    uint32_t addr;
    addrtype_t addrtype;
    uint64_t seen;
    long messages;
    double signalLevel[8];
    int signalNext;
    data_validity callsign_valid;
    char callsign[9];
    data_validity altitude_baro_valid;
    int altitude_baro;
    data_validity altitude_geom_valid;
    int altitude_geom;
    data_validity geom_delta_valid;
    int geom_delta;
    data_validity gs_valid;
    float gs;
    data_validity ias_valid;
    unsigned ias;
    data_validity tas_valid;
    unsigned tas;
    data_validity mach_valid;
    float mach;
    data_validity track_valid;
    float track;
    data_validity track_rate_valid;
    float track_rate;
    data_validity roll_valid;
    float roll;
    data_validity mag_heading_valid;
    float mag_heading;
    data_validity true_heading_valid;
    float true_heading;
    data_validity baro_rate_valid;
    int baro_rate;
    data_validity geom_rate_valid;
    int geom_rate;
    data_validity squawk_valid;
    unsigned squawk;
    data_validity emergency_valid;
    emergency_t emergency;
    unsigned category;
    data_validity airground_valid;
    airground_t airground;
    data_validity nav_qnh_valid;
    float nav_qnh;
    data_validity nav_altitude_valid;
    unsigned nav_altitude;
    data_validity nav_heading_valid;
    float nav_heading;
    data_validity nav_modes_valid;
    nav_modes_t nav_modes;
    data_validity cpr_odd_valid;
    cpr_type_t cpr_odd_type;
    unsigned cpr_odd_lat;
    unsigned cpr_odd_lon;
    unsigned cpr_odd_nic;
    unsigned cpr_odd_rc;
    data_validity cpr_even_valid;
    cpr_type_t cpr_even_type;
    unsigned cpr_even_lat;
    unsigned cpr_even_lon;
    unsigned cpr_even_nic;
    unsigned cpr_even_rc;
    data_validity position_valid;
    double lat, lon;
    unsigned pos_nic;
    unsigned pos_rc;
    int adsb_version;
    heading_type_t adsb_hrd;
    heading_type_t adsb_tah;
    data_validity nic_a_valid;
    data_validity nic_c_valid;
    data_validity nic_baro_valid;
    data_validity nac_p_valid;
    data_validity nac_v_valid;
    data_validity sil_valid;
    data_validity gva_valid;
    data_validity sda_valid;
    unsigned nic_a : 1;
    unsigned nic_c : 1;
    unsigned nic_baro : 1;
    unsigned nac_p : 4;
    unsigned nac_v : 3;
    unsigned sil : 2;
    sil_type_t sil_type;
    unsigned gva : 2;
    unsigned sda : 2;
    int modeA_hit;
    int modeC_hit;
    int fatsv_emitted_altitude_baro;
    int fatsv_emitted_altitude_geom;
    int fatsv_emitted_baro_rate;
    int fatsv_emitted_geom_rate;
    float fatsv_emitted_track;
    float fatsv_emitted_track_rate;
    float fatsv_emitted_mag_heading;
    float fatsv_emitted_true_heading;
    float fatsv_emitted_roll;
    float fatsv_emitted_gs;
    unsigned fatsv_emitted_ias;
    unsigned fatsv_emitted_tas;
    float fatsv_emitted_mach;
    airground_t fatsv_emitted_airground;
    unsigned fatsv_emitted_nav_altitude;
    float fatsv_emitted_nav_heading;
    nav_modes_t fatsv_emitted_nav_modes;
    float fatsv_emitted_nav_qnh;
    unsigned char fatsv_emitted_bds_10[7];
    unsigned char fatsv_emitted_bds_30[7];
    unsigned char fatsv_emitted_es_status[7];
    unsigned char fatsv_emitted_es_acas_ra[7];
    char fatsv_emitted_callsign[9];
    addrtype_t fatsv_emitted_addrtype;
    int fatsv_emitted_adsb_version;
    unsigned fatsv_emitted_category;
    unsigned fatsv_emitted_squawk;
    unsigned fatsv_emitted_nac_p;
    unsigned fatsv_emitted_nac_v;
    unsigned fatsv_emitted_sil;
    sil_type_t fatsv_emitted_sil_type;
    unsigned fatsv_emitted_nic_baro;
    emergency_t fatsv_emitted_emergency;
    uint64_t fatsv_last_emitted;
    uint64_t fatsv_last_force_emit;
    struct aircraft *next;
    struct modesMessage first_message;
};
extern uint32_t modeAC_count[4096];
extern uint32_t modeAC_match[4096];
extern uint32_t modeAC_age[4096];
static inline int trackDataValid(const data_validity *v)
{
    return (v->source != SOURCE_INVALID && messageNow() < v->expires);
}
static inline int trackDataFresh(const data_validity *v)
{
    return (v->source != SOURCE_INVALID && messageNow() < v->stale);
}
static inline uint64_t trackDataAge(const data_validity *v)
{
    if (v->source == SOURCE_INVALID)
        return ~(uint64_t)0;
    if (v->updated >= messageNow())
        return 0;
    return (messageNow() - v->updated);
}
struct modesMessage;
struct aircraft *trackUpdateFromMessage(struct modesMessage *mm);
void trackPeriodicUpdate();
static inline unsigned modeAToIndex(unsigned modeA)
{
    return (modeA & 0x0007) | ((modeA & 0x0070) >> 1) | ((modeA & 0x0700) >> 2) | ((modeA & 0x7000) >> 3);
}
static inline unsigned indexToModeA(unsigned index)
{
    return (index & 0007) | ((index & 0070) << 1) | ((index & 0700) << 2) | ((index & 07000) << 3);
}

//#include "mode_s.h"
#define MODE_S_H 
int modesMessageLenByType(int type);
int scoreModesMessage(unsigned char *msg, int validbits);
int decodeModesMessage (struct modesMessage *mm, unsigned char *msg);
void displayModesMessage(struct modesMessage *mm);
void useModesMessage (struct modesMessage *mm);
static inline __attribute__((always_inline)) unsigned getbit(unsigned char *data, unsigned bitnum)
{
    unsigned bi = bitnum - 1;
    unsigned by = bi >> 3;
    unsigned mask = 1 << (7 - (bi & 7));
    return (data[by] & mask) != 0;
}
static inline __attribute__((always_inline)) unsigned getbits(unsigned char *data, unsigned firstbit, unsigned lastbit)
{
    unsigned fbi = firstbit - 1;
    unsigned lbi = lastbit - 1;
    unsigned nbi = (lastbit - firstbit + 1);
    unsigned fby = fbi >> 3;
    unsigned lby = lbi >> 3;
    unsigned nby = (lby - fby) + 1;
    unsigned shift = 7 - (lbi & 7);
    unsigned topmask = 0xFF >> (fbi & 7);
    ((void) sizeof ((fbi <= lbi) ? 1 : 0), __extension__ ({ if (fbi <= lbi) ; else __assert_fail ("fbi <= lbi", "mode_s.h", 62, __extension__ __PRETTY_FUNCTION__); }));
    ((void) sizeof ((nbi <= 32) ? 1 : 0), __extension__ ({ if (nbi <= 32) ; else __assert_fail ("nbi <= 32", "mode_s.h", 63, __extension__ __PRETTY_FUNCTION__); }));
    ((void) sizeof ((nby <= 5) ? 1 : 0), __extension__ ({ if (nby <= 5) ; else __assert_fail ("nby <= 5", "mode_s.h", 64, __extension__ __PRETTY_FUNCTION__); }));
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

//#include "comm_b.h"
#define COMM_B_H 
void decodeCommB(struct modesMessage *mm);
int detectModeA (uint16_t *m, struct modesMessage *mm);
void decodeModeAMessage(struct modesMessage *mm, int ModeA);
void modeACInit();
int modeAToModeC (unsigned int modeA);
unsigned modeCToModeA (int modeC);
void interactiveInit(void);
void interactiveShowData(void);
void interactiveCleanup(void);
void log_with_timestamp(const char *format, ...) __attribute__((format (printf, 1, 2) ));
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
void *readerThreadEntryPoint(void *arg);
void snipMode(int level);
void display_total_stats(void);
void backgroundTasks(void);

#endif // DUMP1090_LIB1090_H
