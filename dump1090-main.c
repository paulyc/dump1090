//  dump1090 - ADS-B software radio decoder
// 
//  Copyright (C) 2018 Paul Ciarlo <paul.ciarlo@gmail.com>
// 
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
// 
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
// 
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "dump1090.h"

//
// ================================ Main ====================================
//
static void showHelp(void) {

    printf("-----------------------------------------------------------------------------\n");
    printf("| dump1090 ModeS Receiver     %45s |\n", MODES_DUMP1090_VARIANT " " MODES_DUMP1090_VERSION);
    printf("| build options: %-58s |\n",
           ""
#ifdef ENABLE_RTLSDR
           "ENABLE_RTLSDR "
#endif
#ifdef ENABLE_BLADERF
           "ENABLE_BLADERF "
#endif
#ifdef SC16Q11_TABLE_BITS
    // This is a little silly, but that's how the preprocessor works..
#define _stringize(x) #x
#define stringize(x) _stringize(x)
           "SC16Q11_TABLE_BITS=" stringize(SC16Q11_TABLE_BITS)
#undef stringize
#undef _stringize
#endif
           );
    printf("-----------------------------------------------------------------------------\n");
    printf("\n");

    sdrShowHelp();

    printf(
"      Common options\n"
"\n"
"--gain <db>              Set gain (default: max gain. Use -10 for auto-gain)\n"
"--freq <hz>              Set frequency (default: 1090 Mhz)\n"
"--interactive            Interactive mode refreshing data on screen. Implies --throttle\n"
"--interactive-rows <num> Max number of rows in interactive mode (default: 22)\n"
"--interactive-ttl <sec>  Remove from list if idle for <sec> (default: 60)\n"
"--raw                    Show only messages hex values\n"
"--net                    Enable networking\n"
"--modeac                 Enable decoding of SSR Modes 3/A & 3/C\n"
"--no-modeac-auto         Don't enable Mode A/C if requested by a Beast connection\n"
"--net-only               Enable just networking, no RTL device or file used\n"
"--net-bind-address <ip>  IP address to bind to (default: Any; Use 127.0.0.1 for private)\n"
"--net-ri-port <ports>    TCP raw input listen ports  (default: 30001)\n"
"--net-ro-port <ports>    TCP raw output listen ports (default: 30002)\n"
"--net-sbs-port <ports>   TCP BaseStation output listen ports (default: 30003)\n"
"--net-bi-port <ports>    TCP Beast input listen ports  (default: 30004,30104)\n"
"--net-bo-port <ports>    TCP Beast output listen ports (default: 30005)\n"
"--net-ro-size <size>     TCP output minimum size (default: 0)\n"
"--net-ro-interval <rate> TCP output memory flush rate in seconds (default: 0)\n"
"--net-heartbeat <rate>   TCP heartbeat rate in seconds (default: 60 sec; 0 to disable)\n"
"--net-buffer <n>         TCP buffer size 64Kb * (2^n) (default: n=0, 64Kb)\n"
"--net-verbatim           Do not apply CRC corrections to messages we forward; send unchanged\n"
"--forward-mlat           Allow forwarding of received mlat results to output ports\n"
"--lat <latitude>         Reference/receiver latitude for surface posn (opt)\n"
"--lon <longitude>        Reference/receiver longitude for surface posn (opt)\n"
"--max-range <distance>   Absolute maximum range for position decoding (in nm, default: 300)\n"
"--fix                    Enable single-bits error correction using CRC\n"
"--no-fix                 Disable single-bits error correction using CRC\n"
"--no-crc-check           Disable messages with broken CRC (discouraged)\n"
"--aggressive             More CPU for more messages (two bits fixes, ...)\n"
"--mlat                   display raw messages in Beast ascii mode\n"
"--stats                  Print stats at exit\n"
"--stats-range            Collect/show range histogram\n"
"--stats-every <seconds>  Show and reset stats every <seconds> seconds\n"
"--onlyaddr               Show only ICAO addresses (testing purposes)\n"
"--metric                 Use metric units (meters, km/h, ...)\n"
"--gnss                   Show altitudes as HAE/GNSS (with H suffix) when available\n"
"--snip <level>           Strip IQ file removing samples < level\n"
"--debug <flags>          Debug mode (verbose), see README for details\n"
"--quiet                  Disable output to stdout. Use for daemon applications\n"
"--show-only <addr>       Show only messages from the given ICAO on stdout\n"
"--ppm <error>            Set receiver error in parts per million (default 0)\n"
#ifdef HAVE_RTL_BIAST
"--enable-rtlsdr-biast    Set bias tee supply on (default off)\n"
#endif
"--write-json <dir>       Periodically write json output to <dir> (for serving by a separate webserver)\n"
"--write-json-every <t>   Write json output every t seconds (default 1)\n"
"--json-location-accuracy <n>  Accuracy of receiver location in json metadata: 0=no location, 1=approximate, 2=exact\n"
"--dcfilter               Apply a 1Hz DC filter to input data (requires more CPU)\n"
"--help                   Show this help\n"
"\n"
"Debug mode flags: d = Log frames decoded with errors\n"
"                  D = Log frames decoded with zero errors\n"
"                  c = Log frames with bad CRC\n"
"                  C = Log frames with good CRC\n"
"                  p = Log frames with bad preamble\n"
"                  n = Log network debugging info\n"
"                  j = Log frames to frames.js, loadable by debug.html\n"
    );
}

static void dump1090ParseArgs(int argc, char **argv) {
    for (int j = 1; j < argc; j++) {
        int more = j+1 < argc; // There are more arguments

        if (!strcmp(argv[j],"--freq") && more) {
            Modes.freq = (int) strtoll(argv[++j],NULL,10);
        } else if ( (!strcmp(argv[j], "--device") || !strcmp(argv[j], "--device-index")) && more) {
            Modes.dev_name = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--gain") && more) {
            Modes.gain = (int) (atof(argv[++j])*10); // Gain is in tens of DBs
        } else if (!strcmp(argv[j],"--dcfilter")) {
            Modes.dc_filter = 1;
        } else if (!strcmp(argv[j],"--measure-noise")) {
            // Ignored
        } else if (!strcmp(argv[j],"--fix")) {
            Modes.nfix_crc = MODES_MAX_BITERRORS;
        } else if (!strcmp(argv[j],"--no-fix")) {
            Modes.nfix_crc = 0;
        } else if (!strcmp(argv[j],"--no-crc-check")) {
            Modes.check_crc = 0;
        } else if (!strcmp(argv[j],"--phase-enhance")) {
            // Ignored, always enabled
        } else if (!strcmp(argv[j],"--raw")) {
            Modes.raw = 1;
        } else if (!strcmp(argv[j],"--net")) {
            Modes.net = 1;
        } else if (!strcmp(argv[j],"--modeac")) {
            Modes.mode_ac = 1;
            Modes.mode_ac_auto = 0;
        } else if (!strcmp(argv[j],"--no-modeac-auto")) {
            Modes.mode_ac_auto = 0;
        } else if (!strcmp(argv[j],"--net-beast")) {
            fprintf(stderr, "--net-beast ignored, use --net-bo-port to control where Beast output is generated\n");
        } else if (!strcmp(argv[j],"--net-only")) {
            Modes.net = 1;
            Modes.sdr_type = SDR_NONE;
       } else if (!strcmp(argv[j],"--net-heartbeat") && more) {
            Modes.net_heartbeat_interval = (uint64_t)(1000 * atof(argv[++j]));
       } else if (!strcmp(argv[j],"--net-ro-size") && more) {
            Modes.net_output_flush_size = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-ro-rate") && more) {
            Modes.net_output_flush_interval = 1000 * atoi(argv[++j]) / 15; // backwards compatibility
        } else if (!strcmp(argv[j],"--net-ro-interval") && more) {
            Modes.net_output_flush_interval = (uint64_t)(1000 * atof(argv[++j]));
        } else if (!strcmp(argv[j],"--net-ro-port") && more) {
            free(Modes.net_output_raw_ports);
            Modes.net_output_raw_ports = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--net-ri-port") && more) {
            free(Modes.net_input_raw_ports);
            Modes.net_input_raw_ports = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--net-bo-port") && more) {
            free(Modes.net_output_beast_ports);
            Modes.net_output_beast_ports = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--net-bi-port") && more) {
            free(Modes.net_input_beast_ports);
            Modes.net_input_beast_ports = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--net-bind-address") && more) {
            free(Modes.net_bind_address);
            Modes.net_bind_address = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--net-http-port") && more) {
            if (strcmp(argv[++j], "0")) {
                fprintf(stderr, "warning: --net-http-port not supported in this build, option ignored.\n");
            }
        } else if (!strcmp(argv[j],"--net-sbs-port") && more) {
            free(Modes.net_output_sbs_ports);
            Modes.net_output_sbs_ports = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--net-buffer") && more) {
            Modes.net_sndbuf_size = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-verbatim")) {
            Modes.net_verbatim = 1;
        } else if (!strcmp(argv[j],"--forward-mlat")) {
            Modes.forward_mlat = 1;
        } else if (!strcmp(argv[j],"--onlyaddr")) {
            Modes.onlyaddr = 1;
        } else if (!strcmp(argv[j],"--metric")) {
            Modes.metric = 1;
        } else if (!strcmp(argv[j],"--hae") || !strcmp(argv[j],"--gnss")) {
            Modes.use_gnss = 1;
        } else if (!strcmp(argv[j],"--aggressive")) {
            Modes.nfix_crc = MODES_MAX_BITERRORS;
        } else if (!strcmp(argv[j],"--interactive")) {
            Modes.interactive = 1;
        } else if (!strcmp(argv[j],"--interactive-ttl") && more) {
            Modes.interactive_display_ttl = (uint64_t)(1000 * atof(argv[++j]));
        } else if (!strcmp(argv[j],"--lat") && more) {
            Modes.fUserLat = atof(argv[++j]);
        } else if (!strcmp(argv[j],"--lon") && more) {
            Modes.fUserLon = atof(argv[++j]);
        } else if (!strcmp(argv[j],"--max-range") && more) {
            Modes.maxRange = atof(argv[++j]) * 1852.0; // convert to metres
        } else if (!strcmp(argv[j],"--debug") && more) {
            char *f = argv[++j];
            while(*f) {
                switch(*f) {
                case 'D': Modes.debug |= MODES_DEBUG_DEMOD; break;
                case 'd': Modes.debug |= MODES_DEBUG_DEMODERR; break;
                case 'C': Modes.debug |= MODES_DEBUG_GOODCRC; break;
                case 'c': Modes.debug |= MODES_DEBUG_BADCRC; break;
                case 'p': Modes.debug |= MODES_DEBUG_NOPREAMBLE; break;
                case 'n': Modes.debug |= MODES_DEBUG_NET; break;
                case 'j': Modes.debug |= MODES_DEBUG_JS; break;
                default:
                    fprintf(stderr, "Unknown debugging flag: %c\n", *f);
                    exit(1);
                    break;
                }
                f++;
            }
        } else if (!strcmp(argv[j],"--stats")) {
            if (!Modes.stats)
                Modes.stats = (uint64_t)1 << 60; // "never"
        } else if (!strcmp(argv[j],"--stats-range")) {
            Modes.stats_range_histo = 1;
        } else if (!strcmp(argv[j],"--stats-every") && more) {
            Modes.stats = (uint64_t) (1000 * atof(argv[++j]));
        } else if (!strcmp(argv[j],"--snip") && more) {
            snipMode(atoi(argv[++j]));
            exit(0);
        } else if (!strcmp(argv[j],"--help")) {
            showHelp();
            exit(0);
        } else if (!strcmp(argv[j],"--ppm") && more) {
            Modes.ppm_error = atoi(argv[++j]);
#ifdef HAVE_RTL_BIAST
        } else if (!strcmp(argv[j], "--enable-rtlsdr-biast")) {
            Modes.enable_rtlsdr_biast = 1;
#endif
        } else if (!strcmp(argv[j],"--quiet")) {
            Modes.quiet = 1;
        } else if (!strcmp(argv[j],"--show-only") && more) {
            Modes.show_only = (uint32_t) strtoul(argv[++j], NULL, 16);
        } else if (!strcmp(argv[j],"--mlat")) {
            Modes.mlat = 1;
        } else if (!strcmp(argv[j],"--oversample")) {
            // Ignored
#ifndef _WIN32
        } else if (!strcmp(argv[j], "--write-json") && more) {
            Modes.json_dir = strdup(argv[++j]);
        } else if (!strcmp(argv[j], "--write-json-every") && more) {
            Modes.json_interval = (uint64_t)(1000 * atof(argv[++j]));
            if (Modes.json_interval < 100) // 0.1s
                Modes.json_interval = 100;
        } else if (!strcmp(argv[j], "--json-location-accuracy") && more) {
            Modes.json_location_accuracy = atoi(argv[++j]);
#endif
        } else if (sdrHandleOption(argc, argv, &j)) {
            /* handled */
        } else {
            fprintf(stderr,
                "Unknown or not enough arguments for option '%s'.\n\n",
                argv[j]);
            showHelp();
            exit(1);
        }
    }
}

//
//=========================================================================
//
//
//=========================================================================
//
int main(int argc, char **argv) {
    // Set sane defaults
    modesInitConfig();

    // signal handlers:
    install_signal_handlers(true);

    // Parse the command line options
    dump1090ParseArgs(argc, argv);

#ifdef _WIN32
    // Try to comply with the Copyright license conditions for binary distribution
    if (!Modes.quiet) {showCopyright();}
#endif

    // Initialization
    log_with_timestamp("%s %s starting up.", MODES_DUMP1090_VARIANT, MODES_DUMP1090_VERSION);
    modesInit();

    if (!sdrOpen()) {
        exit(1);
    }

    if (Modes.net) {
        modesInitNet();
    }

    // init stats:
    modesInitStats();

    interactiveInit();

    // If the user specifies --net-only, just run in order to serve network
    // clients without reading data from the RTL device
    if (Modes.sdr_type == SDR_NONE) {
        mainLoopNetOnly();
    } else {
        mainLoopSdr();
    }

    interactiveCleanup();

    // If --stats were given, print statistics
    if (Modes.stats) {
        display_total_stats();
    }

    log_with_timestamp("Normal exit.");

    sdrClose();

#ifndef _WIN32
    pthread_exit(0);
#else
    return (0);
#endif
}
//
//=========================================================================
//
