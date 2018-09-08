// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// faup1090-main.c: main function for faup1090 executable
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

#include "dump1090.h"

//
// ================================ Main ====================================
//
static void showHelp(void) {
    printf(
"-----------------------------------------------------------------------------\n"
"| faup1090 ModeS conversion     %45s |\n"
"-----------------------------------------------------------------------------\n"
"--net-bo-ipaddr <addr>   IP address to connect to for Beast data (default: 127.0.0.1)\n"
"--net-bo-port <port>     Port to connect for Beast data (default: 30005)\n"
"--lat <latitude>         Reference/receiver latitude for surface posn (opt)\n"
"--lon <longitude>        Reference/receiver longitude for surface posn (opt)\n"
"--stdout                 REQUIRED. Write results to stdout.\n"
"--help                   Show this help\n"
"\n",
MODES_DUMP1090_VARIANT " " MODES_DUMP1090_VERSION
    );
}

//
//=========================================================================
//
int main(int argc, char **argv) {
    int j;
    int stdout_option = 0;
    char *bo_connect_ipaddr = "127.0.0.1";
    int bo_connect_port = 30005;
    struct client *c;
    struct net_service *beast_input, *fatsv_output;

    // Set sane defaults
    faupInitConfig();

    // Parse the command line options
    for (j = 1; j < argc; j++) {
        int more = j+1 < argc; // There are more arguments

        if (!strcmp(argv[j],"--net-bo-port") && more) {
            bo_connect_port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-bo-ipaddr") && more) {
            bo_connect_ipaddr = argv[++j];
        } else if (!strcmp(argv[j],"--lat") && more) {
            Modes.fUserLat = atof(argv[++j]);
        } else if (!strcmp(argv[j],"--lon") && more) {
            Modes.fUserLon = atof(argv[++j]);
        } else if (!strcmp(argv[j],"--help")) {
            showHelp();
            exit(0);
        } else if (!strcmp(argv[j],"--stdout")) {
            stdout_option = 1;
        } else {
            fprintf(stderr,
                "Unknown or not enough arguments for option '%s'.\n\n",
                argv[j]);
            showHelp();
            exit(1);
        }
    }

    if (!stdout_option) {
        fprintf(stderr,
                "--stdout is required, output always goes to stdout.\n");
            showHelp();
        exit(1);
    }

    // Initialization
    faupInit();
    modesInitNet();

    // Set up input connection
    beast_input = makeBeastInputService();
    c = serviceConnect(beast_input, bo_connect_ipaddr, bo_connect_port);
    if (!c) {
        fprintf (stderr,
                 "faup1090: failed to connect to %s:%d (is dump1090 running?): %s\n",
                 bo_connect_ipaddr, bo_connect_port, Modes.aneterr);
        exit (1);
    }

    sendBeastSettings(c, "Cdfj"); // Beast binary, no filters, CRC checks on, no mode A/C

    // Set up output connection on stdout
    fatsv_output = makeFatsvOutputService();
    createGenericClient(fatsv_output, STDOUT_FILENO);
    writeFATSVHeader();

    // Run it until we've lost either connection
    while (!Modes.exit && beast_input->connections && fatsv_output->connections) {
        backgroundTasks();
        usleep(100000);
    }

    return 0;
}
//
//=========================================================================
//
