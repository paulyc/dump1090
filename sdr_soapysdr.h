// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sdr_soapysdr.h: SoapySDR support (header)
//
// Copyright (c) 2016-2017 Oliver Jowett <oliver@mutability.co.uk>
// Copyright (c) 2017 FlightAware LLC
// Copyright (c) 2018 Paul Ciarlo <paul.ciarlo@gmail.com>
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

#ifndef SDR_SOAPY_H
#define SDR_SOAPY_H

#ifdef __cplusplus
extern "C" {
#endif
    
void soapyInitConfig();
void soapyShowHelp();
bool soapyHandleOption(int argc, char **argv, int *jptr);
bool soapyOpen();
void soapyRun();
void soapyClose();
    
#ifdef __cplusplus
}
#endif

#endif

