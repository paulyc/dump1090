# dump1090 - ADS-B software radio decoder
#
# Copyright (C) 2018 Paul Ciarlo <paul.ciarlo@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.11)

project(lib1090)

find_library(usb-1.0 SHARED /usr/local/lib)
find_library(SoapySDR SHARED /usr/local/lib)

set(CMAKE_MODULE_PATH /usr/local ${CMAKE_MODULE_PATH})

include_directories(/usr/local/include)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -g -UENABLE_RTLSDR -DENABLE_SOAPYSDR -UENABLE_BLADERF  -std=c11 -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g")
set(SDR_SOURCES sdr.c sdr_ifile.c  sdr_soapysdr.cpp)

if (CMAKE_SYSTEM_NAME MATCHES "Darwin")
  set(COMPAT_SOURCES compat/clock_nanosleep/clock_nanosleep.c compat/clock_gettime/clock_gettime.c)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DMISSING_GETTIME -DMISSING_NANOSLEEP -DCLOCKID_T")
  set(EXTRA_LIBS "-lm -lncurses -lpthread")
elseif (CMAKE_SYSTEM_NAME MATCHES "Linux")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_DEFAULT_SOURCE -Wall -Werror -Wno-format-truncation")
  set(COMPAT_SOURCES)
  set(EXTRA_LIBS "-lm -lncurses -lpthread")
endif ()

add_library(
	1090

	SHARED
	
	anet.c
	${COMPAT_SOURCES}
	convert.c
	cpr.c
	crc.c
	demod_2400.c
	icao_filter.c
	interactive.c
	mode_ac.c
	mode_s.c
	net_io.c
	${SDR_SOURCES}
	stats.c
	track.c
	util.c
	
	dump1090.c
	faup1090.c
	view1090.c
)

target_link_libraries(
	1090

	usb-1.0
	rtlsdr
	SoapySDR
	${EXTRA_LIBS}
)

if (CMAKE_SYSTEM_NAME MATCHES "Darwin")
	set_property(TARGET 1090 PROPERTY VERSION 0)
elseif (CMAKE_SYSTEM_NAME MATCHES "Linux")
	set_property(TARGET 1090 PROPERTY SOVERSION 0)
	set_property(TARGET 1090 PROPERTY VERSION 0.0.0)
endif ()

add_executable(dump1090 dump1090-main.c)
target_link_libraries(dump1090 1090 ${EXTRA_LIBS})
add_executable(faup1090 faup1090-main.c)
target_link_libraries(faup1090 1090 ${EXTRA_LIBS})
add_executable(view1090 view1090-main.c)
target_link_libraries(view1090 1090 ${EXTRA_LIBS})

INSTALL(
    TARGETS dump1090 faup1090 view1090 1090
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)
