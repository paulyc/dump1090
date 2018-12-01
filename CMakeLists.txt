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

cmake_minimum_required(VERSION 3.5.1)

project(lib1090)

include_directories(
	/usr/include/libusb-1.0
	/usr/local/include
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -g -DENABLE_BLADERF -DENABLE_RTLSDR -DENABLE_LIMESDR -std=c11 -Wall -Werror")
set(SDR_SOURCES sdr.c sdr_ifile.c sdr_rtlsdr.c sdr_bladerf.c sdr_lime.c)

if(APPLE)
  #for darwin build
  set(COMPAT_SOURCES compat/clock_nanosleep/clock_nanosleep.c)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DMISSING_NANOSLEEP -DCLOCKID_T")
  set(LDFLAGS "-pthread")
elseif(UNIX)
  #for linux build
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_DEFAULT_SOURCE")
  set(COMPAT_SOURCES)
  set(LDFLAGS "-pthread")
#elseif(ANDROID)  ?? is this a thing?
#  add_subdirectory(deps)
else()
  #for ?? build (probably broken)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror")
  set(COMPAT_SOURCES)
  set(LDFLAGS "${LDFLAGS}")
endif()

add_library(
	1090

	SHARED
	
	anet.c
	${COMPAT_SOURCES}
	comm_b.c
	convert.c
	cpr.c
	crc.c
	demod_2400.c
	icao_filter.c
	interactive.c
	lib1090.c
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

	rtlsdr
	bladeRF
	LimeSuite
	m
	ncurses
	usb-1.0

	${LDFLAGS}
)

set_target_properties(
	1090 PROPERTIES
	VERSION 0.2.0
	SOVERSION 0
)

add_executable(dump1090 dump1090-main.c)
target_link_libraries(dump1090 1090)
add_executable(faup1090 faup1090-main.c)
target_link_libraries(faup1090 1090)
add_executable(view1090 view1090-main.c)
target_link_libraries(view1090 1090)

INSTALL(
    TARGETS dump1090 faup1090 view1090 1090
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)

INSTALL(
    FILES lib1090.h
    DESTINATION include
)
