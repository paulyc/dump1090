# dump1090 - ADS-B software radio decoder
#
# Copyright (C) 2019 Paul Ciarlo <paul.ciarlo@gmail.com>
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

project(dump1090)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -Wall -std=gnu++11 -O3")
SET (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb -Wall -std=gnu++11 -D_GLIBCXX_DEBUG -D_GLIBCXX_DEBUG_PEDANTIC")
LIST (APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

# Set default build type
IF( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE "release" )
ENDIF()

link_directories(/usr/local/lib)
include_directories(/usr/local/include)

find_library(usb-1.0 NAMES libusb-1.0 libusb)
find_path(libusb_INCLUDE libusb.h
	HINTS
	/usr/include/libusb-1.0
	/usr/local/include/libusb-1.0
	/usr/include/libusb
	/usr/local/include/libusb
)

set(LIB_INCLUDE_DIRS ${libusb_INCLUDE})

find_library(rtlsdr NAMES librtlsdr)
find_path(rtlsdr_INCLUDE rtl-sdr.h)

find_library(bladeRF NAMES libbladeRF)
find_path(bladeRF_INCLUDE libbladeRF.h)

find_library(LimeSuite names libLimeSuite)
find_path(LimeSuite_INCLUDE LimeSuite.h
	HINTS
	/usr/include/lime
	/usr/local/include/lime
)

set(SDR_SOURCES
	sdr.c
	sdr.h
	sdr_ifile.c
	sdr_ifile.h
)

if(rtlsdr)
	set(LIB_INCLUDE_DIRS "${LIB_INCLUDE_DIRS} ${rtlsdr_INCLUDE}")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DENABLE_RTLSDR")
	set(SDR_LIBS "${SDR_LIBS} rtlsdr")
	set(SDR_SOURCES "${SDR_SOURCES} sdr_rtlsdr.c sdr_rtlsdr.h LimeSuite.h")
endif()

if(bladeRF)
	set(LIB_INCLUDE_DIRS "${LIB_INCLUDE_DIRS} ${bladeRF_INCLUDE}")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DENABLE_BLADERF")
	set(SDR_LIBS "${SDR_LIBS} bladeRF")
	set(SDR_SOURCES "${SDR_SOURCES} sdr_bladerf.c sdr_bladerf.h libbladeRF.h")
endif()

if(LimeSuite)
	set(LIB_INCLUDE_DIRS "${LIB_INCLUDE_DIRS} ${LimeSuite_INCLUDE}")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DENABLE_LIMESDR")
	set(SDR_LIBS "${SDR_LIBS} LimeSuite")
	set(SDR_SOURCES "${SDR_SOURCES} sdr_lime.c sdr_lime.h LimeSuite.h")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -g -std=c11 -Wall -Werror")

SET( DEPS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Deps)
include(${DEPS_DIR}/rtl-sdr/CMakeLists.txt)

# Descend into subdirectories
ADD_SUBDIRECTORY(hp-toa)

set(LIB_SOURCES
	ais_charset.c ais_charset.h
	anet.c anet.h
	comm_b.c comm_b.h
	convert.c convert.h
	cpr.c cpr.h
	crc.c crc.h
	demod_2400.c demod_2400.h
	icao_filter.c icao_filter.h
	interactive.c
	lib1090.c lib1090.h
	mode_ac.c
	mode_s.c mode_s.h
	net_io.c net_io.h
	stats.c stats.h
	track.c track.h
	util.c util.h
	dump1090.c
	faup1090.c
	view1090.c
)

if(APPLE)
  #for darwin build
  set(COMPAT_SOURCES compat/clock_nanosleep/clock_nanosleep.c compat/clock_nanosleep/clock_nanosleep.h)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DMISSING_NANOSLEEP -DCLOCKID_T")
elseif(UNIX)
  #for linux build
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_DEFAULT_SOURCE")
  set(COMPAT_SOURCES)
else()
  #for ?? build (probably broken)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror")
  set(COMPAT_SOURCES)
endif()

add_library(
	1090

	SHARED

	${LIB_SOURCES}
	${COMPAT_SOURCES}
	${SDR_SOURCES}
)

target_link_libraries(
	1090

	${SDR_LIBS}
	m
	rt
	dl
	hp-toa
	ncurses
	usb-1.0
	pthread
)

target_include_directories(1090 PRIVATE ${LIB_INCLUDE_DIRS})

set_target_properties(
	1090 PROPERTIES
	VERSION 0.3.0
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
