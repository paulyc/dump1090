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

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -g -std=c11 -Wall -Werror")

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
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
  set(COMPAT_SOURCES)
endif()

link_directories(/usr/local/lib)
include_directories(/usr/local/include lib1090/src)

find_library(usb-1.0 NAMES libusb-1.0 libusb usb-1.0)
find_path(libusb_INCLUDE libusb.h
        HINTS
        /usr/include/libusb-1.0
        /usr/local/include/libusb-1.0
        /usr/include/libusb
        /usr/local/include/libusb
)

set(LIB_INCLUDE_DIRS ${libusb_INCLUDE})
set(LIB_INCLUDE_DIRS "/usr/local/lib ${LIB_INCLUDE_DIRS}")

set(SDR_SOURCES
        sdr.c
        sdr.h
        sdr_ifile.c
        sdr_ifile.h
)
set(SDR_LIBS)

find_library(rtlsdr NAMES librtlsdr rtlsdr)
find_path(rtlsdr_INCLUDE rtl-sdr.h)

if (rtlsdr_FOUND)
    set(LIB_INCLUDE_DIRS "${LIB_INCLUDE_DIRS} ${rtlsdr_INCLUDE}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I${rtlsdr_INCLUDE} -DENABLE_RTLSDR")
    set(SDR_LIBS "${SDR_LIBS} rtlsdr")
    set(SDR_SOURCES "${SDR_SOURCES} sdr_rtlsdr.c sdr_rtlsdr.h")
endif()

find_library(bladeRF NAMES libbladeRF bladeRF)
find_path(bladeRF_INCLUDE libbladeRF.h)

if (bladeRF_FOUND)
    set(LIB_INCLUDE_DIRS "${LIB_INCLUDE_DIRS} ${bladeRF_INCLUDE}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I${bladeRF_INCLUDE} -DENABLE_BLADERF")
    set(SDR_LIBS "${SDR_LIBS} bladeRF")
    set(SDR_SOURCES "${SDR_SOURCES} sdr_bladerf.c sdr_bladerf.h libbladeRF.h")
endif()

find_library(LimeSuite names LimeSuite libLimeSuite)
find_path(LimeSuite_INCLUDE LimeSuite.h
        HINTS
        /usr/include/lime
        /usr/local/include/lime
)

if(LimeSuite_FOUND)
        set(LIB_INCLUDE_DIRS "${LIB_INCLUDE_DIRS} ${LimeSuite_INCLUDE}")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I${LimeSuite_INCLUDE} -DENABLE_LIMESDR")
        set(SDR_LIBS "${SDR_LIBS} LimeSuite")
        set(SDR_SOURCES "${SDR_SOURCES} sdr_lime.c sdr_lime.h LimeSuite.h")
endif()

add_subdirectory(lib1090)
add_subdirectory(dump1090)

INSTALL(
    TARGETS dump1090 faup1090 view1090 1090
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    #FILES lib1090.h
    #DESTINATION include
)
