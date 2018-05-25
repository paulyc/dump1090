cmake_minimum_required(VERSION 3.11)

project(lib1090)

add_library(usb-1.0 SHARED IMPORTED)
set_target_properties(
	usb-1.0

	PROPERTIES IMPORTED_LOCATION

	/usr/lib/libusb-1.0.so.0
)
include_directories( /usr/include/libusb-1.0 )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -g -DENABLE_RTLSDR -std=c11 -D_DEFAULT_SOURCE")
set(SDR_SOURCES sdr.c sdr_ifile.c sdr_rtlsdr.c)

#for linux build

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_DEFAULT_SOURCE -Wall -Werror -Wno-unknown-warning-option -Wno-format-truncation -fPIC -Wl,visibility=default -Wl,export-dynamic")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_DEFAULT_SOURCE -Wall -Werror -Wno-unknown-warning-option -Wno-format-truncation")
#set(CMAKE_LD_FLAGS "${CMAKE_LD_FLAGS} -lrt -Wl,-soname,lib1090.so.0")
set(COMPAT_SOURCES)
set(EXTRA_LIBS "-lm -lncurses -lpthread")

#for darwin build
#set(COMPAT_SOURCES compat/clock_nanosleep/clock_nanosleep.c)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DMISSING_NANOSLEEP -DCLOCKID_T)
#set(CMAKE_LD_FLAGS "${CMAKE_LD_FLAGS}")
#set(EXTRA_LIBS "-lm -lncurses -lpthread")

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
	event_loop.c
	interactive.c
	mode_ac.c
	mode_s.c
	net_io.c
	${SDR_SOURCES}
	stats.c
	track.c
	util.c
)

target_link_libraries(
	1090

	usb-1.0
	rtlsdr
	"-Wl,-soname,lib1090.so.0"
)

add_executable(dump1090 dump1090.c)
target_link_libraries(dump1090 1090 ${EXTRA_LIBS})
add_executable(faup1090 faup1090.c)
target_link_libraries(faup1090 1090 ${EXTRA_LIBS})
add_executable(view1090 view1090.c)
target_link_libraries(view1090 1090 ${EXTRA_LIBS})

