#ifndef CLOCK_GETTIME_H
#define CLOCK_GETTIME_H

#include <time.h>
#ifdef __MACH__
  #include <mach/clock.h>
  #include <mach/mach.h>
  #include <mach/mach_time.h>
  #define CLOCK_ID_T clock_id_t
#else
  #define CLOCK_ID_T clockid_t
#endif

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

static struct mach_task_basic_info __clock_gettime_inf;

int clock_gettime(clockid_t clk_id, struct timespec *tp);

#endif // CLOCK_GETTIME_H
