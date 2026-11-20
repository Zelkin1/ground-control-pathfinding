// Wall clock for the timing of ticks, with the performance counter of Windows. This file includes
// windows.h, so it does not include sim.h.
#include <windows.h>


// `read_clock_seconds` returns the time of the performance counter, in seconds. Only differences
// of two readings have a meaning.
double read_clock_seconds(void) {
    LARGE_INTEGER frequency;
    LARGE_INTEGER count;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&count);
    return (double)count.QuadPart / (double)frequency.QuadPart;
}
