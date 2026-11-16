/* timing.c reads the wall clock, which sim.c uses to measure how long each simulation tick takes.
 * The clock is in its own file because it needs windows.h, whose macros clash with names in
 * sim.h, so this file does not include sim.h. sim.h declares `read_clock_seconds`. */
#include <windows.h>

/* ------------------------------------------------------------------------------------------------
 * `read_clock_seconds` divides the count of the high-resolution performance counter of Windows by
 * the frequency of that counter, which gives seconds.
 * --------------------------------------------------------------------------------------------- */
double read_clock_seconds(void) {
    LARGE_INTEGER frequency;
    LARGE_INTEGER count;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&count);
    return (double)count.QuadPart / (double)frequency.QuadPart;
}
