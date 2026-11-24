// Worker thread pool for the threaded route smoothing set-up (`--setupthreads`). The first
// submitted job starts one worker thread per processor core minus one, and the main thread runs
// jobs too while it waits. Only the main thread submits jobs. Jobs live in a ring of `MAX_JOBS` entries: the counters of submitted, claimed and
// finished jobs only grow, and a job number modulo `MAX_JOBS` gives its entry.
//
// Each job runs with the x87 and SSE control words of the thread that submitted it, because the
// game sets the x87 precision mode and `cosf` of MinGW uses the x87 unit. A job then gives the
// same results on every thread.
//
// The workers stop and are joined, with a wait of at most `STOP_WAIT_MILLISECONDS`, at exit, on a
// console event such as Ctrl+C, and on an unhandled exception. After the stop, every job runs on
// the main thread. This file includes windows.h, so it does not include sim.h.
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <windows.h>
#include <xmmintrin.h>

#define MAX_JOBS 128                       // entries of the job ring, past the threads of a batch
#define MAX_WAKE_COUNT 0x7fffffff          // largest count of the wake semaphore
#define STOP_WAIT_MILLISECONDS 2000        // longest wait for the workers at a stop

typedef void WorkerJob(void *argument);


// A job: the function, its argument, and the control words of the submitting thread.
typedef struct {
    WorkerJob *function;
    void *argument;
    uint16_t x87_control;
    uint32_t sse_control;
} Job;

static Job jobs[MAX_JOBS];
static volatile LONG submitted_count;
static volatile LONG claimed_count;
static volatile LONG finished_count;

// The semaphore wakes one worker per submitted job.
static HANDLE job_semaphore;
static int worker_count;
static bool are_workers_started;
static HANDLE worker_threads[MAXIMUM_WAIT_OBJECTS];

static volatile LONG is_stop_requested;
static LPTOP_LEVEL_EXCEPTION_FILTER previous_crash_filter;


// `read_x87_control` returns the x87 control word of this thread.
static uint16_t read_x87_control(void) {
    uint16_t control;
    __asm__ volatile("fnstcw %0" : "=m"(control));
    return control;
}


// `write_x87_control` sets the x87 control word of this thread.
static void write_x87_control(uint16_t control) {
    __asm__ volatile("fldcw %0" : : "m"(control));
}


// `run_job` runs a job with the control words of its submitting thread, and restores the control
// words of this thread afterward.
static void run_job(const Job *job) {
    uint16_t own_x87_control = read_x87_control();
    uint32_t own_sse_control = _mm_getcsr();
    write_x87_control(job->x87_control);
    _mm_setcsr(job->sse_control);
    job->function(job->argument);
    write_x87_control(own_x87_control);
    _mm_setcsr(own_sse_control);
}


// `run_unclaimed_jobs` claims and runs submitted jobs until none is left. A compare-and-exchange
// on `claimed_count` gives each job to exactly one thread.
static void run_unclaimed_jobs(void) {
    while (true) {
        LONG job_number = claimed_count;
        if (job_number >= submitted_count)
            return;

        // Another thread that claimed this job first makes the exchange fail, and the loop tries
        // the next job.
        LONG seen_number = InterlockedCompareExchange(&claimed_count, job_number + 1, job_number);
        if (seen_number != job_number)
            continue;
        run_job(&jobs[job_number % MAX_JOBS]);
        InterlockedIncrement(&finished_count);
    }
}


// `run_worker` is the loop of a worker thread: it waits for a wake-up and runs the unclaimed jobs,
// until a stop.
static DWORD WINAPI run_worker(LPVOID unused) {
    (void)unused;
    while (true) {
        WaitForSingleObject(job_semaphore, INFINITE);
        if (is_stop_requested)
            return 0;
        run_unclaimed_jobs();
    }
}


static BOOL WINAPI handle_console_event(DWORD event_type);


// `stop_workers` stops the workers once and waits for them, at most `STOP_WAIT_MILLISECONDS`. A
// worker that crashed and runs this function does not wait for itself.
void stop_workers(void) {
    if (InterlockedExchange(&is_stop_requested, 1) != 0)
        return;
    ReleaseSemaphore(job_semaphore, worker_count, NULL);

    HANDLE waited[MAXIMUM_WAIT_OBJECTS];        // threads to wait for
    DWORD waited_count = 0;
    DWORD own_thread_id = GetCurrentThreadId();
    for (int worker_index = 0; worker_index < worker_count; worker_index++) {
        HANDLE thread = worker_threads[worker_index];
        if (GetThreadId(thread) == own_thread_id)
            continue;
        waited[waited_count] = thread;
        waited_count++;
    }
    if (waited_count > 0)
        WaitForMultipleObjects(waited_count, waited, TRUE, STOP_WAIT_MILLISECONDS);

    for (int worker_index = 0; worker_index < worker_count; worker_index++)
        CloseHandle(worker_threads[worker_index]);

    // In the game mod, the stop runs when the core DLL unloads for a reload. The handlers of
    // this DLL then leave, because Windows would otherwise call them in unloaded code.
    if (worker_count > 0) {
        SetConsoleCtrlHandler(handle_console_event, FALSE);
        SetUnhandledExceptionFilter(previous_crash_filter);
    }
}


// `handle_console_event` stops the workers on a console event, and returns false so that the
// default handler of Windows then ends the program.
static BOOL WINAPI handle_console_event(DWORD event_type) {
    (void)event_type;
    stop_workers();
    return FALSE;
}


// `handle_crash` stops the workers on an unhandled exception, and then passes the exception to the
// filter that was set before.
static LONG WINAPI handle_crash(EXCEPTION_POINTERS *exception) {
    stop_workers();
    if (previous_crash_filter == NULL)
        return EXCEPTION_CONTINUE_SEARCH;
    return previous_crash_filter(exception);
}


// `start_workers` starts one worker thread per processor core minus one, and sets the handlers
// that stop them. `worker_threads` holds at most `MAXIMUM_WAIT_OBJECTS` threads, so the machine
// must have at most that many cores plus one. TODO: confirm.
static void start_workers(void) {
    // A machine with one core gets no workers, and the main thread runs every job.
    SYSTEM_INFO system_info;
    GetSystemInfo(&system_info);
    worker_count = (int)system_info.dwNumberOfProcessors - 1;
    job_semaphore = CreateSemaphoreA(NULL, 0, MAX_WAKE_COUNT, NULL);
    for (int worker_index = 0; worker_index < worker_count; worker_index++)
        worker_threads[worker_index] = CreateThread(NULL, 0, run_worker, NULL, 0, NULL);
    are_workers_started = true;

    if (worker_count == 0)
        return;
    atexit(stop_workers);
    SetConsoleCtrlHandler(handle_console_event, TRUE);
    previous_crash_filter = SetUnhandledExceptionFilter(handle_crash);
}


// `submit_job` adds a job, and wakes one worker thread for it. The ring holds `MAX_JOBS` jobs, so
// at most that many may wait at once.
void submit_job(WorkerJob *function, void *argument) {
    if (!are_workers_started)
        start_workers();
    jobs[submitted_count % MAX_JOBS] = (Job){
        .function = function,
        .argument = argument,
        .x87_control = read_x87_control(),
        .sse_control = _mm_getcsr(),
    };
    InterlockedIncrement(&submitted_count);
    if (worker_count > 0)
        ReleaseSemaphore(job_semaphore, 1, NULL);
}


// `wait_for_jobs` runs the unclaimed jobs on the calling thread, and then waits until every
// submitted job has finished. The memory barrier makes the results of the jobs visible.
void wait_for_jobs(void) {
    run_unclaimed_jobs();
    while (finished_count < submitted_count)
        SwitchToThread();
    MemoryBarrier();
}
