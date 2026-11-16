/* workers.c runs jobs on worker threads, so that the route set-ups of several squads (squad.c,
 * `--setupthreads on`) run at the same time on several processor cores.
 *
 * Jobs: The main thread hands a job over with `submit_job`, and a free worker thread starts it at
 * once. `wait_for_jobs` makes the main thread run the jobs that no worker took yet, and then wait
 * until every submitted job has ended.
 *
 * Counts: Three counts only grow: the jobs submitted, the jobs that a thread took (claimed), and
 * the jobs that ended. Job k lies in entry k % `MAX_JOBS` of `jobs`. A thread takes job k by
 * raising the claimed count from k to k + 1, which only one thread can do. Because the counts
 * never go back, a thread that saw an old count can never take a job twice. An entry is used
 * again only after its job ended, because `wait_for_jobs` waits for all jobs, and fewer than
 * `MAX_JOBS` jobs come between two waits.
 *
 * Floating point: Each job runs with the x87 and SSE control words that the main thread had when
 * it submitted the job. Some functions of the C library, such as cosf, compute on the x87 unit,
 * and their results depend on its precision setting, so a job gives the same results on any
 * thread.
 *
 * Stop: The worker threads stop in order when the program ends (`atexit`), on Ctrl+C, Ctrl+Break
 * or a closed console (console control handler), and on a crash (unhandled-exception filter,
 * which then passes the crash on). Each worker first ends the job it runs. The stop waits at most
 * `STOP_WAIT_MILLISECONDS` for the workers, so a stuck job cannot hold up the end of the program.
 * After the stop, `wait_for_jobs` runs every job on the main thread. A process that is killed from
 * outside (TerminateProcess, Task Manager) runs none of this, and Windows ends its threads itself.
 *
 * This file includes windows.h, whose macros clash with names in sim.h, so it does not include
 * sim.h. sim.h declares `submit_job` and `wait_for_jobs`. */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <windows.h>
#include <xmmintrin.h>

#define MAX_JOBS 32          /* Jobs between two waits stay below this, as runs have 8 squads. */
#define MAX_WAKE_COUNT 0x7fffffff /* The semaphore of the jobs never reaches this count. */
#define STOP_WAIT_MILLISECONDS 2000 /* The stop waits this long at most for the workers to end. */

typedef void WorkerJob(void *argument);

/* A `Job` holds one call to run, `function(argument)`, with the floating-point control words of
 * the thread that submitted it. */
typedef struct {
    WorkerJob *function;
    void *argument;
    uint16_t x87_control;
    uint32_t sse_control;
} Job;

/* `jobs` holds the jobs as the top comment describes. The threads change the counts only with
 * Interlocked functions of Windows, which also keep the reads and writes around them in order. */
static Job jobs[MAX_JOBS];
static volatile LONG submitted_count;
static volatile LONG claimed_count;
static volatile LONG finished_count;

/* Each submitted job adds 1 to `job_semaphore`, and each worker thread takes 1 before it looks
 * for jobs. When the main thread ran a job itself, a worker later wakes once and finds nothing.
 * `worker_count` worker threads run once `are_workers_started` is true, and `worker_threads` holds
 * their handles. GetSystemInfo counts the cores of one processor group, at most 64, so the handles
 * fit, and one call of WaitForMultipleObjects can wait for all of them. */
static HANDLE job_semaphore;
static int worker_count;
static bool are_workers_started;
static HANDLE worker_threads[MAXIMUM_WAIT_OBJECTS];

/* `is_stop_requested` becomes 1 when `stop_workers` starts. It is a LONG, not a bool, so that
 * InterlockedExchange can set it. `previous_crash_filter` is the unhandled-exception filter that
 * was set before `handle_crash`, or NULL. */
static volatile LONG is_stop_requested;
static LPTOP_LEVEL_EXCEPTION_FILTER previous_crash_filter;

/* ------------------------------------------------------------------------------------------------
 * `read_x87_control` returns the control word of the x87 unit of this thread.
 * --------------------------------------------------------------------------------------------- */
static uint16_t read_x87_control(void) {
    uint16_t control;
    __asm__ volatile("fnstcw %0" : "=m"(control));
    return control;
}

/* ------------------------------------------------------------------------------------------------
 * `write_x87_control` sets the control word of the x87 unit of this thread.
 * --------------------------------------------------------------------------------------------- */
static void write_x87_control(uint16_t control) {
    __asm__ volatile("fldcw %0" : : "m"(control));
}

/* ------------------------------------------------------------------------------------------------
 * `run_job` runs one job with the control words of the job, and then gives the thread its own
 * control words back.
 * --------------------------------------------------------------------------------------------- */
static void run_job(const Job *job) {
    uint16_t own_x87_control = read_x87_control();
    uint32_t own_sse_control = _mm_getcsr();
    write_x87_control(job->x87_control);
    _mm_setcsr(job->sse_control);
    job->function(job->argument);
    write_x87_control(own_x87_control);
    _mm_setcsr(own_sse_control);
}

/* ------------------------------------------------------------------------------------------------
 * `run_unclaimed_jobs` takes the jobs that no thread took yet, one after the other, and runs
 * each. It returns when no submitted job is left to take.
 * --------------------------------------------------------------------------------------------- */
static void run_unclaimed_jobs(void) {
    while (true) {
        /* Stop when every submitted job is taken. */
        LONG job_number = claimed_count;
        if (job_number >= submitted_count)
            return;

        /* Take the job, unless another thread took it first, and run it. */
        LONG seen_number = InterlockedCompareExchange(&claimed_count, job_number + 1, job_number);
        if (seen_number != job_number)
            continue;
        run_job(&jobs[job_number % MAX_JOBS]);
        InterlockedIncrement(&finished_count);
    }
}

/* ------------------------------------------------------------------------------------------------
 * `run_worker` is the loop of a worker thread: it sleeps until a job comes, and runs the jobs
 * that it can take. The thread ends when it wakes after `stop_workers` asked the workers to stop.
 * --------------------------------------------------------------------------------------------- */
static DWORD WINAPI run_worker(LPVOID unused) {
    (void)unused;
    while (true) {
        WaitForSingleObject(job_semaphore, INFINITE);
        if (is_stop_requested)
            return 0;
        run_unclaimed_jobs();
    }
}

/* ------------------------------------------------------------------------------------------------
 * `stop_workers` asks the worker threads to stop, wakes them, and waits until they end, for at
 * most `STOP_WAIT_MILLISECONDS`. Only the first call stops them, and later calls return at once.
 * It does not wait for the calling thread, which can be a worker that crashed or called `exit`.
 * The semaphore stays open, because `submit_job` may still release it after the stop, and the
 * process frees it at its end.
 * --------------------------------------------------------------------------------------------- */
static void stop_workers(void) {
    /* Let only the first caller stop the workers. */
    if (InterlockedExchange(&is_stop_requested, 1) != 0)
        return;

    /* Wake every worker. A worker that runs a job sees the request after its jobs. */
    ReleaseSemaphore(job_semaphore, worker_count, NULL);

    /* Wait for the workers other than the calling thread. */
    HANDLE waited_threads[MAXIMUM_WAIT_OBJECTS];
    DWORD waited_count = 0;
    DWORD own_thread_id = GetCurrentThreadId();
    for (int worker_index = 0; worker_index < worker_count; worker_index++) {
        HANDLE thread = worker_threads[worker_index];
        if (GetThreadId(thread) == own_thread_id)
            continue;
        waited_threads[waited_count] = thread;
        waited_count++;
    }
    if (waited_count > 0)
        WaitForMultipleObjects(waited_count, waited_threads, TRUE, STOP_WAIT_MILLISECONDS);

    /* Release the thread handles. A thread that is still running goes on until the process ends. */
    for (int worker_index = 0; worker_index < worker_count; worker_index++)
        CloseHandle(worker_threads[worker_index]);
}

/* ------------------------------------------------------------------------------------------------
 * `handle_console_event` stops the workers on Ctrl+C, Ctrl+Break, a closed console, a logoff or a
 * shutdown. Windows calls it on a thread of its own. It returns FALSE, so that the next handler,
 * by default the one of Windows, ends the program as before.
 * --------------------------------------------------------------------------------------------- */
static BOOL WINAPI handle_console_event(DWORD event_type) {
    (void)event_type;
    stop_workers();
    return FALSE;
}

/* ------------------------------------------------------------------------------------------------
 * `handle_crash` stops the workers when a thread meets an exception that nothing handles, such as
 * an access violation. It then passes the exception to the filter that was set before, so that the
 * crash goes on as before.
 * --------------------------------------------------------------------------------------------- */
static LONG WINAPI handle_crash(EXCEPTION_POINTERS *exception) {
    stop_workers();
    if (previous_crash_filter == NULL)
        return EXCEPTION_CONTINUE_SEARCH;
    return previous_crash_filter(exception);
}

/* ------------------------------------------------------------------------------------------------
 * `start_workers` starts one worker thread for each processor core but one, which the main thread
 * keeps. On one core, it starts none, and the main thread runs every job in `wait_for_jobs`.
 * With workers, it also sets the handlers that stop them (see "Stop" in the top comment).
 * --------------------------------------------------------------------------------------------- */
static void start_workers(void) {
    /* Start the threads. */
    SYSTEM_INFO system_info;
    GetSystemInfo(&system_info);
    worker_count = (int)system_info.dwNumberOfProcessors - 1;
    job_semaphore = CreateSemaphoreA(NULL, 0, MAX_WAKE_COUNT, NULL);
    for (int worker_index = 0; worker_index < worker_count; worker_index++)
        worker_threads[worker_index] = CreateThread(NULL, 0, run_worker, NULL, 0, NULL);
    are_workers_started = true;

    /* Stop them at the end of the program, on Ctrl+C or a closed console, and on a crash. */
    if (worker_count == 0)
        return;
    atexit(stop_workers);
    SetConsoleCtrlHandler(handle_console_event, TRUE);
    previous_crash_filter = SetUnhandledExceptionFilter(handle_crash);
}

/* ------------------------------------------------------------------------------------------------
 * `submit_job` fills the entry of the next job before it counts the job, so a thread that sees
 * the count also sees the entry. It starts the worker threads at its first call.
 * --------------------------------------------------------------------------------------------- */
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

/* ------------------------------------------------------------------------------------------------
 * `wait_for_jobs` lets the main thread help with the jobs, and then waits for the jobs that the
 * workers still run. While it waits, it gives its processor core to other threads.
 * --------------------------------------------------------------------------------------------- */
void wait_for_jobs(void) {
    run_unclaimed_jobs();
    while (finished_count < submitted_count)
        SwitchToThread();
    MemoryBarrier();
}
