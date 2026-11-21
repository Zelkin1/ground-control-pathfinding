"""`loadtest` checks that the gcmod proxy DLL really loads inside gc.exe, without starting the
game.

The proxy DLL is the mod DLL that gc.exe loads in place of the system wsock32.dll, and
mod/tools/gen_proxy.py describes it. The test works in three steps:
1. It empties `build/loadtest` and copies into it gc.exe, the DLLs that gc.exe loads from its own
   folder, and the .dll and .ini files of the mod from `build/mod/dist` (mod/build.sh). It only
   reads the game folder.
2. It starts the copied gc.exe with the compatibility layers that Steam sets. The shims
   (compatibility fixes) that these layers load keep a winmm proxy from loading.
3. It waits until the proxy writes `gcmod.log`, lists the modules (the exe and its DLLs) of the
   process, and kills the process. The copy has no data folder, so the game cannot get far.
The test passes when the process loaded the wsock32.dll in `build/loadtest`, not the system one.

Usage: python -I mod/tools/loadtest.py
"""
import ctypes
import os
import shutil
import sys
import time
from ctypes import wintypes
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]  # This file is in the mod/tools folder.
# This is the game folder of a default Steam install. The test only reads from it.
GAME_DIR = Path(r"C:\Program Files (x86)\Steam\steamapps\common\Ground Control Anthology")
TEST_DIR = PROJECT_ROOT / "build" / "loadtest"  # The test runs gc.exe in this folder.

GAME_FILES = ("gc.exe", "binkw32.dll", "ijl11.dll")  # These are gc.exe and its own DLLs.
MOD_DIST_DIR = PROJECT_ROOT / "build" / "mod" / "dist"  # mod/build.sh puts the mod files here.
MOD_FILE_EXTENSIONS = (".dll", ".ini")  # The test copies the files of these types from it.
# Steam sets the layers RUNASADMIN HIGHDPIAWARE for gc.exe. The test uses RunAsInvoker instead of
# RUNASADMIN, so that Windows does not ask for admin rights (the UAC prompt).
COMPAT_LAYERS = "RunAsInvoker HighDpiAware"
# The layers load these DLLs of the shim engine. `find_modules` needs the names in lower case.
SHIM_DLL_NAMES = ("apphelp.dll", "acgenral.dll", "aclayers.dll")
PROXY_DLL_NAMES = ("wsock32.dll", "winmm.dll")  # The test prints the paths of these DLLs.
LOG_FILE_NAME = "gcmod.log"  # The proxy writes this file next to gc.exe when it loads.

LOG_WAIT_TIMEOUT_SECONDS = 4  # The test waits this long for the log of the proxy.
LOG_SETTLE_SECONDS = 0.15  # This gives the proxy time to finish its first log lines.
POLL_INTERVAL_SECONDS = 0.01  # The test looks for the log this often.
EXIT_WAIT_TIMEOUT_MILLISECONDS = 3000  # The test waits this long for the killed process to end.
EXIT_SUCCESS = 0  # The script exits with this status when the proxy loaded.
EXIT_FAILURE = 1  # The script exits with this status when the proxy did not load.

# The Windows SDK headers define these Windows API values.
CREATE_UNICODE_ENVIRONMENT = 0x400  # With this flag, CreateProcessW reads a UTF-16 environment.
LIST_MODULES_32BIT = 0x01  # With this filter, EnumProcessModulesEx lists 32-bit modules.
MAX_MODULES = 1024  # The list of module handles has room for this many modules.
MAX_PATH_CHARACTERS = 520  # This is twice MAX_PATH (260), so that long module paths fit.

# The test calls these Windows libraries through ctypes. With `use_last_error`, ctypes keeps the
# error code of each call for `ctypes.get_last_error`.
kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
psapi = ctypes.WinDLL("psapi", use_last_error=True)


class STARTUPINFO(ctypes.Structure):
    """`STARTUPINFO` mirrors the Windows struct STARTUPINFOW, which CreateProcessW reads."""

    _fields_ = [
        ("cb", wintypes.DWORD),
        ("lpReserved", wintypes.LPWSTR),
        ("lpDesktop", wintypes.LPWSTR),
        ("lpTitle", wintypes.LPWSTR),
        ("dwX", wintypes.DWORD),
        ("dwY", wintypes.DWORD),
        ("dwXSize", wintypes.DWORD),
        ("dwYSize", wintypes.DWORD),
        ("dwXCountChars", wintypes.DWORD),
        ("dwYCountChars", wintypes.DWORD),
        ("dwFillAttribute", wintypes.DWORD),
        ("dwFlags", wintypes.DWORD),
        ("wShowWindow", wintypes.WORD),
        ("cbReserved2", wintypes.WORD),
        ("lpReserved2", ctypes.c_void_p),
        ("hStdInput", wintypes.HANDLE),
        ("hStdOutput", wintypes.HANDLE),
        ("hStdError", wintypes.HANDLE),
    ]


class PROCESS_INFORMATION(ctypes.Structure):
    """`PROCESS_INFORMATION` mirrors the Windows struct of the same name, which CreateProcessW
    fills with the handles and ids of the new process."""

    _fields_ = [
        ("hProcess", wintypes.HANDLE),
        ("hThread", wintypes.HANDLE),
        ("dwProcessId", wintypes.DWORD),
        ("dwThreadId", wintypes.DWORD),
    ]


def list_modules(process_handle):
    """`list_modules` returns the paths of the 32-bit modules loaded in the process.

    Edge case: It returns [] when Windows cannot list them.
    """
    # List the module handles. `needed_bytes` receives the size of the whole list.
    module_handles = (ctypes.c_void_p * MAX_MODULES)()
    needed_bytes = wintypes.DWORD()
    handles_size = ctypes.sizeof(module_handles)
    needed_bytes_pointer = ctypes.byref(needed_bytes)
    is_listed = psapi.EnumProcessModulesEx(process_handle, module_handles, handles_size,
                                           needed_bytes_pointer, LIST_MODULES_32BIT)
    if not is_listed:
        return []

    # Read the path of each module.
    module_count = needed_bytes.value // ctypes.sizeof(ctypes.c_void_p)
    module_paths = []
    for module_index in range(module_count):
        path_buffer = ctypes.create_unicode_buffer(MAX_PATH_CHARACTERS)
        module_handle = ctypes.c_void_p(module_handles[module_index])
        psapi.GetModuleFileNameExW(process_handle, module_handle, path_buffer, MAX_PATH_CHARACTERS)
        module_paths.append(path_buffer.value)
    return module_paths


def find_modules(module_paths, file_names):
    """`find_modules` returns the module paths whose file name, in any case, is one of the
    lower-case `file_names`."""
    found_paths = []
    for module_path in module_paths:
        file_name = os.path.basename(module_path).lower()
        if file_name in file_names:
            found_paths.append(module_path)
    return found_paths


def is_our_proxy(module_path):
    """`is_our_proxy` tells whether the module is the wsock32.dll copied into `TEST_DIR`, not the
    system one: its folder is `TEST_DIR` and its path ends with wsock32.dll, both in any case."""
    is_in_test_dir = os.path.dirname(module_path).lower() == str(TEST_DIR).lower()
    return is_in_test_dir and module_path.lower().endswith("wsock32.dll")


def prepare_test_dir():
    """`prepare_test_dir` empties `TEST_DIR` and fills it with the exe and DLLs of the game and
    the files of the mod from `MOD_DIST_DIR`."""
    # Delete the test folder and make it again, empty.
    shutil.rmtree(TEST_DIR, ignore_errors=True)
    TEST_DIR.mkdir(parents=True)

    # Copy the files of the game.
    for file_name in GAME_FILES:
        shutil.copy2(GAME_DIR / file_name, TEST_DIR)

    # Copy the files of the mod.
    for file_name in os.listdir(MOD_DIST_DIR):
        if file_name.lower().endswith(MOD_FILE_EXTENSIONS):
            shutil.copy2(MOD_DIST_DIR / file_name, TEST_DIR)


def start_game():
    """`start_game` starts the copied gc.exe with the compatibility layers of Steam.

    Result: It returns the PROCESS_INFORMATION of the process, or None when the process did not
      start.
    """
    # Set the layers in the environment of the new process. Windows applies the layers that
    # `__COMPAT_LAYER` names when the process starts.
    environment = dict(os.environ)
    environment["__COMPAT_LAYER"] = COMPAT_LAYERS

    # Build the environment block that CreateProcessW reads: each `key=value` string ends with a
    # NUL character, and one more NUL character ends the block.
    variables = "".join(f"{key}={value}\0" for key, value in environment.items())
    environment_block = variables + "\0"
    environment_buffer = ctypes.create_unicode_buffer(environment_block)

    # Start the process in the test folder. Windows needs the size of the struct in `cb`, and
    # ctypes can pass the folder only as a `str`.
    startup_info = STARTUPINFO(cb=ctypes.sizeof(STARTUPINFO))
    process_info = PROCESS_INFORMATION()
    exe_path = TEST_DIR / "gc.exe"
    command_line = ctypes.create_unicode_buffer(f'"{exe_path}"')
    working_dir = str(TEST_DIR)
    startup_info_pointer = ctypes.byref(startup_info)
    process_info_pointer = ctypes.byref(process_info)
    is_created = kernel32.CreateProcessW(None, command_line, None, None, False,
                                         CREATE_UNICODE_ENVIRONMENT, environment_buffer,
                                         working_dir, startup_info_pointer, process_info_pointer)
    if not is_created:
        print("CreateProcess failed", ctypes.get_last_error())
        return None
    return process_info


def wait_for_log(log_path, start_time):
    """`wait_for_log` waits until the file at `log_path` has content, and then for
    `LOG_SETTLE_SECONDS` more.

    Inputs: `start_time` is a `time.time()` value. `wait_for_log` stops waiting
      `LOG_WAIT_TIMEOUT_SECONDS` after it.
    """
    while time.time() - start_time < LOG_WAIT_TIMEOUT_SECONDS:
        has_content = log_path.exists() and log_path.stat().st_size > 0
        if has_content:
            time.sleep(LOG_SETTLE_SECONDS)
            break
        time.sleep(POLL_INTERVAL_SECONDS)


def stop_process(process_info):
    """`stop_process` kills the process, waits for it to end and closes its handles."""
    kernel32.TerminateProcess(process_info.hProcess, 0)
    kernel32.WaitForSingleObject(process_info.hProcess, EXIT_WAIT_TIMEOUT_MILLISECONDS)
    kernel32.CloseHandle(process_info.hProcess)
    kernel32.CloseHandle(process_info.hThread)


def main():
    """`main` runs the test and prints the result.

    Result: It returns the exit status, `EXIT_SUCCESS` when the proxy loaded and `EXIT_FAILURE`
      otherwise.
    """
    # Copy the game and the mod, and start the game.
    prepare_test_dir()
    process_info = start_game()
    if process_info is None:
        return EXIT_FAILURE

    # Wait for the log of the proxy, then list the loaded modules and stop the game.
    log_path = TEST_DIR / LOG_FILE_NAME
    module_paths = []
    start_time = time.time()
    try:
        wait_for_log(log_path, start_time)
        module_paths = list_modules(process_info.hProcess)
    finally:
        stop_process(process_info)

    # Print the shim and proxy DLLs and the log.
    shim_paths = find_modules(module_paths, SHIM_DLL_NAMES)
    proxy_paths = find_modules(module_paths, PROXY_DLL_NAMES)
    elapsed_seconds = time.time() - start_time
    print(f"after {elapsed_seconds:.2f}s: {len(module_paths)} modules")
    shim_names = [os.path.basename(module_path) for module_path in shim_paths]
    print("  shim DLLs:", shim_names or "none")
    for module_path in proxy_paths:
        print("  ", module_path)
    print(f"{LOG_FILE_NAME}:")
    if log_path.exists():
        with open(log_path) as log_file:
            print(log_file.read())
    else:
        print("  (not created)")

    # Pass when the wsock32.dll inside gc.exe is the copy in `TEST_DIR`.
    is_our_proxy_loaded = any(is_our_proxy(module_path) for module_path in proxy_paths)
    result_text = "proxy NOT loaded"
    if is_our_proxy_loaded:
        result_text = "gcmod proxy loaded inside gc.exe"
    print("RESULT:", result_text)
    return EXIT_SUCCESS if is_our_proxy_loaded else EXIT_FAILURE


if __name__ == "__main__":
    sys.exit(main())
