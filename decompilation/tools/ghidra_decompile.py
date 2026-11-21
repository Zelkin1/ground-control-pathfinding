"""`ghidra_decompile` decompiles every function of gc.exe with Ghidra, headless (without the
window of Ghidra), in about 6 minutes.

The script works in four steps:
1. It imports gc.exe into a new Ghidra project in `project_dir` and lets Ghidra analyze it.
2. It finds every function whose body reads ECX before it sets it. In MSVC6 code, such a function
   is a `thiscall` method, which gets its `this` pointer in ECX.
3. It marks those functions as __thiscall, so that their decompiled code shows `this`.
4. It decompiles every function into <out_dir>/funcs/<address>.c and into <out_dir>/all.c, one
   file with the whole decompile.

It needs a Python with PyGhidra, JAVA_HOME set to a JDK 25+ and GHIDRA_INSTALL_DIR set to a
Ghidra 12.x folder. The script puts the settings, cache and temp folders of Ghidra in
<project_dir>/../ghidra_home, because a sandboxed %APPDATA% breaks the OSGi path check of Ghidra.
Run it on a copy of gc.exe that decompilation/tools/fix_pe_header.py wrote. That tool moves the NT
headers of gc.exe out of its DOS header, where they overlap.

Usage: python -I decompilation/tools/ghidra_decompile.py <exe> <project_dir> <out_dir>
"""
import sys
import time
from pathlib import Path

THISCALL_CHECK_TIMEOUT_SECONDS = 60  # A decompile that looks for reads of ECX gives up after this.
DECOMPILE_TIMEOUT_SECONDS = 120  # A decompile for the output gives up after this.
PROJECT_NAME = "gc2"  # This is the name of the Ghidra project in `project_dir`.
IS_FORWARD_ORDER = True  # With this, `getFunctions` lists the functions by rising address.

# Make the output folders.
exe_path, project_dir, output_dir = (Path(argument) for argument in sys.argv[1:4])
functions_dir = output_dir / "funcs"
functions_dir.mkdir(parents=True, exist_ok=True)
ghidra_home = project_dir.parent / "ghidra_home"

# Start the JVM of Ghidra (the Java virtual machine that runs Ghidra) with its settings, cache and
# temp folders under `ghidra_home`. Each pair names a JVM property and its folder. The ghidra
# modules imported below exist only once the JVM runs.
from pyghidra.launcher import HeadlessPyGhidraLauncher  # noqa: E402

launcher = HeadlessPyGhidraLauncher()
property_subfolders = (("settingsdir", "settings"), ("cachedir", "cache"), ("tempdir", "temp"))
for property_name, subfolder in property_subfolders:
    (ghidra_home / subfolder).mkdir(parents=True, exist_ok=True)
    launcher.add_vmargs(f"-Dapplication.{property_name}={ghidra_home / subfolder}")
launcher.start()
import pyghidra  # noqa: E402
from ghidra.app.decompiler import DecompInterface, DecompileOptions  # noqa: E402
from ghidra.util.task import ConsoleTaskMonitor  # noqa: E402

# Import the exe and let Ghidra analyze it.
start_time = time.time()
open_options = dict(project_location=str(project_dir), project_name=PROJECT_NAME, analyze=True)
with pyghidra.open_program(str(exe_path), **open_options) as flat_api:
    print(f"analyzed in {time.time() - start_time:.0f}s", flush=True)
    program = flat_api.getCurrentProgram()
    monitor = ConsoleTaskMonitor()

    def create_decompiler():
        """`create_decompiler` returns a new decompiler with the default options, opened on
        `program`."""
        decompiler = DecompInterface()
        decompiler.setOptions(DecompileOptions())
        decompiler.openProgram(program)
        return decompiler

    # Find the functions that read ECX before they set it. Ghidra names such an input "in_ECX",
    # and in MSVC6 code it is the `this` pointer of a thiscall method.
    decompiler = create_decompiler()
    thiscall_functions = []
    for function in program.getFunctionManager().getFunctions(IS_FORWARD_ORDER):
        result = decompiler.decompileFunction(function, THISCALL_CHECK_TIMEOUT_SECONDS, monitor)
        if not result.decompileCompleted():
            continue
        c_code = str(result.getDecompiledFunction().getC())
        if "in_ECX" in c_code:
            thiscall_functions.append(function)
    decompiler.dispose()

    # Mark them __thiscall. Ghidra changes a program only inside a transaction, and True makes
    # `endTransaction` keep the changes.
    transaction_id = program.startTransaction("thiscall")
    for function in thiscall_functions:
        function.setCallingConvention("__thiscall")
    program.endTransaction(transaction_id, True)
    thiscall_count = len(thiscall_functions)
    elapsed_seconds = time.time() - start_time
    print(f"marked {thiscall_count} functions __thiscall ({elapsed_seconds:.0f}s)", flush=True)

    # Decompile every function into all.c and into one file per function. The text of each
    # function starts with the line "// ==== <address> <name> (<calling convention>)", so that a
    # search in all.c can find a function by these lines. all.c gets one more newline after each
    # text.
    decompiler = create_decompiler()
    function_count = 0
    failed_count = 0
    with open(output_dir / "all.c", "w", encoding="utf-8") as all_c_file:
        for function in program.getFunctionManager().getFunctions(IS_FORWARD_ORDER):
            address = str(function.getEntryPoint())
            result = decompiler.decompileFunction(function, DECOMPILE_TIMEOUT_SECONDS, monitor)
            if result.decompileCompleted():
                c_code = str(result.getDecompiledFunction().getC())
            else:
                c_code = f"/* decompile failed: {result.getErrorMessage()} */\n"
                failed_count += 1
            function_name = function.getName()
            convention_name = function.getCallingConventionName()
            header = f"// ==== {address} {function_name} ({convention_name})"
            text = f"{header}\n{c_code}"
            all_c_file.write(text + "\n")
            (functions_dir / f"{address}.c").write_text(text, encoding="utf-8")
            function_count += 1
    decompiler.dispose()
    elapsed_seconds = time.time() - start_time
    summary = f"done: {function_count} functions, {failed_count} failed, {elapsed_seconds:.0f}s"
    print(summary, flush=True)
