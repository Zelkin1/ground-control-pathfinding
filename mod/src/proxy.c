// Proxy layer of the mod. gc.exe imports wsock32.dll, and Windows looks for it in the folder of
// gc.exe first, so this DLL loads in place of the system one (docs/PATHFINDING.md, "Loading a mod
// DLL"). Every export of this DLL is an assembly stub, written by tools/pathfix/gen_proxy.py,
// that jumps through `g_proxy_procs`. `DllMain` fills that table from the real wsock32.dll and
// then starts the mod.
#include <windows.h>

#include "mod.h"
#include "proxy_names.h"

// Addresses of the real exports, in the order of `g_proxy_exports`. The stubs read this table, so
// its name is fixed by gen_proxy.py.
void *g_proxy_procs[PROXY_NUM_EXPORTS];


// `load_real_dll` loads the system wsock32.dll and fills `g_proxy_procs`. It returns false when
// the system DLL cannot be loaded. A 32-bit process that asks for the system folder reaches
// SysWOW64, where the 32-bit DLL is.
static bool load_real_dll(void) {
    wchar_t dll_path[MAX_PATH];
    UINT folder_length = GetSystemDirectoryW(dll_path, MAX_PATH);
    if (folder_length == 0)
        return false;
    lstrcatW(dll_path, L"\\");
    lstrcatW(dll_path, PROXY_DLL);
    HMODULE real_dll = LoadLibraryW(dll_path);
    if (!real_dll)
        return false;

    // An export without a name is found by its ordinal, which GetProcAddress takes in place of
    // the name.
    for (int export_index = 0; export_index < PROXY_NUM_EXPORTS; export_index++) {
        const char *export_name = g_proxy_exports[export_index].name;
        if (!export_name)
            export_name = (const char *)(ULONG_PTR)g_proxy_exports[export_index].ordinal;
        g_proxy_procs[export_index] = (void *)GetProcAddress(real_dll, export_name);
    }
    return true;
}


// `DllMain` loads the real DLL and starts the mod when gc.exe loads this DLL. Without the real
// DLL the game could not use the network, so the load fails then.
BOOL WINAPI DllMain(HINSTANCE instance, DWORD reason, LPVOID reserved) {
    (void)instance;
    (void)reserved;
    if (reason != DLL_PROCESS_ATTACH)
        return TRUE;
    if (!load_real_dll())
        return FALSE;
    start_mod();
    return TRUE;
}
