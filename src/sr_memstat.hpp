#pragma once

#include <windows.h>
#include <psapi.h>
#include <cstdio>

inline void print_mem(const char* tag = "")
{
    PROCESS_MEMORY_COUNTERS_EX pmc = {};
    pmc.cb = sizeof(pmc);
    GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc));

    float working_set  = pmc.WorkingSetSize  / (1024.0f * 1024.0f);
    float private_mb   = pmc.PrivateUsage    / (1024.0f * 1024.0f);
    float peak_ws      = pmc.PeakWorkingSetSize / (1024.0f * 1024.0f);

    printf("[MEM] %s\n", tag);
    printf("  WorkingSet:  %.1f MB  (lo que ve Task Manager)\n", working_set);
    printf("  PrivateUse:  %.1f MB  (solo tuyo, excluye DLLs compartidas)\n", private_mb);
    printf("  PeakWS:      %.1f MB\n", peak_ws);
}
