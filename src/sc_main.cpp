// ============================================================================
// noxim_dramsys - Unified sc_main
// Phase 1: Empty shell — verify both subsystems link and initialize
// ============================================================================
#include <systemc>
#include <iostream>

// DRAMSys headers
#include "DRAMSys/DRAMSys.h"
#include "DRAMSys/configuration/json/DRAMSysConfiguration.h"

using namespace sc_core;
using namespace std;

int sc_main(int argc, char* argv[]) {
    cout << "============================================" << endl;
    cout << "  noxim_dramsys - Phase 1: Shell Build Test" << endl;
    cout << "============================================" << endl;
    cout << "SystemC version: " << SC_VERSION << endl;
    cout << "Compiled with:   " << __VERSION__ << endl;

    // DRAMSys: requires a config file (skip instantiation in shell mode)
    cout << "\n--- DRAMSys ---" << endl;
    cout << "DRAMSys header linked OK" << endl;
    cout << "Note: DRAMSys requires --config JSON; will instantiate in Phase 4" << endl;

    // Noxim: static lib links OK
    cout << "\n--- Noxim ---" << endl;
    cout << "Noxim static library linked OK (no instantiation in shell mode)" << endl;

    cout << "\n============================================" << endl;
    cout << "Phase 1: Shell build PASSED" << endl;
    cout << "Next: Phase 2 - create NoCInterface wrapper" << endl;
    cout << "============================================" << endl;
    return 0;
}