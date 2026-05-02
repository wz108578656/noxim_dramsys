// ============================================================================
// sc_main.cpp — Noxim + DRAMSys unified entry point
// ============================================================================

#include <systemc>
#include <tlm>
#include <iostream>
#include <string>
#include <cstdlib>
// noxim subsystem
#include "NoCInterface.h"     // NoximIntegration::NoCInterface
#include "DramInterface.h"
#include "PE4DRAM.h"
#include "Initiator.h"        // noxim global ::Initiator

using namespace std;
using namespace sc_core;

// -------------------------------------------------------------------------
// Usage
// -------------------------------------------------------------------------
static void printUsage(const char* prog)
{
    cerr << "Usage: " << prog << " [options]\n"
         << "  --dram-config <path>      DRAMSys JSON config (required)\n"
         << "  --noxim-config <path>     Noxim YAML config (optional)\n"
         << "  --pe-mode                 4PE x 4CH dedicated test (no Noxim)\n"
         << "  --pe-tx <N>               Transactions per PE (default 1000)\n"
         << "  --pe-rate <N>             Transactions per ns (0=full speed)\n"
         << "  --max-cycles <N>          Max sim cycles (default 10000)\n"
         << "  --verify                  Enable DRAM path verification\n"
         << endl;
}

// -------------------------------------------------------------------------
// Parse arguments
// -------------------------------------------------------------------------
struct Args {
    string dramConfig;
    string noximConfig;
    int peTx = 1000;
    double peRate = 0.0;   // 0 = full speed
    int maxCycles = 10000;
    bool verify = false;
    bool peMode = false;
};

static Args parseArgs(int argc, char** argv)
{
    Args args;
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "--dram-config" && i + 1 < argc) args.dramConfig = argv[++i];
        else if (arg == "--noxim-config" && i + 1 < argc) args.noximConfig = argv[++i];
        else if (arg == "--pe-mode") args.peMode = true;
        else if (arg == "--pe-tx" && i + 1 < argc) args.peTx = atoi(argv[++i]);
        else if (arg == "--pe-rate" && i + 1 < argc) args.peRate = atof(argv[++i]);
        else if (arg == "--max-cycles" && i + 1 < argc) args.maxCycles = atoi(argv[++i]);
        else if (arg == "--verify") args.verify = true;
        else if (arg == "-h" || arg == "--help") { printUsage(argv[0]); exit(0); }
    }
    return args;
}

// -------------------------------------------------------------------------
// PE4DRAM run helper — called from spawned thread (simulation already running).
// DO NOT create modules here (E529). PE4DRAM is pre-allocated in sc_main.
static void runPE4Test(DramIf::PE4DRAM& pe4test)
{
    pe4test.start();  // Blocks until all testers complete
}

// -------------------------------------------------------------------------
// Main
// -------------------------------------------------------------------------
int sc_main(int argc, char** argv)
{
    Args args = parseArgs(argc, argv);

    if (args.dramConfig.empty()) {
        cerr << "ERROR: --dram-config required" << endl;
        printUsage(argv[0]);
        return 1;
    }

    cout << "============================================" << endl;
    cout << "  noxim_dramsys — Unified NoC + DRAM sim  " << endl;
    cout << "============================================" << endl;
    cout << "  DRAMSys config: " << args.dramConfig << endl;

    // -------------------------------------------------------------------------
    // PE dedicated channel test mode (no Noxim needed)
    // -------------------------------------------------------------------------
    if (args.peMode) {
        cout << "\n--- PE Mode: 4PE x 4CH dedicated (no interleaving) ---" << endl;
        cout << "  Transactions/PE: " << args.peTx << endl;
        cout << "  Injection rate: " << args.peRate << " tx/ns"
             << (args.peRate == 0 ? " [full speed]" : "") << endl;

        // Create DramInterface BEFORE sc_start (DRAMSys spawns internal threads).
        DramIf::DramInterface dramIf("DramInterface", args.dramConfig, 0);

        if (!dramIf.isConfigured()) {
            cerr << "ERROR: DramInterface init failed" << endl;
            return 1;
        }

        // Create PE4DRAM BEFORE sc_start (modules must not be created during simulation).
        DramIf::PE4DRAM pe4test("PE4DRAM", dramIf, args.peTx, args.peRate);

        cout << "\n--- Starting simulation ---" << endl;

        // Spawn thread to call start() after simulation begins (no module creation here).
        sc_spawn([&]() {
            pe4test.start();
            sc_stop();
        });

        sc_start(1000, SC_SEC);

        cout << "\n[sc_main] Simulation complete at " << sc_time_stamp() << endl;
        return 0;
    }

    // -------------------------------------------------------------------------
    // Normal Noxim NoC + DRAMSys mode
    // -------------------------------------------------------------------------
    cout << "\n--- Creating DramInterface ---" << endl;
    DramIf::DramInterface dramIf("DramInterface", args.dramConfig, 0x40000000ULL);

    if (!dramIf.isConfigured()) {
        cerr << "ERROR: DramInterface init failed" << endl;
        return 1;
    }

    cout << "\n--- Creating NoCInterface ---" << endl;
    NoximIntegration::NoCInterface nocIf("NoCInterface");

    if (!args.noximConfig.empty()) {
        if (!nocIf.loadConfiguration(args.noximConfig)) {
            cerr << "ERROR: NoCInterface configuration failed" << endl;
            return 1;
        }
    }

    cout << "\n--- Binding Hub initiators to DramInterface.upstream[0] ---" << endl;
    auto& hubInitiators = nocIf.getHubInitiators();
    auto& targetSocket = dramIf.getUpstreamSocket(0);
    int boundCount = 0;

    if (hubInitiators.empty()) {
        cout << "Note: No Hub initiators in current config.\n"
             << "      Use --pe-mode for memory tests." << endl;
    } else {
        for (auto& hubPair : hubInitiators) {
            int hubId = hubPair.first;
            for (auto& chPair : hubPair.second) {
                int channel = chPair.first;
                ::Initiator* initiator = chPair.second;  // global namespace
                initiator->socket.bind(targetSocket);
                cout << "  Hub[" << hubId << "].init[" << channel
                     << "] -> upstream[0]" << endl;
                ++boundCount;
            }
        }
        cout << "  Total bindings: " << boundCount << endl;
    }

    if (args.verify) {
        cout << "\n--- Creating DramVerifier ---" << endl;
        DramIf::DramVerifier verifier("DramVerifier", targetSocket, 0x1000, 0xDEADBEEF);
    }

    cout << "\n--- Starting Noxim simulation (max " << args.maxCycles << " cycles) ---" << endl;
    sc_start(args.maxCycles, SC_NS);

    cout << "\n============================================" << endl;
    cout << "  Simulation finished at " << sc_time_stamp() << endl;
    cout << "============================================" << endl;

    return 0;
}