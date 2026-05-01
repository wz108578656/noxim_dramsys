// ============================================================================
// sc_main.cpp - Unified NoC + DRAM co-simulation entry point
// ============================================================================
//
// Architecture:
//   Mode 1 (default): Hub initiators (wireless) → Passthrough → DRAMSys
//     - Used when use_winoc=true with configured Hub initiators
//
//   Mode 2 (--mem-traffic): MemTrafficManager → Passthrough → DRAMSys
//     - 4 traffic generators (one per Hub) send interleaved memory traffic
//     - Each generator cycles through all 6 channels for even distribution
//     - Reports per-channel and aggregate bandwidth statistics
//
// Memory map:
//   Channel stride = 0x20000000 (512MB per channel for 6-channel config)
//   Channel n base = n * 0x20000000
//
// ============================================================================

#include <cstring>
#include <iostream>
#include <memory>

#include <systemc>
#include <tlm>

#include "DramInterface.h"
#include "NoCInterface.h"
#include "MemTrafficGen.h"
#include "GlobalParams.h"
#include "Initiator.h"

using namespace sc_core;
using namespace std;

static void printUsage(const char* prog)
{
    cerr << "Usage: " << prog << " [options]\n"
         << "  --noxim-config <file>    Noxim YAML config (required for NoC mode)\n"
         << "  --dram-config  <file>    DRAMSys JSON config (required)\n"
         << "  --dram-base   <addr>     DRAM base address (default: 0x0)\n"
         << "  --dram-stride <n>        Address stride per hub (default: 0x100000)\n"
         << "  --cycles      <n>        Simulation cycles (0 = use config default)\n"
         << "  --mem-traffic <n>        Memory bandwidth test with N Hubs (e.g., --mem-traffic 4)\n"
         << "                           Replaces Hub initiators with MemTrafficManager\n"
         << "  --injection-rate <rate>  Traffic injection rate (default: 0.02)\n"
         << "  --num-channels <n>       Number of DRAM channels (default: 6)\n"
         << "  --help                  Show this help\n"
         << endl;
}

int sc_main(int argc, char* argv[])
{
    cout << "============================================" << endl;
    cout << "  noxim_dramsys - Unified NoC + DRAM co-sim" << endl;
    cout << "============================================" << endl;
    cout << "SystemC version: " << SC_VERSION << endl;
    cout << "Compiled with:   " << __VERSION__ << endl;

    // -------------------------------------------------------------------------
    // Parse command-line arguments
    // -------------------------------------------------------------------------
    string noximConfig, dramConfig, dramBase = "0x0";
    uint64_t dramStride = 0x100000ULL;
    int maxCycles = 0;
    int memTrafficHubs = 0;  // 0 = no mem traffic mode
    double injectionRate = 0.02;
    int numDramChannels = 6;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "--noxim-config") == 0 && i + 1 < argc) {
            noximConfig = argv[++i];
        } else if (strcmp(argv[i], "--dram-config") == 0 && i + 1 < argc) {
            dramConfig = argv[++i];
        } else if (strcmp(argv[i], "--dram-base") == 0 && i + 1 < argc) {
            dramBase = argv[++i];
        } else if (strcmp(argv[i], "--dram-stride") == 0 && i + 1 < argc) {
            dramStride = std::stoull(argv[++i], nullptr, 0);
        } else if (strcmp(argv[i], "--cycles") == 0 && i + 1 < argc) {
            maxCycles = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "--mem-traffic") == 0 && i + 1 < argc) {
            memTrafficHubs = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "--injection-rate") == 0 && i + 1 < argc) {
            injectionRate = std::stod(argv[++i]);
        } else if (strcmp(argv[i], "--num-channels") == 0 && i + 1 < argc) {
            numDramChannels = std::stoi(argv[++i]);
        } else {
            cerr << "Unknown option: " << argv[i] << endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    if (dramConfig.empty()) {
        cerr << "ERROR: --dram-config is required" << endl;
        printUsage(argv[0]);
        return 1;
    }

    cout << "\n--- Configuration ---" << endl;
    cout << "  Noxim config:   " << (noximConfig.empty() ? "(none)" : noximConfig) << endl;
    cout << "  DRAMSys config: " << dramConfig << endl;
    cout << "  DRAM base:      " << dramBase << endl;
    cout << "  DRAM stride:    0x" << std::hex << dramStride << std::dec << endl;
    cout << "  DRAM channels:  " << numDramChannels << endl;
    if (memTrafficHubs > 0) {
        cout << "  MEM traffic:    " << memTrafficHubs << " Hubs, rate=" << injectionRate << endl;
    }

    // -------------------------------------------------------------------------
    // 1. Create DramInterface (owns DRAMSys + internal Passthrough bridge).
    // -------------------------------------------------------------------------
    cout << "\n--- Creating DramInterface ---" << endl;
    DramIf::DramInterface dramIf("DramInterface",
                                  dramConfig,
                                  dramBase,
                                  dramStride);

    if (!dramIf.isConfigured()) {
        cerr << "ERROR: DramInterface initialization failed" << endl;
        return 1;
    }

    // -------------------------------------------------------------------------
    // 2a. Memory traffic mode: Create MemTrafficManager for bandwidth testing
    // -------------------------------------------------------------------------
    DramIf::MemTrafficManager* trafficMgr = nullptr;

    if (memTrafficHubs > 0) {
        cout << "\n--- Memory Traffic Bandwidth Test Mode ---" << endl;
        cout << "  Creating " << memTrafficHubs << " memory traffic generators" << endl;
        cout << "  Channel interleaving: " << numDramChannels << " channels" << endl;
        cout << "  Injection rate: " << injectionRate << " tx/cycle" << endl;

        auto& passthroughSocket = dramIf.getUpstreamSocket();

        trafficMgr = new DramIf::MemTrafficManager(
            "MemTrafficManager",
            memTrafficHubs,
            passthroughSocket,
            injectionRate,
            numDramChannels);

        // -------------------------------------------------------------------------
        // 3. Run simulation with Noxim (for clock generation)
        // -------------------------------------------------------------------------
        cout << "\n--- Starting simulation (memory traffic mode) ---" << endl;

        // Run simulation (no Noxim needed in mem-traffic mode)
        sc_start(maxCycles, SC_NS);

        // Print bandwidth statistics
        if (trafficMgr) {
            trafficMgr->printAggregateStats();
        }
    } else {
        // -------------------------------------------------------------------------
        // 2. Create NoCInterface (owns Noxim NoC + Hub initiators).
        // -------------------------------------------------------------------------
        cout << "\n--- Creating NoCInterface ---" << endl;
        NoximIntegration::NoCInterface nocIf("NoCInterface");

        if (!nocIf.loadConfiguration(noximConfig)) {
            cerr << "ERROR: NoCInterface configuration failed" << endl;
            return 1;
        }

        // -------------------------------------------------------------------------
        // 3. Bind Hub initiators directly to DramInterface's Passthrough downstream.
        // -------------------------------------------------------------------------
        cout << "\n--- Binding Hub initiators to DramInterface Passthrough ---" << endl;

        auto& hubInitiators = nocIf.getHubInitiators();
        int boundCount = 0;

        if (hubInitiators.empty()) {
            cout << "Note: No Hub initiators (wireless channels) in current config.\n"
                 << "      Use --mem-traffic N to generate memory bandwidth traffic.\n";
        } else {
            auto& passthroughSocket = dramIf.getUpstreamSocket();

            for (auto& hubPair : hubInitiators) {
                int hubId = hubPair.first;
                for (auto& chPair : hubPair.second) {
                    int channel = chPair.first;
                    Initiator* initiator = chPair.second;
                    initiator->socket.bind(passthroughSocket);
                    cout << "  Hub[" << hubId << "].init[" << channel
                         << "] -> Passthrough" << endl;
                    ++boundCount;
                }
            }
            cout << "  Total bindings: " << boundCount << endl;
        }

        // -------------------------------------------------------------------------
        // 3c. DRAM Verification
        // -------------------------------------------------------------------------
        {
            auto& passthroughSocket = dramIf.getUpstreamSocket();
            DramIf::DramVerifier* verifier = new DramIf::DramVerifier(
                "DramVerifier", passthroughSocket, dramIf, 0x1000, 0xDEADBEEF);
            (void)verifier;
        }

        // -------------------------------------------------------------------------
        // 4. Run simulation
        // -------------------------------------------------------------------------
        cout << "\n--- Starting simulation ---" << endl;
        nocIf.run(maxCycles);
    }

    cout << "\n============================================" << endl;
    cout << "Simulation completed at " << sc_time_stamp() << endl;
    cout << "============================================" << endl;
    return 0;
}