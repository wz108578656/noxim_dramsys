// ============================================================================
// sc_main.cpp - Unified NoC + DRAM co-simulation entry point
// ============================================================================
//
// Architecture (simplified — no NoC2DramBridge):
//   HubInitiator.socket
//     | .bind(dramIf.getUpstreamSocket())    ← binds to Passthrough.downstream
//     | Passthrough captures Hub's bw if at end of elaboration
//     v
//   DramIf::Passthrough (sc_module, fw+bw tlm interfaces)
//     | translates address (hub-local → DRAM)
//     | calls ds->tSocket.b_transport / nb_transport_fw (registered callbacks)
//     v
//   DRAMSys (dramsys->tSocket = multi_passthrough_target_socket<DRAMSys>)
//     | internal chain: tSocket→Arbiter→Controller→Dram
//     v
//   DRAMSys calls Passthrough.nb_transport_bw (via registered callback)
//     → Passthrough forwards to Hub's bw if (captured at bind time).
//
// Memory map:
//   hub_id → DRAM address = BASE + hub_id * STRIDE + hub-local addr
//
// Verification mode (--verify-dram):
//   Inject TLM read/write transactions directly into Passthrough downstream
//   to validate the full Passthrough → Arbiter → Controller → Dram path.
// ============================================================================

#include <cstring>
#include <iostream>
#include <memory>

#include <systemc>
#include <tlm>

#include "DramInterface.h"
#include "NoCInterface.h"
#include "GlobalParams.h"
#include "Initiator.h"

using namespace sc_core;
using namespace std;

static void printUsage(const char* prog)
{
    cerr << "Usage: " << prog << " [options]\n"
         << "  --noxim-config <file>    Noxim YAML config (required)\n"
         << "  --dram-config  <file>    DRAMSys JSON config (required)\n"
         << "  --dram-base   <addr>     DRAM base address (default: 0x1000000000)\n"
         << "  --dram-stride <n>        Address stride per hub (default: 0x100000)\n"
         << "  --cycles      <n>        Simulation cycles (0 = use config default)\n"
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
    string noximConfig, dramConfig, dramBase = "0x1000000000";
    uint64_t dramStride = 0x100000ULL;
    int maxCycles = 0;

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
        } else {
            cerr << "Unknown option: " << argv[i] << endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    if (noximConfig.empty() || dramConfig.empty()) {
        cerr << "ERROR: --noxim-config and --dram-config are required" << endl;
        printUsage(argv[0]);
        return 1;
    }

    cout << "\n--- Configuration ---" << endl;
    cout << "  Noxim config:   " << noximConfig << endl;
    cout << "  DRAMSys config: " << dramConfig << endl;
    cout << "  DRAM base:      " << dramBase << endl;
    cout << "  DRAM stride:    0x" << std::hex << dramStride << std::dec << endl;

    // -------------------------------------------------------------------------
    // 1. Create DramInterface (owns DRAMSys + internal Passthrough bridge).
    //    Passthrough registers fw+bw callbacks on DRAMSys.tSocket and binds
    //    its upstream initiator to DRAMSys.tSocket.
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
             << "      Standalone NoC simulation — no DRAM traffic expected.\n";
    } else {
        // Get Passthrough's downstream target socket.
        auto& passthroughSocket = dramIf.getUpstreamSocket();

        for (auto& hubPair : hubInitiators) {
            int hubId = hubPair.first;
            for (auto& chPair : hubPair.second) {
                int channel = chPair.first;
                Initiator* initiator = chPair.second;

                // Bind Hub initiator to Passthrough's downstream.
                // Passthrough implements tlm_fw_transport_if (required by
                // simple_target_socket) and will forward transactions to DRAMSys.
                initiator->socket.bind(passthroughSocket);

                cout << "  Hub[" << hubId << "].init[" << channel
                     << "] -> Passthrough" << endl;
                ++boundCount;
            }
        }
        cout << "  Total bindings: " << boundCount << endl;
    }

// -------------------------------------------------------------------------
    // 3c. DRAM Verification: inject transactions directly into Passthrough
    //     to validate the full Passthrough → Arbiter → Controller → Dram path.
    // -------------------------------------------------------------------------
    {
        // Access Passthrough's downstream target socket for verification
        auto& passthroughSocket = dramIf.getUpstreamSocket();

        // Create a verification initiator that binds to Passthrough
        DramIf::DramVerifier* verifier = new DramIf::DramVerifier(
            "DramVerifier", passthroughSocket, dramIf, 0x1000, 0xDEADBEEF);
        (void)verifier;  // owned by SystemC, deleted at end of simulation
    }

    // -------------------------------------------------------------------------
    // 4. Run simulation
    // -------------------------------------------------------------------------
    cout << "\n--- Starting simulation ---" << endl;
    nocIf.run(maxCycles);

    cout << "\n============================================" << endl;
    cout << "Simulation completed at " << sc_time_stamp() << endl;
    cout << "============================================" << endl;
    return 0;
}