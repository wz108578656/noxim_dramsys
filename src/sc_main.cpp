// ============================================================================
// sc_main.cpp — NoC-Mesh (Noxim) + DRAMSys co-simulation top-level
// ============================================================================
// Architecture: 2x4 Noxim mesh with 4 TrafficPEs (row 0) + 4 DramPEs (row 1)
// ============================================================================
#include <systemc.h>
#include <tlm.h>
#include <sysc/tracing/sc_trace.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <cstdlib>
#include <memory>
#include <vector>

#include "traffic_pe.h"
#include "dram_pe.h"
#include "noc_mesh_wiring.h"
#include "DramInterface.h"
#include "GlobalParams.h"

using namespace std;
using namespace sc_core;

// ---------------------------------------------------------------------------
// Arguments
// ---------------------------------------------------------------------------
struct Args {
    string dramConfig;
    int    nocTx        = 1000;
    int    numPEs       = 4;
    double nocRate      = 0.0;
    double clockPeriod  = 1.0;
    bool   modeA        = false;
    bool   modeB        = false;
    bool   interleave   = true;
    bool   lpddr4       = false;
    bool   is_read      = false;
    int    maxCycles    = 100000;
    bool   experimentCompete  = false;
    bool   experimentOpposite = false;
    string vcdFile;
};

static Args parseArgs(int argc, char** argv)
{
    Args args;
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "--dram-config" && i + 1 < argc) args.dramConfig = argv[++i];
        else if (arg == "--noc-tx" && i + 1 < argc) args.nocTx = atoi(argv[++i]);
        else if (arg == "--noc-pe" && i + 1 < argc) args.numPEs = atoi(argv[++i]);
        else if (arg == "--noc-rate" && i + 1 < argc) args.nocRate = atof(argv[++i]);
        else if (arg == "--noc-clock" && i + 1 < argc) args.clockPeriod = atof(argv[++i]);
        else if (arg == "--noc-mode-a") { args.modeA = true; args.interleave = false; }
        else if (arg == "--noc-mode-b") { args.modeB = true; args.interleave = false; }
        else if (arg == "--noc-interleave") args.interleave = true;
        else if (arg == "--vcd" && i + 1 < argc) args.vcdFile = argv[++i];
        else if (arg == "--experiment-compete") {
            args.experimentCompete = true;
            args.interleave = false;
            args.numPEs = 16;
            args.nocTx = 2000;
            args.clockPeriod = 0.2;
            args.maxCycles = 500000;
        }
        else if (arg == "--experiment-opposite") {
            args.experimentOpposite = true;
            args.interleave = false;
            args.numPEs = 16;
            args.nocTx = 2000;
            args.clockPeriod = 0.2;
            args.maxCycles = 500000;
        }
        else if (arg == "--lpddr4") args.lpddr4 = true;
        else if (arg == "--noc-read") args.is_read = true;
        else if (arg == "--max-cycles" && i + 1 < argc) args.maxCycles = atoi(argv[++i]);
        else if (arg == "-h" || arg == "--help") {
            cout << "Usage: " << argv[0] << " --dram-config <json> [opts]\n"
                 << "  --noc-tx <N>       Transactions per PE (default 1000)\n"
                 << "  --noc-pe <N>       Number of PEs (default 4)\n"
                 << "  --noc-rate <ns>    Injection interval in ns (0=max)\n"
                 << "  --noc-clock <ns>   NoC clock period (default 1.0ns)\n"
                 << "  --noc-mode-a       All PEs -> single channel (1x BW)\n"
                 << "  --noc-mode-b       Per-channel (4x BW, default)\n"
                 << "  --lpddr4           LPDDR4 CH bits [31:30]\n"
                 << "  --noc-read         READ transactions (default WRITE)\n"
                 << "  --max-cycles <N>   Max cycles (default 100000)\n"
                 << "  --vcd <file>       VCD trace\n"
                 << "\n  Experiment modes:\n"
                 << "  --experiment-compete   Interleave (16 PE, 4x4 crossbar route)\n"
                 << "  --experiment-opposite  Channel-aware rotated stagger\n"
                 << endl;
            exit(0);
        }
    }
    return args;
}

// ---------------------------------------------------------------------------
// sc_main
// ---------------------------------------------------------------------------
int sc_main(int argc, char** argv)
{
    Args args = parseArgs(argc, argv);

    if (args.dramConfig.empty()) {
        cerr << "ERROR: --dram-config required" << endl;
        return 1;
    }

    // ---- Banner ----
    string modeStr;
    if (args.experimentCompete) modeStr = "Experiment: Interleave (16 PE, 2x4 mesh)";
    else if (args.experimentOpposite) modeStr = "Experiment: Channel-aware (16 PE, 2x4 mesh)";
    else if (args.modeA) modeStr = "A (all->1ch, 1xBW, 2x4 mesh)";
    else modeStr = "B (per-ch, 4xBW, 2x4 mesh)";

    int chShift = args.lpddr4 ? 30 : 12;

    cout << "\n================================================" << endl;
    cout << "  NoC (Noxim 2x4 mesh) + DRAMSys Co-Simulation" << endl;
    cout << "  Mode: " << modeStr << endl;
    cout << "  PEs: " << args.numPEs << endl;
    cout << "  Transactions/PE: " << args.nocTx << endl;
    cout << "  Clock: " << args.clockPeriod << "ns" << endl;
    cout << "  DRAM: " << (args.lpddr4 ? "LPDDR4" : "DDR4")
         << " (CH bits at [" << chShift + 1 << ":" << chShift << "])" << endl;
    if (!args.vcdFile.empty())
        cout << "  VCD trace: " << args.vcdFile << endl;
    cout << "================================================\n" << endl;

    // ---- Configure Noxim GlobalParams ----
    GlobalParams::topology = "MESH";
    GlobalParams::mesh_dim_x = 4;
    GlobalParams::mesh_dim_y = 2;
    GlobalParams::buffer_depth = 8;
    GlobalParams::flit_size = 32;
    GlobalParams::n_virtual_channels = 1;
    GlobalParams::routing_algorithm = "XY";
    GlobalParams::selection_strategy = "RANDOM";
    GlobalParams::clock_period_ps = static_cast<int>(args.clockPeriod * 1000);
    GlobalParams::use_winoc = false;
    GlobalParams::min_packet_size = 18;
    GlobalParams::max_packet_size = 18;
    GlobalParams::packet_injection_rate = 1.0;
    GlobalParams::probability_of_retransmission = 0.0;
    GlobalParams::simulation_time = args.maxCycles;
    GlobalParams::stats_warm_up_time = 0;
    GlobalParams::reset_time = 1;
    GlobalParams::max_volume_to_be_drained = 0;
    GlobalParams::verbose_mode = "VERBOSE_OFF";
    GlobalParams::log_level = "OFF";
    GlobalParams::detailed = false;

    // ---- DRAMSys ----
    DramIf::DramInterface dramIf("DramInterface", args.dramConfig, 0, chShift);
    if (!dramIf.isConfigured()) {
        cerr << "ERROR: DramInterface init failed" << endl;
        return 1;
    }
    dramIf.getDramsys()->setThreadCount(4);

    // ---- Clock & Reset ----
    sc_clock noc_clk("noc_clk", args.clockPeriod, SC_NS);
    sc_signal<bool> noc_rst("noc_rst");

    // ---- Create TrafficPEs (row 0, one per DRAM channel) ----
    int numDataPEs = 4;  // 4 PEs in a 2x4 mesh
    TrafficPE* pes[4];
    for (int pe = 0; pe < numDataPEs; ++pe) {
        uint32_t base_addr;
        int ch = pe % 4;
        bool use_interleave = args.interleave || args.experimentCompete;

        if (args.modeA) {
            base_addr = static_cast<uint32_t>(pe) * 0x10000;
        } else if (use_interleave) {
            base_addr = static_cast<uint32_t>(pe) * 0x10000;
        } else {
            int subIdx = pe / 4;
            base_addr = (static_cast<uint32_t>(ch) << chShift)
                      | (static_cast<uint32_t>(subIdx) * 0x10000);
        }

        // Experiment mode port sequences
        int port_seq[4] = {-1, -1, -1, -1};
        if (args.experimentOpposite) {
            static const int opp_seq[4][4] = {
                {0, 1, 2, 3}, {1, 2, 3, 0},
                {2, 3, 0, 1}, {3, 0, 1, 2}
            };
            for (int d = 0; d < 4; ++d)
                port_seq[d] = opp_seq[pe][d];
        }

        int data_len = args.lpddr4 ? 32 : 64;
        int force_ch = args.modeA ? 0 : -1;
        auto* p = new TrafficPE(
            sc_module_name(("PE" + to_string(pe)).c_str()),
            pe, args.nocTx, base_addr, args.nocRate, args.is_read,
            data_len, use_interleave, chShift,
            port_seq[0] >= 0 ? port_seq : nullptr,
            force_ch);

        // Enable one-shot if experiment mode
        if (args.experimentCompete || args.experimentOpposite) {
            p->enableOneShot(chShift, 64, 4, false);
        }
        pes[pe] = p;
    }

    // ---- Create DramPEs (row 1, one per channel) ----
    DramPE* drams[4];
    for (int ch = 0; ch < 4; ++ch) {
        drams[ch] = new DramPE(
            sc_module_name(("DramPE" + to_string(ch)).c_str()), ch);
        auto& sock = dramIf.getDramsys()->getArbiterTargetSocket();
        drams[ch]->bindToDramsys(&sock, ch);
    }

    // ---- Create 2x4 Noxim Mesh ----
    NocMeshWiring mesh("noc_mesh");
    mesh.create(pes, drams, noc_clk, noc_rst);

    // ---- VCD trace ----
    sc_core::sc_trace_file* vcd_tf = nullptr;
    if (!args.vcdFile.empty()) {
        vcd_tf = sc_create_vcd_trace_file(args.vcdFile.c_str());
        sc_trace(vcd_tf, noc_clk, "noc_clk");
        sc_trace(vcd_tf, noc_rst, "noc_rst");
        cout << "  [VCD trace] Writing to " << args.vcdFile << endl;
    }

    // ---- Run simulation ----
    cout << "\n--- Starting simulation ---" << endl;

    noc_rst.write(1);
    sc_start(10, SC_NS);
    noc_rst.write(0);

    sc_time poll_interval(100, SC_NS);
    sc_time timeout(args.maxCycles * args.clockPeriod, SC_NS);
    sc_time t0 = sc_time_stamp();

    while (true) {
        sc_start(poll_interval);

        // Check completion: all DramPEs received expected transactions
        int totalSent = args.nocTx * args.numPEs;
        uint64_t totalCompleted = 0;
        for (int ch = 0; ch < 4; ++ch) {
            totalCompleted += drams[ch]->completed();
        }

        if (totalCompleted >= static_cast<uint64_t>(totalSent))
            break;

        if ((sc_time_stamp() - t0) > timeout) {
            cout << "\n[TIMEOUT] at " << sc_time_stamp()
                 << " (completed " << totalCompleted << "/" << totalSent << ")"
                 << endl;
            break;
        }
    }

    // Drain pipeline
    cout << "  [Drain] Waiting for pipeline..." << endl;
    for (int drain = 0; drain < 200; ++drain) {
        sc_start(sc_time(100, SC_NS));
        bool allIdle = dramIf.getDramsys()->idle();
        bool noPending = true;
        for (int ch = 0; ch < 4; ++ch) {
            if (drams[ch]->hasPending()) { noPending = false; break; }
        }
        if (allIdle && noPending) break;
    }

    if (vcd_tf) {
        sc_close_vcd_trace_file(vcd_tf);
        cout << "  [VCD trace] Closed " << args.vcdFile << endl;
    }

    double sim_time_ns = sc_time_stamp().to_seconds() * 1e9;

    // ---- Bandwidth Report ----
    cout << "\n============ NoC (Noxim) + DRAMSys Bandwidth Report ============" << endl;
    cout << "  Mode: " << modeStr << endl;
    cout << "  Simulation time: " << fixed << setprecision(1)
         << sim_time_ns << " ns" << endl;

    uint64_t totalBytes = 0;
    for (int ch = 0; ch < 4; ++ch) {
        uint64_t chBytes = drams[ch]->bytesTransferred();
        uint64_t chTx    = drams[ch]->completed();
        double chBW = (sim_time_ns > 0) ? (chBytes / sim_time_ns) : 0.0;

        cout << "  CH" << ch << ": " << chTx << " tx, "
             << chBytes << " bytes, "
             << fixed << setprecision(2) << chBW << " GB/s" << endl;
        totalBytes += chBytes;
    }

    double totalBW = (sim_time_ns > 0) ? (totalBytes / sim_time_ns) : 0.0;
    cout << "  ----------------------------------------" << endl;
    cout << "  Total:      " << totalBytes << " bytes, "
         << fixed << setprecision(2) << totalBW << " GB/s" << endl;
    cout << "========================================================\n" << endl;

    // ---- Data consistency check ----
    {
        cout << "============ Data Consistency Check ============" << endl;
        int errors = 0;
        for (int ch = 0; ch < 4; ++ch) {
            uint32_t vpattern = 0xBEEF0000 | (ch << 8);
            uint64_t vaddr = static_cast<uint64_t>(ch) * 256;

            {
                tlm::tlm_generic_payload wtrans;
                wtrans.set_command(tlm::TLM_WRITE_COMMAND);
                uint64_t chanAddr = (static_cast<uint64_t>(ch) << chShift) | vaddr;
                wtrans.set_address(chanAddr);
                wtrans.set_data_ptr(reinterpret_cast<unsigned char*>(&vpattern));
                wtrans.set_data_length(sizeof(vpattern));
                wtrans.set_byte_enable_ptr(nullptr);
                wtrans.set_byte_enable_length(0);
                wtrans.set_dmi_allowed(false);
                sc_time d = SC_ZERO_TIME;
                dramIf.getDramsys()->b_transport(wtrans, d);
            }

            uint32_t readback = 0;
            bool ok = dramIf.verifyRead(ch, vaddr, &readback, sizeof(readback));
            if (ok && readback == vpattern) {
                cout << "  CH" << ch << " PASS" << endl;
            } else {
                cout << "  CH" << ch << " FAIL (wrote=0x" << hex << vpattern
                     << " read=0x" << readback << dec << ")" << endl;
                errors++;
            }
        }
        cout << "  Errors: " << errors << endl;
        cout << "  Overall: " << (errors == 0 ? "PASS" : "FAIL") << endl;
        cout << "================================================\n" << endl;
    }

    sc_stop();
    return 0;
}
