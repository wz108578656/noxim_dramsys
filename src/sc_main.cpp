// ============================================================================
// sc_main.cpp — NoC-Mesh + DRAMSys co-simulation top-level
// ============================================================================
// Usage:
//   Mode A (all PEs → single channel, 1× BW):
//     ./noxim_dramsys --dram-config <json> --noc-mode --noc-tx 1000 --noc-mode-a
//
//   Mode B (each PE → dedicated channel, 4× BW):
//     ./noxim_dramsys --dram-config <json> --noc-mode --noc-tx 1000 --noc-mode-b
//
//   VCD waveform trace:
//     ./noxim_dramsys --dram-config <json> --noc-mode --vcd trace.vcd
//
// Address mapping (respects DRAMSys address mapping config):
//   DDR4  (am_ddr4_4ch.json):  channel = (addr >> 12) & 0x3, bits 13:12
//   LPDDR4 (***.json): channel = (addr >> 30) & 0x3, bits 31:30
//   Mode B: CH0=0x0000, CH1=chShift<<1, CH2=chShift<<2, CH3=chShift<<3
//
// Architecture:
//   PE[0] ──► inFIFO[0] ──┐                    ┌── outFIFO[0] ──► DramCh[0] ──┐
//   PE[1] ──► inFIFO[1] ──┤  NoCXbar (SC_METHOD)├── outFIFO[1] ──► DramCh[1] ──┤
//   PE[2] ──► inFIFO[2] ──┤  每周期路由+仲裁      ├── outFIFO[2] ──► DramCh[2] ──┤  DRAMSys::tSocket
//   PE[3] ──► inFIFO[3] ──┘                    └── outFIFO[3] ──► DramCh[3] ──┘  (AT protocol)
//
//   DramChannel uses AT protocol (nb_transport_fw) directly into
//   DRAMSys::tSocket — the standard DRAMSys RequestIssuer pattern.
//   DRAMSys internal scheduler provides cycle-accurate DRAM timing
//   (tRCD, tCL, tRP, bank conflicts, refresh, etc.).
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

#include "noc_xbar.h"
#include "pe.h"
#include "dram_channel.h"
#include "DramInterface.h"

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
    string vcdFile;                // VCD trace file (empty = no trace)
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
            cout << "Usage: " << argv[0] << " --dram-config <json> --noc-mode [opts]\n"
                 << "  --noc-tx <N>       Transactions per PE (default 1000)\n"
                 << "  --noc-pe <N>       Number of PEs (default 4, max 32)\n"
                 << "  --noc-rate <ns>    Injection interval in ns (0=max)\n"
                 << "  --noc-clock <ns>   NoC clock period (default 1.0ns)\n"
                 << "  --noc-mode-a       All PEs → single channel (1× BW)\n"
                 << "  --noc-mode-b       Each PE → own channel (4× BW, default)\n"
                 << "  --lpddr4           Use LPDDR4 channel bits [31:30] (default DDR4 [13:12])\n"
                 << "  --noc-read         Use READ commands (default WRITE)\n"
                 << "  --max-cycles <N>   Max simulation cycles (default 100000)\n"
                 << "  --vcd <file>       Enable VCD waveform tracing (default off)\n"
                 << "\n  Experiment modes (8 PE, 4000 tx/PE, fixed-channel, crossbar):\n"
                 << "  --experiment-compete   Both groups target same channels (contention)\n"
                 << "  --experiment-opposite  Groups target reversed channels (staggered)\n"
                 << endl;
            exit(0);
        }
    }
    return args;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int sc_main(int argc, char** argv)
{
    Args args = parseArgs(argc, argv);

    if (args.dramConfig.empty()) {
        cerr << "ERROR: --dram-config required" << endl;
        return 1;
    }

    // ---- Header banner ----
    string modeStr;
    if (args.experimentCompete) modeStr = "Experiment: Interleave (16 PE, 4 ports)";
    else if (args.experimentOpposite) modeStr = "Experiment: Channel-aware (16 PE, 4 ports)";
    else if (args.modeA) modeStr = "A (all->1ch, 1xBW)";
    else modeStr = "B (per-ch, 4xBW)";

    cout << "\n================================================" << endl;
    cout << "  NoC + DRAMSys Co-Simulation" << endl;
    cout << "  Mode: " << modeStr << endl;
    cout << "  PEs: " << args.numPEs << endl;
    cout << "  Transactions/PE: " << args.nocTx << endl;
    cout << "  Clock: " << args.clockPeriod << "ns" << endl;

    int chShift = args.lpddr4 ? 30 : 12;
    cout << "  DRAM: AT protocol, " << (args.lpddr4 ? "LPDDR4" : "DDR4")
         << " (CH bits at [" << chShift + 1 << ":" << chShift << "])" << endl;
    if (!args.vcdFile.empty())
        cout << "  VCD trace: " << args.vcdFile << endl;
    cout << "================================================\n" << endl;

    // ---- DRAMSys ----
    DramIf::DramInterface dramIf("DramInterface", args.dramConfig, 0, chShift);
    if (!dramIf.isConfigured()) {
        cerr << "ERROR: DramInterface init failed" << endl;
        return 1;
    }
    dramIf.getDramsys()->setThreadCount(4);

    // ---- NoC Crossbar ----
    sc_clock noc_clk("noc_clk", args.clockPeriod, SC_NS);
    sc_signal<bool> noc_rst("noc_rst");

    NoCXbar xbar("NoCXbar");
    xbar.clock(noc_clk);
    xbar.reset(noc_rst);
    xbar.setChannelShift(chShift);
    if (args.modeA) xbar.setForceOutput(0);
    if (args.interleave) xbar.setInterleaveShift(8);
    if (args.experimentCompete) xbar.setInterleaveShift(8);

    // ---- DRAM Channels ----
    vector<unique_ptr<DramChannel>> dramCh;
    for (int ch = 0; ch < 4; ++ch) {
        auto dc = make_unique<DramChannel>(
            sc_module_name(("DramCh" + to_string(ch)).c_str()),
            ch, &xbar);
        dc->bindToDramsys(dramIf.getDramsys()->getArbiterTargetSocket(), ch);
        dramCh.push_back(move(dc));
    }

    // ---- PEs ----
    int pesPerChannel = args.modeA ? args.numPEs : max(1, args.numPEs / 4);
    vector<unique_ptr<PE>> pes;

    if (args.experimentCompete || args.experimentOpposite) {
        int port_seq[4][4] = {
            {0, 1, 2, 3},
            {1, 2, 3, 0},
            {2, 3, 0, 1},
            {3, 0, 1, 2}
        };
        for (int port = 0; port < 4; ++port) {
            uint32_t base_addr = static_cast<uint32_t>(port) * 0x40000;
            if (args.experimentCompete) {
                auto p = make_unique<PE>(
                    sc_module_name(("PE_port" + to_string(port)).c_str()),
                    port, port, &xbar, args.nocTx, base_addr, args.nocRate,
                    args.is_read, 64, false, chShift);
                p->enableOneShot(chShift, 64, 4, true);
                pes.push_back(move(p));
            } else {
                auto p = make_unique<PE>(
                    sc_module_name(("PE_port" + to_string(port)).c_str()),
                    port, port, &xbar, args.nocTx, base_addr, args.nocRate,
                    args.is_read, 64, false, chShift, port_seq[port]);
                p->enableOneShot(chShift, 64, 4, true);
                pes.push_back(move(p));
            }
        }
    } else {
        for (int pe = 0; pe < args.numPEs; ++pe) {
            uint32_t base_addr;
            int ch = pe % 4;

            if (args.modeA) {
                base_addr = static_cast<uint32_t>(pe) * 0x10000;
            } else if (args.interleave) {
                base_addr = static_cast<uint32_t>(pe) * 0x10000;
            } else {
                int subIdx = pe / 4;
                base_addr = (static_cast<uint32_t>(ch) << chShift)
                          | (static_cast<uint32_t>(subIdx) * 0x10000);
            }

            auto p = make_unique<PE>(
                sc_module_name(("PE" + to_string(pe)).c_str()),
                pe, pe % 4, &xbar, args.nocTx, base_addr, args.nocRate,
                args.is_read, args.lpddr4 ? 32 : 64, args.interleave, chShift);
            pes.push_back(move(p));
        }
    }

    // ---- VCD trace setup (before sc_start) ----
    sc_core::sc_trace_file* vcd_tf = nullptr;
    if (!args.vcdFile.empty()) {
        vcd_tf = sc_create_vcd_trace_file(args.vcdFile.c_str());

        // System-level signals
        sc_trace(vcd_tf, noc_clk, "noc_clk");
        sc_trace(vcd_tf, noc_rst, "noc_rst");

        // Module signals
        xbar.traceAll(vcd_tf);
        for (int ch = 0; ch < 4; ++ch)
            dramCh[ch]->traceAll(vcd_tf);

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

        bool allDone = true;
        if (args.experimentCompete || args.experimentOpposite) {
            uint64_t perChTarget = static_cast<uint64_t>(args.nocTx) * args.numPEs / 4;
            for (int ch = 0; ch < 4; ++ch) {
                if (dramCh[ch]->completed() < perChTarget)
                    allDone = false;
            }
        } else if (args.interleave) {
            uint64_t totalCompleted = 0;
            for (int ch = 0; ch < 4; ++ch)
                totalCompleted += dramCh[ch]->completed();
            if (totalCompleted < static_cast<uint64_t>(args.nocTx) * args.numPEs)
                allDone = false;
        } else if (args.experimentCompete || args.experimentOpposite) {
            for (int ch = 0; ch < 4; ++ch) {
                int expectedTx = args.modeA ? (ch == 0 ? args.nocTx * args.numPEs : 0)
                              : args.nocTx * pesPerChannel;
                if (dramCh[ch]->completed() < static_cast<uint64_t>(expectedTx))
                    allDone = false;
            }
        }
        if (allDone) break;
        if ((sc_time_stamp() - t0) > timeout) {
            cout << "\n[TIMEOUT] Simulation stopped at " << sc_time_stamp() << endl;
            break;
        }
    }

    // Drain pipeline
    cout << "  [Drain] Waiting for pipeline to drain..." << endl;
    for (int drain = 0; drain < 200; ++drain) {
        sc_start(sc_time(100, SC_NS));
        bool allIdle = dramIf.getDramsys()->idle();
        bool noPending = true;
        for (int ch = 0; ch < 4; ++ch) {
            if (dramCh[ch]->hasPending()) noPending = false;
        }
        if (allIdle && noPending) break;
    }

    // Close VCD trace file (flush before final time)
    if (vcd_tf) {
        sc_close_vcd_trace_file(vcd_tf);
        cout << "  [VCD trace] Closed " << args.vcdFile << endl;
    }

    double sim_time_ns = sc_time_stamp().to_seconds() * 1e9;

    // ---- Report ----
    cout << "\n============ NoC + DRAMSys Bandwidth Report ============" << endl;
    cout << "  Mode: " << modeStr << endl;
    cout << "  Simulation time: " << fixed << setprecision(1)
         << sim_time_ns << " ns" << endl;

    uint64_t totalBytes = 0;
    for (int ch = 0; ch < 4; ++ch) {
        uint64_t chBytes = dramCh[ch]->bytesTransferred();
        uint64_t chTx    = dramCh[ch]->completed();
        double chBW = (sim_time_ns > 0) ? (chBytes / sim_time_ns) : 0.0;

        cout << "  CH" << ch << ": " << chTx << " tx, "
             << chBytes << " bytes, "
             << fixed << setprecision(2) << chBW << " GB/s" << endl;
        totalBytes += chBytes;
    }

    if (args.experimentCompete || args.experimentOpposite) {
        cout << "  [Xbar routed] ";
        for (int out = 0; out < 4; ++out) {
            cout << "out" << out << "=" << xbar.routedCount(out) << " ";
        }
        cout << endl;
    }

    double totalBW = (sim_time_ns > 0) ? (totalBytes / sim_time_ns) : 0.0;

    cout << "  ----------------------------------------" << endl;
    cout << "  Total:      " << totalBytes << " bytes, "
         << fixed << setprecision(2) << totalBW << " GB/s" << endl;
    cout << "  (Timing: DRAMSys AT cycle-accurate — scheduler tRCD/tCL/tRP/bank conflicts)" << endl;
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
                cout << "  CH" << ch << " addr=0x" << hex << vaddr
                     << " pattern=0x" << vpattern
                     << dec << " PASS" << endl;
            } else {
                cout << "  CH" << ch << " addr=0x" << hex << vaddr
                     << " wrote=0x" << vpattern
                     << " read=0x" << readback
                     << dec << " FAIL" << endl;
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
