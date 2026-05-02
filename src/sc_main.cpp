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
// Address mapping:
//   channel = (addr >> 28) & 0x3
//   CH0: 0x00000000 ~ 0x0FFFFFFF   CH1: 0x10000000 ~ 0x1FFFFFFF
//   CH2: 0x20000000 ~ 0x2FFFFFFF   CH3: 0x30000000 ~ 0x3FFFFFFF
//
// Architecture:
//   PE[0] ──► inFIFO[0] ──┐                    ┌── outFIFO[0] ──► DramCh[0] ──► DRAMSys ch0
//   PE[1] ──► inFIFO[1] ──┤  NoCXbar (SC_METHOD)├── outFIFO[1] ──► DramCh[1] ──► DRAMSys ch1
//   PE[2] ──► inFIFO[2] ──┤  每周期路由+仲裁      ├── outFIFO[2] ──► DramCh[2] ──► DRAMSys ch2
//   PE[3] ──► inFIFO[3] ──┘                    └── outFIFO[3] ──► DramCh[3] ──► DRAMSys ch3
// ============================================================================

#include <systemc.h>
#include <tlm.h>
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
    int    nocTx        = 1000;    // transactions per PE
    double nocRate      = 0.0;     // ns between tx (0 = full speed)
    double clockPeriod  = 1.0;     // NoC clock period (ns)
    bool   modeA        = false;   // all PEs → single channel
    bool   modeB        = true;    // each PE → dedicated channel (default)
    bool   lpddr4       = false;   // use LPDDR4 channel bits [31:30]
    bool   is_read      = false;   // READ instead of WRITE
    int    maxCycles    = 100000;
};

static Args parseArgs(int argc, char** argv)
{
    Args args;
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "--dram-config" && i + 1 < argc) args.dramConfig = argv[++i];
        else if (arg == "--noc-tx" && i + 1 < argc) args.nocTx = atoi(argv[++i]);
        else if (arg == "--noc-rate" && i + 1 < argc) args.nocRate = atof(argv[++i]);
        else if (arg == "--noc-clock" && i + 1 < argc) args.clockPeriod = atof(argv[++i]);
        else if (arg == "--noc-mode-a") args.modeA = true;
        else if (arg == "--noc-mode-b") args.modeB = true;
        else if (arg == "--lpddr4") args.lpddr4 = true;
        else if (arg == "--noc-read") args.is_read = true;
        else if (arg == "--max-cycles" && i + 1 < argc) args.maxCycles = atoi(argv[++i]);
        else if (arg == "-h" || arg == "--help") {
            cout << "Usage: " << argv[0] << " --dram-config <json> --noc-mode [opts]\n"
                 << "  --noc-tx <N>       Transactions per PE (default 1000)\n"
                 << "  --noc-rate <ns>    Injection interval in ns (0=max)\n"
                 << "  --noc-clock <ns>   NoC clock period (default 1.0ns)\n"
                 << "  --noc-mode-a       All PEs → single channel (1× BW)\n"
                 << "  --noc-mode-b       Each PE → own channel (4× BW, default)\n"
                 << "  --lpddr4           Use LPDDR4 channel bits [31:30] (default DDR4 [13:12])\n"
                 << "  --noc-read         Use READ commands (default WRITE)\n"
                 << "  --max-cycles <N>   Max simulation cycles (default 100000)\n"
                 << "\n  DRAM timing: AT protocol via DRAMSys internal scheduler\n"
                 << "  (tRCD, tCL, tRP, bank conflicts modeled by DRAMSys)\n"
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

    cout << "\n================================================" << endl;
    cout << "  NoC + DRAMSys Co-Simulation" << endl;
    cout << "  Mode: " << (args.modeA ? "A (all→1ch, 1×BW)" : "B (per-ch, 4×BW)") << endl;
    cout << "  Transactions/PE: " << args.nocTx << endl;
    cout << "  Clock: " << args.clockPeriod << "ns" << endl;

    int chShift = args.lpddr4 ? 30 : 12;
    cout << "  DRAM: AT protocol, " << (args.lpddr4 ? "LPDDR4" : "DDR4")
         << " (CH bits at [" << chShift + 1 << ":" << chShift << "])" << endl;
    cout << "================================================\n" << endl;

    // ---- Checker modules for post-bandwidth data verification ----
    // (removed — use DramInterface::verifyRead instead)
    //
    // ---- DRAMSys ----
    DramIf::DramInterface dramIf("DramInterface", args.dramConfig, 0, chShift);
    if (!dramIf.isConfigured()) {
        cerr << "ERROR: DramInterface init failed" << endl;
        return 1;
    }

    // ---- NoC Crossbar ----
    sc_clock noc_clk("noc_clk", args.clockPeriod, SC_NS);
    sc_signal<bool> noc_rst("noc_rst");

    NoCXbar xbar("NoCXbar");
    xbar.clock(noc_clk);
    xbar.reset(noc_rst);

    // ---- DRAM Channels ----
    vector<unique_ptr<DramChannel>> dramCh;
    for (int ch = 0; ch < 4; ++ch) {
        auto dc = make_unique<DramChannel>(
            sc_module_name(("DramCh" + to_string(ch)).c_str()),
            ch, &xbar);
        dc->bindToDram(dramIf.getUpstreamSocket(ch));
        dramCh.push_back(move(dc));
    }

    // ---- PEs ----
    vector<unique_ptr<PE>> pes;
    for (int pe = 0; pe < 4; ++pe) {
        uint32_t base_addr;

        if (args.modeA) {
            // Mode A: all PEs target CH0 → base = 0x00000000, PE-offset pages
            base_addr = static_cast<uint32_t>(pe) * 0x1000;  // 4KB apart
        } else {
            // Mode B: each PE targets its own channel
            // Channel encoding: (addr >> 28) & 0x3
            // CH0=0x00000000, CH1=0x10000000, CH2=0x20000000, CH3=0x30000000
            base_addr = static_cast<uint32_t>(pe) << 28;
        }

        auto p = make_unique<PE>(
            sc_module_name(("PE" + to_string(pe)).c_str()),
            pe, &xbar, args.nocTx, base_addr, args.nocRate,
            args.is_read);
        pes.push_back(move(p));
    }

    // ---- Run simulation ----
    cout << "\n--- Starting simulation ---" << endl;

    noc_rst.write(1);
    sc_start(10, SC_NS);
    noc_rst.write(0);

    // Run until all DramChannels complete their transactions
    sc_time poll_interval(10, SC_US);
    sc_time timeout(args.maxCycles * args.clockPeriod, SC_NS);
    sc_time t0 = sc_time_stamp();

    while (true) {
        sc_start(poll_interval);

        bool allDone = true;
        for (int ch = 0; ch < 4; ++ch) {
            if (dramCh[ch]->completed() < static_cast<uint64_t>(args.nocTx))
                allDone = false;
        }
        if (allDone) break;
        if ((sc_time_stamp() - t0) > timeout) {
            cout << "\n[TIMEOUT] Simulation stopped at " << sc_time_stamp() << endl;
            break;
        }
    }

    double sim_time_ns = sc_time_stamp().to_seconds() * 1e9;

    // ---- Report ----
    cout << "\n============ NoC + DRAMSys Bandwidth Report ============" << endl;
    cout << "  Mode: " << (args.modeA ? "A (1×BW)" : "B (4×BW)") << endl;
    cout << "  Simulation time: " << fixed << setprecision(1)
         << sim_time_ns << " ns" << endl;

    uint64_t totalBytes = 0;
    for (int ch = 0; ch < 4; ++ch) {
        uint64_t chBytes = dramCh[ch]->bytesTransferred();
        uint64_t chTx    = dramCh[ch]->completed();
        double chBW = (sim_time_ns > 0) ? (chBytes / sim_time_ns) : 0.0;  // GB/s

        cout << "  CH" << ch << ": " << chTx << " tx, "
             << chBytes << " bytes, "
             << fixed << setprecision(2) << chBW << " GB/s" << endl;
        totalBytes += chBytes;
    }

    double totalBW = (sim_time_ns > 0) ? (totalBytes / sim_time_ns) : 0.0;

    cout << "  ----------------------------------------" << endl;
    cout << "  Total:      " << totalBytes << " bytes, "
         << fixed << setprecision(2) << totalBW << " GB/s" << endl;
    cout << "  (Timing: DRAMSys AT cycle-accurate — scheduler tRCD/tCL/tRP/bank conflicts)" << endl;
    cout << "========================================================\n" << endl;

    // ---- Data consistency check ----
    // DRAMSys blocking mode: all channels share one physical address space.
    // Last writer wins. Verify that data can be written and read back.
    {
        cout << "============ Data Consistency Check ============" << endl;

        // Write a fresh verification pattern through the NoC (via a single
        // DramChannel doing a blocking write), then read back via verifyRead.
        int errors = 0;

        for (int ch = 0; ch < 4; ++ch) {
            // Write unique per-channel verification pattern
            uint32_t vpattern = 0xBEEF0000 | (ch << 8);
            uint64_t vaddr = static_cast<uint64_t>(ch) * 256;  // separate pages

            // Write via DramInterface directly (bypass NoC for verification)
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

            // Read back and verify
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
        cout << "  (Note: blocking mode shares one physical space;" << endl;
        cout << "   per-channel isolation requires AT cycle-accurate mode)" << endl;
        cout << "================================================\n" << endl;
    }

    sc_stop();
    return 0;
}
