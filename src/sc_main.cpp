// ============================================================================
// sc_main.cpp — 4 PE → 4×8 Xbar → 8 ChannelScheduler → 8 DramPE → DRAMSys
// ============================================================================
#include <systemc.h>
#include <tlm.h>
#include <sysc/tracing/sc_trace.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <cstdlib>

#include "traffic_pe.h"
#include "xbar_4x8.h"
#include "channel_scheduler.h"
#include "dram_pe.h"
#include "DramInterface.h"

using namespace std;
using namespace sc_core;

struct Args {
    string dramConfig;
    int    nocTx         = 1000;
    int    numPEs        = 4;
    double clockPeriod   = 1.0;
    bool   modeA         = false;
    string addrMode      = "nointerleave";
    int    blockSize     = 4096;
    int    txSize        = 256;
    int    ageThreshold  = 16;
    string arbMode       = "rowhit";
    bool   is_read       = false;
    int    maxCycles     = 100000;
    string vcdFile;
};

static Args parseArgs(int argc, char** argv)
{
    Args args;
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "--dram-config" && i + 1 < argc) args.dramConfig = argv[++i];
        else if (arg == "--noc-tx" && i + 1 < argc) args.nocTx = atoi(argv[++i]);
        else if (arg == "--noc-clock" && i + 1 < argc) args.clockPeriod = atof(argv[++i]);
        else if (arg == "--addr-mode" && i + 1 < argc) args.addrMode = argv[++i];
        else if (arg == "--block-size" && i + 1 < argc) args.blockSize = atoi(argv[++i]);
        else if (arg == "--tx-size" && i + 1 < argc) args.txSize = atoi(argv[++i]);
        else if (arg == "--age-threshold" && i + 1 < argc) args.ageThreshold = atoi(argv[++i]);
        else if (arg == "--arb-mode" && i + 1 < argc) args.arbMode = argv[++i];
        else if (arg == "--noc-mode-a") args.modeA = true;
        else if (arg == "--noc-read") args.is_read = true;
        else if (arg == "--max-cycles" && i + 1 < argc) args.maxCycles = atoi(argv[++i]);
        else if (arg == "--vcd" && i + 1 < argc) args.vcdFile = argv[++i];
        else if (arg == "-h" || arg == "--help") {
            cout << "Usage: " << argv[0] << " [opts]\n"
                 << "  --dram-config <path>  DRAMSys JSON config\n"
                 << "  --noc-tx <N>          Transactions per PE\n"
                 << "  --addr-mode <mode>    nointerleave | interleave\n"
                 << "  --block-size <N>      Interleave block size\n"
                 << "  --tx-size <N>         Transaction size (default 256)\n"
                 << "  --age-threshold <N>   Anti-starvation age (default 16)\n"
                 << "  --noc-mode-a          Force all traffic to channel 0\n"
                 << "  --noc-read            READ transactions\n"
                 << "  --max-cycles <N>      Max cycles\n"
                 << "  --vcd <file>          VCD trace\n"
                 << endl;
            exit(0);
        }
    }
    return args;
}

int sc_main(int argc, char** argv)
{
    Args args = parseArgs(argc, argv);

    bool interleave = (args.addrMode == "interleave");

    string dramConfig = args.dramConfig;
    if (dramConfig.empty())
        dramConfig = "../configs/dramsys_ddr4_8ch.json";

    string modeStr = interleave ? "Interleave (block=" + to_string(args.blockSize) + "B)"
                   : args.modeA  ? "No-interleave (all->ch0)"
                   : "No-interleave (per-ch)";

    cout << "\n=== 4PE × 8ch Xbar + ChannelScheduler + DRAMSys ===" << endl;
    cout << "  Mode: " << modeStr << endl;
    cout << "  PEs: " << args.numPEs << endl;
    cout << "  Transactions/PE: " << args.nocTx << endl;
    cout << "  Config: " << dramConfig << endl;
    cout << "========================================\n" << endl;

    // ---- DRAMSys ----
    DramIf::DramInterface dramIf("DramInterface", dramConfig, 0, 12);
    if (!dramIf.isConfigured()) {
        cerr << "ERROR: DramInterface init failed" << endl;
        return 1;
    }
    dramIf.getDramsys()->setThreadCount(8);

    // ---- Clock ----
    sc_clock clk("clk", args.clockPeriod, SC_NS);
    sc_signal<bool> rst("rst");

    // ---- 4×8 Crossbar ----
    Xbar4x8 xbar("xbar");

    // ---- 8 Channel Schedulers ----
    static const int NUM_CH = 8;
    ChannelScheduler* sched[NUM_CH];
    for (int ch = 0; ch < NUM_CH; ++ch) {
        sched[ch] = new ChannelScheduler(
            sc_module_name(("SchedCh" + to_string(ch)).c_str()),
            ch, args.ageThreshold);
        sched[ch]->clock(clk);
        sched[ch]->reset(rst);
        if (args.arbMode == "rronly")
            sched[ch]->setArbMode(ChannelScheduler::RR_ONLY);
        xbar.bindScheduler(ch, sched[ch]);
    }

    // ---- 4 Traffic PEs ----
    TrafficPE* pes[4];
    int data_len = args.txSize;
    for (int pe = 0; pe < 4; ++pe) {
        uint64_t base;
        if (interleave) {
            base = static_cast<uint64_t>(pe) * 0x10000;
        } else if (args.modeA) {
            base = 0;
        } else {
            base = static_cast<uint64_t>(pe) << 29;
        }

        auto* p = new TrafficPE(
            sc_module_name(("PE" + to_string(pe)).c_str()),
            pe, args.nocTx, base, 0.0, args.is_read, data_len);
        p->bindXbar(&xbar);
        if (interleave)
            p->setAddrMode(AddrDecoder::INTERLEAVE, args.blockSize);
        else
            p->setAddrMode(AddrDecoder::NO_INTERLEAVE);
        pes[pe] = p;
    }

    // ---- 8 Dram PEs ----
    DramPE* drams[NUM_CH];
    for (int ch = 0; ch < NUM_CH; ++ch) {
        drams[ch] = new DramPE(
            sc_module_name(("DramPE" + to_string(ch)).c_str()), ch, sched[ch]);
        drams[ch]->clock(clk);
        drams[ch]->reset(rst);
        auto& sock = dramIf.getDramsys()->getArbiterTargetSocket();
        drams[ch]->bindToDramsys(&sock, ch);
    }

    // ---- VCD trace ----
    sc_core::sc_trace_file* vcd_tf = nullptr;
    if (!args.vcdFile.empty()) {
        vcd_tf = sc_create_vcd_trace_file(args.vcdFile.c_str());
        sc_trace(vcd_tf, clk, "clk");
        sc_trace(vcd_tf, rst, "rst");
        for (int pe = 0; pe < 4; ++pe) pes[pe]->traceAll(vcd_tf);
        for (int ch = 0; ch < NUM_CH; ++ch) {
            sched[ch]->traceAll(vcd_tf);
            drams[ch]->traceAll(vcd_tf);
        }
        xbar.traceAll(vcd_tf);
        cout << "  [VCD trace] Writing to " << args.vcdFile << endl;
    }

    // ---- Run ----
    cout << "\n--- Starting simulation ---" << endl;
    rst.write(1);
    sc_start(10, SC_NS);
    rst.write(0);
    sc_time t_start = sc_time_stamp();

    sc_time timeout(args.maxCycles * args.clockPeriod, SC_NS);
    while (true) {
        sc_start(sc_time(100, SC_NS));

        int totalSent = args.nocTx * args.numPEs;
        uint64_t totalCompleted = 0;
        for (int ch = 0; ch < NUM_CH; ++ch)
            totalCompleted += drams[ch]->completed();

        if (totalCompleted >= static_cast<uint64_t>(totalSent))
            break;
        if ((sc_time_stamp() - t_start) > timeout) {
            cout << "\n[TIMEOUT] at " << sc_time_stamp()
                 << " (" << totalCompleted << "/" << totalSent << ")" << endl;
            break;
        }
    }
    sc_time t_end = sc_time_stamp();
    double e2e_ns = (t_end - t_start).to_seconds() * 1e9;

    // Drain
    cout << "  [Drain]..." << endl;
    for (int d = 0; d < 200; ++d) {
        sc_start(sc_time(100, SC_NS));
        bool allIdle = dramIf.getDramsys()->idle();
        bool noPending = true;
        for (int ch = 0; ch < NUM_CH; ++ch)
            if (drams[ch]->hasPending()) { noPending = false; break; }
        if (allIdle && noPending) break;
    }

    if (vcd_tf) sc_close_vcd_trace_file(vcd_tf);

    // ---- Report ----
    cout << "\n============ Performance Report ============" << endl;
    cout << "  Mode: " << modeStr << endl;
    cout << "  E2E time: " << fixed << setprecision(0) << e2e_ns << " ns" << endl;
    int active = 0;
    uint64_t totalTx = 0;
    for (int ch = 0; ch < NUM_CH; ++ch) {
        uint64_t c = drams[ch]->completed();
        if (c > 0) active++;
        totalTx += c;
    }
    cout << "  " << totalTx << " tx, " << active << " channels active" << endl;
    for (int ch = 0; ch < NUM_CH; ++ch) {
        if (drams[ch]->completed() > 0) {
            double bw = drams[ch]->bytesTransferred() / e2e_ns;
            cout << "  CH" << ch << ": " << drams[ch]->completed()
                 << " tx, " << bw << " GB/s (raw)"
                 << "  util=";
        }
    }
    // DRAMSys controller output has real utilization
    cout << "  (DRAM bus utilization from DRAMSys controller output)" << endl;

    // ---- Data consistency ----
    {
        cout << "\n============ Data Consistency Check ============" << endl;
        int errors = 0;
        for (int ch = 0; ch < NUM_CH; ++ch) {
            uint32_t vpattern = 0xBEEF0000 | (ch << 8);
            uint64_t vaddr = static_cast<uint64_t>(ch) * 256;
            {
                tlm::tlm_generic_payload wtrans;
                wtrans.set_command(tlm::TLM_WRITE_COMMAND);
                uint64_t chanAddr = (static_cast<uint64_t>(ch) << 12) | vaddr;
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
            if (ok && readback == vpattern)
                cout << "  CH" << ch << " PASS" << endl;
            else {
                cout << "  CH" << ch << " FAIL" << endl;
                errors++;
            }
        }
        cout << "  Overall: " << (errors == 0 ? "PASS" : "FAIL") << endl;
    }

    sc_stop();
    return 0;
}
