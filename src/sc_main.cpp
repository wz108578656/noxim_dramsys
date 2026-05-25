// ============================================================================
// sc_main.cpp — NoC (Noxim 2x4 mesh) + DRAMSys co-simulation
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
    int    nocTx         = 1000;
    int    numPEs        = 4;
    double nocRate       = 0.0;
    double clockPeriod   = 1.0;
    bool   modeA         = false;   // force all traffic to ch0
    string addrMode      = "nointerleave";
    int    blockSize     = 4096;
    int    txSize        = 256;
    bool   lpddr4        = false;
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
        else if (arg == "--noc-pe" && i + 1 < argc) args.numPEs = atoi(argv[++i]);
        else if (arg == "--noc-rate" && i + 1 < argc) args.nocRate = atof(argv[++i]);
        else if (arg == "--noc-clock" && i + 1 < argc) args.clockPeriod = atof(argv[++i]);
        else if (arg == "--addr-mode" && i + 1 < argc) args.addrMode = argv[++i];
        else if (arg == "--pe-count" && i + 1 < argc) args.numPEs = atoi(argv[++i]);
        else if (arg == "--block-size" && i + 1 < argc) args.blockSize = atoi(argv[++i]);
        else if (arg == "--tx-size" && i + 1 < argc) args.txSize = atoi(argv[++i]);
        else if (arg == "--noc-mode-a") args.modeA = true;
        else if (arg == "--lpddr4") args.lpddr4 = true;
        else if (arg == "--noc-read") args.is_read = true;
        else if (arg == "--max-cycles" && i + 1 < argc) args.maxCycles = atoi(argv[++i]);
        else if (arg == "--vcd" && i + 1 < argc) args.vcdFile = argv[++i];
        else if (arg == "-h" || arg == "--help") {
            cout << "Usage: " << argv[0] << " [opts]\n"
                 << "  --dram-config <path>  DRAMSys JSON config\n"
                 << "  --noc-tx <N>          Transactions per PE (default 1000)\n"
                 << "  --noc-clock <ns>      NoC clock period (default 1.0ns)\n"
                 << "  --addr-mode <mode>    nointerleave | interleave\n"
                 << "  --block-size <N>      Interleave block size in bytes (default 4096)\n"
                 << "  --noc-mode-a          Force all traffic to channel 0\n"
                 << "  --lpddr4              LPDDR4 CH bits [31:30]\n"
                 << "  --noc-read            READ transactions (default WRITE)\n"
                 << "  --max-cycles <N>      Max cycles (default 100000)\n"
                 << "  --vcd <file>          VCD trace\n"
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

    // ---- Address mode setup ----
    bool interleave = (args.addrMode == "interleave");
    AddrDecoder::Mode addrMode = interleave ? AddrDecoder::INTERLEAVE
                                            : AddrDecoder::NO_INTERLEAVE;

    // For DDR4/LPDDR4, the channel bit position differs
    int chShift = args.lpddr4 ? 30 : 12;

    // Default DRAMSys config (DDR4 4ch, CHANNEL_BIT at [12,13])
    // DramPE translates the address to put channel at [12,13] before sending.
    string dramConfig = args.dramConfig;
    if (dramConfig.empty())
        dramConfig = "../configs/dramsys_ddr4_8ch.json";

    // ---- Banner ----
    string modeStr = interleave ? "Interleave (block=" + to_string(args.blockSize) + "B)"
                   : args.modeA  ? "No-interleave (all->ch0)"
                   : "No-interleave (per-ch)";

    cout << "\n================================================" << endl;
    cout << "  NoC (Noxim 3x4 mesh) + DRAMSys" << endl;
    cout << "  Mode: " << modeStr << endl;
    cout << "  PEs: " << args.numPEs << endl;
    cout << "  Transactions/PE: " << args.nocTx << endl;
    cout << "  Clock: " << args.clockPeriod << "ns" << endl;
    cout << "  DRAM: " << (args.lpddr4 ? "LPDDR4" : "DDR4") << endl;
    cout << "  Config: " << dramConfig << endl;
    cout << "================================================\n" << endl;

    // ---- Noxim GlobalParams ----
    GlobalParams::topology = "MESH";
    GlobalParams::mesh_dim_x = 4;
    GlobalParams::mesh_dim_y = 3;      // 3 rows: PE + 2 DRAM rows
    GlobalParams::buffer_depth = 8;
    GlobalParams::flit_size = 1024;  // 128B = 1024 bits
    GlobalParams::n_virtual_channels = 1;
    GlobalParams::routing_algorithm = "XY";
    GlobalParams::selection_strategy = "RANDOM";
    GlobalParams::clock_period_ps = static_cast<int>(args.clockPeriod * 1000);
    GlobalParams::use_winoc = false;
    GlobalParams::min_packet_size = 2;       // HEAD + TAIL (128B)
    GlobalParams::max_packet_size = 2;       // will be recalculated per tx_size
    // Recalculate: ceil(txSize/128) + 2  (HEAD + DATA flits + TAIL)
    GlobalParams::max_packet_size = 2 + (args.txSize - 1) / 128;
    GlobalParams::min_packet_size = GlobalParams::max_packet_size;
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
    DramIf::DramInterface dramIf("DramInterface", dramConfig, 0, chShift);
    if (!dramIf.isConfigured()) {
        cerr << "ERROR: DramInterface init failed" << endl;
        return 1;
    }
    dramIf.getDramsys()->setThreadCount(8);

    // ---- Clock & Reset ----
    sc_clock noc_clk("noc_clk", args.clockPeriod, SC_NS);
    sc_signal<bool> noc_rst("noc_rst");

    // ---- Create TrafficPEs (row 0) ----
    TrafficPE* pes[4];
    int data_len = args.txSize;

    for (int pe = 0; pe < 4; ++pe) {
        // Flat base address per mode
        uint64_t base;
        bool pe_active = (pe < args.numPEs);
        int pe_tx = pe_active ? args.nocTx : 0;

        if (interleave) {
            base = static_cast<uint64_t>(pe) * 0x10000;
        } else if (args.modeA) {
            base = 0;
        } else {
            base = static_cast<uint64_t>(pe) << 29;
        }

        auto* p = new TrafficPE(
            sc_module_name(("PE" + to_string(pe)).c_str()),
            pe, pe_tx, base, args.nocRate, args.is_read, data_len);

        if (interleave)
            p->setAddrMode(AddrDecoder::INTERLEAVE, args.blockSize);
        else
            p->setAddrMode(AddrDecoder::NO_INTERLEAVE);

        pes[pe] = p;
    }

    // ---- Create DramPEs (rows 1-2, ch0..7) ----
    static const int NUM_CH = 8;
    DramPE* drams[NUM_CH] = {};
    for (int ch = 0; ch < NUM_CH; ++ch) {
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

        // Module-level traces
        for (int pe = 0; pe < 4; ++pe)
            pes[pe]->traceAll(vcd_tf);
        for (int ch = 0; ch < NUM_CH; ++ch)
            drams[ch]->traceAll(vcd_tf);

        cout << "  [VCD trace] Writing to " << args.vcdFile << endl;
    }

    // ---- Run simulation ----
    cout << "\n--- Starting simulation ---" << endl;

    noc_rst.write(1);
    sc_start(10, SC_NS);
    noc_rst.write(0);
    sc_time t_start = sc_time_stamp();           // E2E: first request starts here

    sc_time poll_interval(100, SC_NS);
    sc_time timeout(args.maxCycles * args.clockPeriod, SC_NS);
    sc_time t0 = sc_time_stamp();

    while (true) {
        sc_start(poll_interval);

        int totalSent = args.nocTx * args.numPEs;
        uint64_t totalCompleted = 0;
        for (int ch = 0; ch < NUM_CH; ++ch)
            totalCompleted += drams[ch]->completed();

        if (totalCompleted >= static_cast<uint64_t>(totalSent))
            break;

        if ((sc_time_stamp() - t0) > timeout) {
            cout << "\n[TIMEOUT] at " << sc_time_stamp()
                 << " (" << totalCompleted << "/" << totalSent << ")" << endl;
            break;
        }
    }
    sc_time t_end = sc_time_stamp();             // E2E: last response received
    double e2e_ns = (t_end - t_start).to_seconds() * 1e9;

    // Drain
    cout << "  [Drain] Waiting..." << endl;
    for (int drain = 0; drain < 200; ++drain) {
        sc_start(sc_time(100, SC_NS));
        bool allIdle = dramIf.getDramsys()->idle();
        bool noPending = true;
        for (int ch = 0; ch < NUM_CH; ++ch) {
            if (drams[ch]->hasPending()) { noPending = false; break; }
        }
        if (allIdle && noPending) break;
    }

    if (vcd_tf) sc_close_vcd_trace_file(vcd_tf);

    // DDR4-1866 ×64: 1866 MT/s × 8B = 14.9 GB/s per channel (DRAM bus max)
    const double BUS_GBS_PER_CH = 14.9;

    // ---- Bandwidth Report ----
    // E2E BW = total_bytes / (last_resp - first_req). DDR4-1866 ×64 bus limit
    // is 14.9 GB/s/ch. When E2E BW exceeds the bus limit, the DRAM bus is
    // saturated; actual utilization visible in DRAMSys controller output.
    cout << "\n============ Bandwidth Report ============" << endl;
    cout << "  Mode: " << modeStr << endl;
    cout << "  E2E time: " << fixed << setprecision(1) << e2e_ns << " ns" << endl;

    uint64_t totalBytes = 0;
    int active_channels = 0;
    int tx_size = 0;
    for (int ch = 0; ch < NUM_CH; ++ch) {
        uint64_t chBytes = drams[ch]->bytesTransferred();
        uint64_t chTx    = drams[ch]->completed();
        if (chTx == 0) continue;
        active_channels++;

        int txb = static_cast<int>(chBytes / chTx);
        if (tx_size == 0) tx_size = txb;

        double chBW = (e2e_ns > 0) ? (chBytes / e2e_ns) : 0.0;
        cout << "  CH" << ch << ": " << chTx << " tx, "
             << fixed << setprecision(1) << chBW << " GB/s";
        if (chBW > BUS_GBS_PER_CH)
            cout << "  (bus sat, >" << BUS_GBS_PER_CH << ")";
        cout << endl;
        totalBytes += chBytes;
    }

    if (active_channels > 0) {
        double totalBW = totalBytes / e2e_ns;
        cout << "  Total: " << fixed << setprecision(1) << totalBW << " GB/s"
         << "  tx=" << tx_size << "B"
         << "  ch=" << active_channels << endl;
    }
    cout << "==========================================\n" << endl;

    // ---- Data consistency check ----
    {
        cout << "============ Data Consistency Check ============" << endl;
        int errors = 0;
        for (int ch = 0; ch < NUM_CH; ++ch) {
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
                cout << "  CH" << ch << " FAIL (0x" << hex << vpattern
                     << " vs 0x" << readback << dec << ")" << endl;
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
