// ============================================================================
// noc_xbar.h — Cycle-accurate 4x4 non-blocking crossbar
// ============================================================================
// Clock-driven SC_METHOD. Each cycle:
//   - Scans 4 input FIFOs (max 1 flit per input per cycle)
//   - Extracts target channel from address bits
//   - Per-input serving: each input pops one transaction per cycle
//   - Rotating in_start to prevent input starvation
// ============================================================================
#ifndef NOC_XBAR_H
#define NOC_XBAR_H

#include <systemc.h>
#include <queue>
#include <cstdint>

static const int XBAR_PORTS = 4;
static const int XBAR_FIFO_DEPTH = 64;

// ---------------------------------------------------------------------------
// Memory transaction descriptor (passes through NoC)
// ---------------------------------------------------------------------------
struct MemTransaction {
    uint64_t address;       // full address (channel encoded at CH bits)
    bool     is_write;
    uint32_t data[16];      // 64 bytes max
    uint8_t  data_len;
    int      pe_id;
    int      tag;           // transaction sequence id
};

// ---------------------------------------------------------------------------
// NoCXbar — 4x4 crossbar switch
// ---------------------------------------------------------------------------
SC_MODULE(NoCXbar)
{
public:
    SC_HAS_PROCESS(NoCXbar);

    // Clock and reset
    sc_in_clk   clock;
    sc_in<bool> reset;

    NoCXbar(sc_module_name name);

    // ---- PE interface (called from PE SC_THREAD) ----
    bool inputFull(int pe_id) const;
    void pushInput(int pe_id, MemTransaction* tx);

    // ---- DramChannel interface (called from DramChannel SC_THREAD) ----
    bool popOutput(int channel, MemTransaction*& tx);

    // Configurable channel bit position (set before simulation)
    void setChannelShift(int shift) { m_chShift = shift; m_sig_ch_shift.write(shift); }
    int  channelShift() const { return m_chShift; }

    // Force all traffic to a single output port (Mode A: baseline BW test)
    void setForceOutput(int port) { m_forceOutput = port; m_forceEnable = true; m_sig_force_enable.write(true); }
    void clearForceOutput()      { m_forceEnable = false; m_sig_force_enable.write(false); }

    // Interleave: address-based channel selection using low address bits.
    void setInterleaveShift(int shift) { m_ilShift = shift; m_interleave = (shift >= 0); }

    // Statistics
    uint64_t routedCount(int channel) const { return m_routed[channel]; }
    uint64_t inputPopCount(int port) const { return m_input_pops[port]; }

    // ---- VCD trace ----
    void traceAll(sc_core::sc_trace_file* tf) const;

private:
    void routeProcess();   // SC_METHOD: 1 cycle of routing

    // 4 input FIFOs (PE pushes, routeProcess pops)
    std::queue<MemTransaction*> m_input[XBAR_PORTS];

    // 4 output FIFOs (routeProcess pushes, DramChannel pops)
    std::queue<MemTransaction*> m_output[XBAR_PORTS];

    // Round-robin pointer per output port
    int  m_rr[XBAR_PORTS];

    // Channel bit shift for routing (DDR4=12, LPDDR4=30)
    int  m_chShift = 28;
    int  m_ilShift = 8;
    bool m_interleave = false;

    // Force-output mode (Mode A: all traffic to one channel)
    int  m_forceOutput = 0;
    bool m_forceEnable = false;

    // Per-output routed counter / per-input pop counter
    uint64_t m_routed[XBAR_PORTS];
    uint64_t m_input_pops[XBAR_PORTS];

    // ==================================================================
    // VCD trace signals

    // Layer 1: PE → NoC input port signals (updated on pushInput)
    sc_signal<int>      m_sig_in_fifo_depth[XBAR_PORTS];
    sc_signal<uint64_t> m_sig_inject_addr[XBAR_PORTS];
    sc_signal<int>      m_sig_inject_cmd[XBAR_PORTS];   // 0=WRITE, 1=READ
    sc_signal<uint64_t> m_sig_inject_data_lo[XBAR_PORTS];
    sc_signal<uint64_t> m_sig_inject_data_hi[XBAR_PORTS];
    sc_signal<int>      m_sig_inject_id[XBAR_PORTS];

    // Layer 2: Crossbar internal
    sc_signal<int>      m_sig_out_fifo_depth[XBAR_PORTS];
    sc_signal<uint64_t> m_sig_routed[XBAR_PORTS];
    sc_signal<int>      m_sig_in_start;
    sc_signal<bool>     m_sig_force_enable;
    sc_signal<int>      m_sig_ch_shift;
};

#endif // NOC_XBAR_H
