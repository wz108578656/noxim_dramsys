// ============================================================================
// noc_xbar.h — Cycle-accurate 4×4 non-blocking crossbar
// ============================================================================
// Clock-driven SC_METHOD. Each cycle:
//   - Scans 4 input FIFOs (max 1 flit per input per cycle)
//   - Extracts target channel from address bits [29:28]
//   - Round-robin arbitration per output port
//   - Different output ports proceed in parallel (non-blocking)
//   - Strips channel bits from address before forwarding
// ============================================================================
#ifndef NOC_XBAR_H
#define NOC_XBAR_H

#include <systemc.h>
#include <queue>
#include <cstdint>
#include <mutex>

static const int XBAR_PORTS = 4;
static const int XBAR_FIFO_DEPTH = 64;

// ---------------------------------------------------------------------------
// Memory transaction descriptor (passes through NoC)
// ---------------------------------------------------------------------------
struct MemTransaction {
    uint64_t address;       // full address (channel encoded at [29:28])
    bool     is_write;
    uint32_t data[16];      // 64 bytes max
    uint8_t  data_len;
    int      pe_id;
    int      tag;
};

// ---------------------------------------------------------------------------
// NoCXbar — 4×4 crossbar switch
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
    void setChannelShift(int shift) { m_chShift = shift; }
    int  channelShift() const { return m_chShift; }

    // Force all traffic to a single output port (Mode A: baseline BW test)
    void setForceOutput(int port) { m_forceOutput = port; m_forceEnable = true; }
    void clearForceOutput()      { m_forceEnable = false; }

    // Statistics
    uint64_t routedCount(int channel) const { return m_routed[channel]; }

private:
    void routeProcess();   // SC_METHOD: 1 cycle of routing

    // 4 input FIFOs (PE pushes, routeProcess pops)
    std::queue<MemTransaction*> m_input[XBAR_PORTS];

    // 4 output FIFOs (routeProcess pushes, DramChannel pops)
    std::queue<MemTransaction*> m_output[XBAR_PORTS];

    // Round-robin pointer per output port
    int  m_rr[XBAR_PORTS];

    // Channel bit shift for routing (DDR4=12, LPDDR4=30)
    int  m_chShift = 28;  // default: legacy [29:28]

    // Force-output mode (Mode A: all traffic to one channel)
    int  m_forceOutput = 0;
    bool m_forceEnable = false;

    // Per-output routed counter
    uint64_t m_routed[XBAR_PORTS];
};

#endif // NOC_XBAR_H
