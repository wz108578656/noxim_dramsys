// ============================================================================
// noc_xbar.cpp — 4x4 non-blocking crossbar: routing + round-robin
// ============================================================================
#include "noc_xbar.h"
#include <iostream>
#include <cstring>

using namespace std;

NoCXbar::NoCXbar(sc_module_name name)
    : sc_module(name)
{
    for (int i = 0; i < XBAR_PORTS; ++i) {
        m_rr[i]     = XBAR_PORTS - 1;
        m_routed[i] = 0;
        m_input_pops[i] = 0;
    }

    SC_METHOD(routeProcess);
    sensitive << reset;
    sensitive << clock.pos();

    cout << "[NoCXbar] 4x4 non-blocking crossbar created"
         << " (channel bits configurable, default addr[" << (m_chShift+1) << ":" << m_chShift << "])" << endl;
}

// ============================================================================
// PE interface (called from SC_THREAD context)
// ============================================================================

bool NoCXbar::inputFull(int pe_id) const
{
    return m_input[pe_id].size() >= XBAR_FIFO_DEPTH;
}

void NoCXbar::pushInput(int pe_id, MemTransaction* tx)
{
    m_input[pe_id].push(tx);

    // ---- VCD signals: input port ----
    m_sig_in_fifo_depth[pe_id].write(static_cast<int>(m_input[pe_id].size()));
    m_sig_inject_addr[pe_id].write(tx->address);
    m_sig_inject_cmd[pe_id].write(tx->is_write ? 0 : 1);
    // First 16 bytes of data → 2 uint64_t
    uint64_t lo = (static_cast<uint64_t>(tx->data[1]) << 32) | tx->data[0];
    uint64_t hi = (static_cast<uint64_t>(tx->data[3]) << 32) | tx->data[2];
    m_sig_inject_data_lo[pe_id].write(lo);
    m_sig_inject_data_hi[pe_id].write(hi);
    m_sig_inject_id[pe_id].write(tx->tag);
}

// ============================================================================
// DramChannel interface (called from SC_THREAD context)
// ============================================================================

bool NoCXbar::popOutput(int channel, MemTransaction*& tx)
{
    if (m_output[channel].empty())
        return false;
    tx = m_output[channel].front();
    m_output[channel].pop();
    m_sig_out_fifo_depth[channel].write(static_cast<int>(m_output[channel].size()));
    return true;
}

// ============================================================================
// Core routing: SC_METHOD, triggered every clock cycle
// ============================================================================
void NoCXbar::routeProcess()
{
    if (reset.read())
        return;

    // Per-input serving: each input port pops one transaction per cycle
        // and routes to its target output channel.
        // This is strictly fair: every input gets exactly one service per cycle.
        // No output priority bias (unlike output-scan which favors lower outputs).
        //
        // Backpressure: if target output FIFO is full, skip this input port.
        // This propagates DRAM saturation back through the NoC to the PE.
        static int in_start = 0;
        for (int s = 0; s < XBAR_PORTS; ++s) {
            int in_port = (in_start + s) % XBAR_PORTS;

            if (m_input[in_port].empty())
                continue;

            MemTransaction* tx = m_input[in_port].front();

            // Determine target channel
            int target_ch;
            if (m_forceEnable) {
                target_ch = m_forceOutput;
            } else if (m_interleave) {
                target_ch = static_cast<int>((tx->address >> m_ilShift) & 0x3);
            } else {
                target_ch = static_cast<int>((tx->address >> m_chShift) & 0x3);
            }

            // Backpressure: skip if target output FIFO is full
            if (m_output[target_ch].size() >= XBAR_FIFO_DEPTH) {
                continue;
            }

            // Pop and route
            m_input[in_port].pop();
        m_sig_in_fifo_depth[in_port].write(static_cast<int>(m_input[in_port].size()));

        // Rewrite channel bits for DRAMSys decode
        if (m_forceEnable || m_interleave) {
            tx->address &= ~(0x3ULL << m_chShift);
            tx->address |= (static_cast<uint64_t>(target_ch) << m_chShift);
        }

        m_output[target_ch].push(tx);
        m_sig_out_fifo_depth[target_ch].write(static_cast<int>(m_output[target_ch].size()));

        m_routed[target_ch]++;
        m_sig_routed[target_ch].write(m_routed[target_ch]);

        m_input_pops[in_port]++;
    }

    // Rotate input start for next cycle
    in_start = (in_start + 1) % XBAR_PORTS;
    m_sig_in_start.write(in_start);
}

// ============================================================================
// VCD trace: register all signals
// ============================================================================
void NoCXbar::traceAll(sc_core::sc_trace_file* tf) const
{
    char buf[64];
    for (int i = 0; i < XBAR_PORTS; ++i) {
        snprintf(buf, sizeof(buf), "NoCXbar.in_fifo_depth_%d", i);
        sc_core::sc_trace(tf, m_sig_in_fifo_depth[i], buf);
        snprintf(buf, sizeof(buf), "NoCXbar.inject_addr_%d", i);
        sc_core::sc_trace(tf, m_sig_inject_addr[i], buf);
        snprintf(buf, sizeof(buf), "NoCXbar.inject_cmd_%d", i);
        sc_core::sc_trace(tf, m_sig_inject_cmd[i], buf);
        snprintf(buf, sizeof(buf), "NoCXbar.inject_data_lo_%d", i);
        sc_core::sc_trace(tf, m_sig_inject_data_lo[i], buf);
        snprintf(buf, sizeof(buf), "NoCXbar.inject_data_hi_%d", i);
        sc_core::sc_trace(tf, m_sig_inject_data_hi[i], buf);
        snprintf(buf, sizeof(buf), "NoCXbar.inject_id_%d", i);
        sc_core::sc_trace(tf, m_sig_inject_id[i], buf);

        snprintf(buf, sizeof(buf), "NoCXbar.out_fifo_depth_%d", i);
        sc_core::sc_trace(tf, m_sig_out_fifo_depth[i], buf);
        snprintf(buf, sizeof(buf), "NoCXbar.routed_%d", i);
        sc_core::sc_trace(tf, m_sig_routed[i], buf);
    }
    sc_core::sc_trace(tf, m_sig_in_start, "NoCXbar.in_start");
    sc_core::sc_trace(tf, m_sig_force_enable, "NoCXbar.force_enable");
    sc_core::sc_trace(tf, m_sig_ch_shift, "NoCXbar.ch_shift");
}
