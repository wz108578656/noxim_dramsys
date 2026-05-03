// ============================================================================
// noc_xbar.cpp — 4×4 non-blocking crossbar: routing + round-robin
// ============================================================================
#include "noc_xbar.h"
#include <iostream>
#include <cstring>

using namespace std;

NoCXbar::NoCXbar(sc_module_name name)
    : sc_module(name)
{
    for (int i = 0; i < XBAR_PORTS; ++i) {
        m_rr[i]     = XBAR_PORTS - 1;  // start from 0 after first wrap
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

        // Route this transaction
        m_input[in_port].pop();

        // Rewrite channel bits for DRAMSys decode
        if (m_forceEnable || m_interleave) {
            tx->address &= ~(0x3ULL << m_chShift);
            tx->address |= (static_cast<uint64_t>(target_ch) << m_chShift);
        }

        m_output[target_ch].push(tx);
        m_routed[target_ch]++;
        m_input_pops[in_port]++;  // debug
    }

    // Rotate input start for next cycle
    in_start = (in_start + 1) % XBAR_PORTS;
}
