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

    // Each output port has independent round-robin arbitration.
    // Different outputs can accept flits in the same cycle (non-blocking).
    for (int out_port = 0; out_port < XBAR_PORTS; ++out_port) {

        // Round-robin: start search from last winner + 1
        for (int attempt = 0; attempt < XBAR_PORTS; ++attempt) {
            int in_port = (m_rr[out_port] + 1 + attempt) % XBAR_PORTS;

            if (m_input[in_port].empty())
                continue;

            MemTransaction* tx = m_input[in_port].front();

            // Extract target channel from address bits [chShift+1:chShift]
            int target_ch = static_cast<int>((tx->address >> m_chShift) & 0x3);

            if (target_ch != out_port)
                continue;

            // Match found — route this flit
            m_input[in_port].pop();

            // Keep channel bits in address — DRAMSys uses them for channel decode

            m_output[out_port].push(tx);
            m_routed[out_port]++;
            m_rr[out_port] = in_port;  // remember winner for next cycle

            // Only 1 flit per input per cycle, so stop searching this output
            break;
        }
    }
}
