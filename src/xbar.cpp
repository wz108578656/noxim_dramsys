// ============================================================================
// xbar.cpp — 4×N crossbar: PE queue + per-channel clock-edge RR arbiter
// ============================================================================
#include "xbar.h"
#include <cstring>

using namespace std;

Xbar::Xbar(sc_module_name name)
    : sc_module(name)
{
    SC_METHOD(resetArbitrate); sensitive << reset;

    SC_METHOD(arbitrateAll); sensitive << clock.pos();
    dont_initialize();
}

void Xbar::bindScheduler(int ch, ChannelScheduler* sched)
{
    if (ch >= 0 && ch < NUM_CHANNELS) m_sched[ch] = sched;
}

// ---------------------------------------------------------------------------
// route — called by PE. Write to per-PE-per-channel queue if depth allows.
// ---------------------------------------------------------------------------
bool Xbar::route(int src_pe, const ReqEntry& req)
{
    if (src_pe < 0 || src_pe >= 4) return false;
    int ch = req.channel;
    if (ch < 0 || ch >= NUM_CHANNELS) return false;
    if (static_cast<int>(m_pe_buf[src_pe][ch].size()) >= m_max_qdepth)
        return false;
    m_pe_buf[src_pe][ch].push(req);
    return true;
}

// ---------------------------------------------------------------------------
// routeBatch — atomic batch route. Succeeds only if ALL fragments fit
// in their respective per-channel queues.
// ---------------------------------------------------------------------------
bool Xbar::routeBatch(int src_pe, const vector<ReqEntry>& batch)
{
    if (src_pe < 0 || src_pe >= 4) return false;
    // Atomic: all per-channel queues must have room
    for (auto& req : batch) {
        int ch = req.channel;
        if (ch < 0 || ch >= NUM_CHANNELS) return false;
        if (static_cast<int>(m_pe_buf[src_pe][ch].size()) >= m_max_qdepth)
            return false;
    }
    // Push each fragment to its per-channel queue
    for (auto& req : batch)
        m_pe_buf[src_pe][req.channel].push(req);
    return true;
}

// ---------------------------------------------------------------------------
// resetArbitrate — SC_METHOD at reset: clear all PE buffers and RR ptrs
// ---------------------------------------------------------------------------
void Xbar::resetArbitrate()
{
    if (!reset.read()) return;
    for (int p = 0; p < 4; ++p)
        for (int ch = 0; ch < NUM_CHANNELS; ++ch)
            while (!m_pe_buf[p][ch].empty()) m_pe_buf[p][ch].pop();
    for (int ch = 0; ch < NUM_CHANNELS; ++ch)
        m_rr_ptr[ch] = 0;
}

// ---------------------------------------------------------------------------
// arbitrateAll — single SC_METHOD: loop over all channels on each clock edge
// ---------------------------------------------------------------------------
void Xbar::arbitrateAll()
{
    for (int ch = 0; ch < NUM_CHANNELS; ++ch)
        arbitrateOne(ch);
}

// ---------------------------------------------------------------------------
// arbitrateOne — per-channel RR among 4 per-channel PE queues
// ---------------------------------------------------------------------------
void Xbar::arbitrateOne(int ch)
{
    for (int a = 0; a < 4; ++a) {
        int pe = (m_rr_ptr[ch] + a) % 4;

        if (m_pe_buf[pe][ch].empty()) continue;
        ReqEntry& front = m_pe_buf[pe][ch].front();

        if (!m_sched[ch]->enqueue(pe, front)) continue;
        m_sched[ch]->notifyReq();
        m_pe_buf[pe][ch].pop();
        m_rr_ptr[ch] = (pe + 1) % 4;
        m_sig_routed[ch].write(m_sig_routed[ch].read() + 1);
        m_sig_rr[ch].write(m_rr_ptr[ch]);
        break;
    }
}

void Xbar::traceAll(sc_core::sc_trace_file* tf) const
{
    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
        string p = string(name()) + ".ch" + to_string(ch);
        sc_core::sc_trace(tf, m_sig_routed[ch], p + "_routed");
        sc_core::sc_trace(tf, m_sig_rr[ch], p + "_rr");
    }
}
