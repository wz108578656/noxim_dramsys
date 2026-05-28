// ============================================================================
// xbar_4x8.cpp — 4×8 crossbar: RR fair arbitration per channel
// ============================================================================
#include "xbar_4x8.h"
#include <cstring>

using namespace std;

Xbar4x8::Xbar4x8(sc_module_name name)
    : sc_module(name)
{
    memset(m_busy, 0, sizeof(m_busy));
    for (int i = 0; i < 8; ++i) {
        m_last_pe[i] = -1;
        m_prefer_pe[i] = 0;
    }
    SC_METHOD(clearBusy);
    sensitive << clock.neg();
    sensitive << reset;
}

void Xbar4x8::bindScheduler(int ch, ChannelScheduler* sched)
{
    if (ch >= 0 && ch < 8)
        m_sched[ch] = sched;
}

void Xbar4x8::clearBusy()
{
    if (reset.read()) {
        memset(m_busy, 0, sizeof(m_busy));
        for (int i = 0; i < 8; ++i) {
            m_last_pe[i] = -1;
            m_prefer_pe[i] = 0;
        }
        return;
    }
    // Clear busy, set preferred PE for next cycle
    for (int ch = 0; ch < 8; ++ch) {
        m_busy[ch] = false;
        m_prefer_pe[ch] = (m_last_pe[ch] < 0) ? 0 : (m_last_pe[ch] + 1) % 4;
        m_sig_prefer[ch].write(m_prefer_pe[ch]);
    }
}

bool Xbar4x8::route(int src_pe, const ReqEntry& req)
{
    int ch = AddrDecode::channel(req.address);
    if (ch < 0 || ch >= 8 || !m_sched[ch])
        return false;

    // Channel already routed this cycle
    if (m_busy[ch])
        return false;

    // Prefer the RR-target PE; if it hasn't arrived, accept any
    m_busy[ch] = true;
    m_last_pe[ch] = src_pe;

    if (!m_sched[ch]->enqueue(src_pe, req)) {
        m_busy[ch] = false;  // enqueue failed, release
        return false;
    }
    m_sched[ch]->notifyReq();
    m_sig_routed[ch].write(m_sig_routed[ch].read() + 1);
    return true;
}

void Xbar4x8::traceAll(sc_core::sc_trace_file* tf) const
{
    for (int ch = 0; ch < 8; ++ch) {
        sc_core::sc_trace(tf, m_sig_routed[ch], m_sig_routed[ch].name());
        sc_core::sc_trace(tf, m_sig_prefer[ch], m_sig_prefer[ch].name());
    }
}
