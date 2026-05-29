// ============================================================================
// xbar_4x8.cpp — 4×8 crossbar: per-cycle FCFS per channel
// ============================================================================
#include "xbar_4x8.h"
#include <cstring>

using namespace std;

Xbar4x8::Xbar4x8(sc_module_name name)
    : sc_module(name)
{
    memset(m_busy, 0, sizeof(m_busy));
    SC_METHOD(clearBusy);
    sensitive << clock.neg();
    sensitive << reset;
}

void Xbar4x8::bindScheduler(int ch, ChannelScheduler* sched)
{
    if (ch >= 0 && ch < 8) m_sched[ch] = sched;
}

void Xbar4x8::clearBusy()
{
    if (reset.read()) { memset(m_busy, 0, sizeof(m_busy)); return; }
    for (int ch = 0; ch < 8; ++ch) m_busy[ch] = false;
}

bool Xbar4x8::route(int src_pe, const ReqEntry& req)
{
    int ch = AddrDecode::channel(req.address);
    if (ch < 0 || ch >= 8 || !m_sched[ch]) return false;
    if (m_busy[ch]) return false;
    if (!m_sched[ch]->enqueue(src_pe, req)) return false;
    m_sched[ch]->notifyReq();
    m_busy[ch] = true;
    m_sig_routed[ch].write(m_sig_routed[ch].read() + 1);
    return true;
}

void Xbar4x8::traceAll(sc_core::sc_trace_file* tf) const
{
    for (int ch = 0; ch < 8; ++ch) {
        string p = string(name()) + ".ch" + to_string(ch);
        sc_core::sc_trace(tf, m_sig_routed[ch], p + "_routed");
    }
}
