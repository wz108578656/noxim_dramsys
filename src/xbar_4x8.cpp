// ============================================================================
// xbar_4x8.cpp — 4×8 crossbar implementation
// ============================================================================
#include "xbar_4x8.h"
#include "traffic_pe.h"

using namespace std;

Xbar4x8::Xbar4x8(sc_module_name name)
    : sc_module(name)
{
}

void Xbar4x8::bindScheduler(int ch, ChannelScheduler* sched)
{
    if (ch >= 0 && ch < 8)
        m_sched[ch] = sched;
}

bool Xbar4x8::route(int src_pe, const ReqEntry& req)
{
    int ch = AddrDecode::channel(req.address);
    if (ch < 0 || ch >= 8 || !m_sched[ch])
        return false;

    // Check backpressure: if queue full, return false
    // Simple backpressure via checking all 4 queues... use a heuristic
    // For now, always accept (bounded by scheduler's queue depth)
    m_sched[ch]->enqueue(src_pe, req);
    m_sched[ch]->notifyReq();  // wakes DramPE to dequeue
    m_sig_routed[ch].write(m_sig_routed[ch].read() + 1);
    return true;
}

void Xbar4x8::traceAll(sc_core::sc_trace_file* tf) const
{
    for (int ch = 0; ch < 8; ++ch)
        sc_core::sc_trace(tf, m_sig_routed[ch], m_sig_routed[ch].name());
}
