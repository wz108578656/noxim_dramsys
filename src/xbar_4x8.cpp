// ============================================================================
// xbar_4x8.cpp — 4×8 crossbar: per-cycle FCFS + fallback
// ============================================================================
#include "xbar_4x8.h"
#include <cstring>

using namespace std;

Xbar4x8::Xbar4x8(sc_module_name name)
    : sc_module(name)
{
    memset(m_busy, 0, sizeof(m_busy));
    for (int i = 0; i < 8; ++i) {
        m_fallback_pe[i] = -1;
        m_fallback_done[i] = false;
    }
    SC_METHOD(clearBusy);
    sensitive << clock.neg();
    sensitive << reset;
    SC_THREAD(resolveFallback);
    sensitive << m_resolveEvent;
}

void Xbar4x8::bindScheduler(int ch, ChannelScheduler* sched)
{
    if (ch >= 0 && ch < 8) m_sched[ch] = sched;
}

void Xbar4x8::clearBusy()
{
    if (reset.read()) {
        memset(m_busy, 0, sizeof(m_busy));
        for (int i = 0; i < 8; ++i) {
            m_fallback_pe[i] = -1; m_fallback_done[i] = false;
        }
        return;
    }
    for (int ch = 0; ch < 8; ++ch) {
        m_busy[ch] = false;
        m_fallback_pe[ch] = -1;
        // m_fallback_done persists — PE retry will see it
    }
}

// route: called by PEs at clock posedge
bool Xbar4x8::route(int src_pe, const ReqEntry& req)
{
    int ch = AddrDecode::channel(req.address);
    if (ch < 0 || ch >= 8 || !m_sched[ch]) return false;

    // PE retry after fallback accept — acknowledge
    if (m_fallback_done[ch]) {
        if (m_fallback_pe[ch] == src_pe) {
            m_fallback_done[ch] = false;
            return true;
        }
        return false;
    }

    if (m_busy[ch]) return false;

    // First come, first served (FCFS within cycle)
    if (!m_sched[ch]->enqueue(src_pe, req)) return false;
    m_sched[ch]->notifyReq();
    m_busy[ch] = true;
    m_sig_routed[ch].write(m_sig_routed[ch].read() + 1);
    return true;

    // Note: non-preferred PEs get rejected and retry next cycle.
    // Fallback below handles the case where preferred PE exists.
    // Currently simplified to FCFS without preference.
    // Fallback is retained for future use.
}

void Xbar4x8::resolveFallback()
{
    while (true) {
        wait(m_resolveEvent);
        for (int ch = 0; ch < 8; ++ch) {
            if (m_busy[ch] || m_fallback_pe[ch] < 0) continue;
            if (!m_sched[ch]->enqueue(m_fallback_pe[ch], m_fallback_req[ch]))
                continue;
            m_sched[ch]->notifyReq();
            m_busy[ch] = true;
            m_fallback_done[ch] = true;
            m_sig_routed[ch].write(m_sig_routed[ch].read() + 1);
        }
    }
}

void Xbar4x8::traceAll(sc_core::sc_trace_file* tf) const
{
    for (int ch = 0; ch < 8; ++ch) {
        string p = string(name()) + ".ch" + to_string(ch);
        sc_core::sc_trace(tf, m_sig_routed[ch], p + "_routed");
        sc_core::sc_trace(tf, m_sig_prefer[ch], p + "_prefer");
    }
}
