// ============================================================================
// xbar_4x8.cpp — 4×8 crossbar: PE queue + per-channel clock-edge RR arbiter
// ============================================================================
#include "xbar_4x8.h"
#include <cstring>

using namespace std;

Xbar4x8::Xbar4x8(sc_module_name name)
    : sc_module(name)
{
    SC_METHOD(resetArbitrate); sensitive << reset;

    SC_METHOD(arbitrateCh0); sensitive << clock.pos();
    SC_METHOD(arbitrateCh1); sensitive << clock.pos();
    SC_METHOD(arbitrateCh2); sensitive << clock.pos();
    SC_METHOD(arbitrateCh3); sensitive << clock.pos();
    SC_METHOD(arbitrateCh4); sensitive << clock.pos();
    SC_METHOD(arbitrateCh5); sensitive << clock.pos();
    SC_METHOD(arbitrateCh6); sensitive << clock.pos();
    SC_METHOD(arbitrateCh7); sensitive << clock.pos();
}

void Xbar4x8::bindScheduler(int ch, ChannelScheduler* sched)
{
    if (ch >= 0 && ch < 8) m_sched[ch] = sched;
}

// ---------------------------------------------------------------------------
// route — called by PE. Write to per-PE queue if depth allows.
// ---------------------------------------------------------------------------
bool Xbar4x8::route(int src_pe, const ReqEntry& req)
{
    if (src_pe < 0 || src_pe >= 4) return false;
    if (static_cast<int>(m_pe_buf[src_pe].size()) >= m_max_qdepth)
        return false;
    m_pe_buf[src_pe].push(req);
    return true;
}

// ---------------------------------------------------------------------------
// resetArbitrate — SC_METHOD at reset: clear all PE buffers and RR ptrs
// ---------------------------------------------------------------------------
void Xbar4x8::resetArbitrate()
{
    if (!reset.read()) return;
    for (int p = 0; p < 4; ++p)
        while (!m_pe_buf[p].empty()) m_pe_buf[p].pop();
    for (int ch = 0; ch < 8; ++ch)
        m_rr_ptr[ch] = 0;
}

// ---------------------------------------------------------------------------
// arbitrateOne — per-channel RR among 4 PE queues
// ---------------------------------------------------------------------------
void Xbar4x8::arbitrateOne(int ch)
{
    for (int a = 0; a < 4; ++a) {
        int pe = (m_rr_ptr[ch] + a) % 4;

        if (m_pe_buf[pe].empty()) continue;
        ReqEntry& front = m_pe_buf[pe].front();
        if (front.channel != ch) continue;

        if (!m_sched[ch]->enqueue(pe, front)) continue;
        m_sched[ch]->notifyReq();
        m_pe_buf[pe].pop();
        m_rr_ptr[ch] = (pe + 1) % 4;
        m_sig_routed[ch].write(m_sig_routed[ch].read() + 1);
        m_sig_rr[ch].write(m_rr_ptr[ch]);
        break;
    }
}

// ---------------------------------------------------------------------------
// 8 independent per-channel arbitrate methods
// ---------------------------------------------------------------------------
void Xbar4x8::arbitrateCh0() { arbitrateOne(0); }
void Xbar4x8::arbitrateCh1() { arbitrateOne(1); }
void Xbar4x8::arbitrateCh2() { arbitrateOne(2); }
void Xbar4x8::arbitrateCh3() { arbitrateOne(3); }
void Xbar4x8::arbitrateCh4() { arbitrateOne(4); }
void Xbar4x8::arbitrateCh5() { arbitrateOne(5); }
void Xbar4x8::arbitrateCh6() { arbitrateOne(6); }
void Xbar4x8::arbitrateCh7() { arbitrateOne(7); }

void Xbar4x8::traceAll(sc_core::sc_trace_file* tf) const
{
    for (int ch = 0; ch < 8; ++ch) {
        string p = string(name()) + ".ch" + to_string(ch);
        sc_core::sc_trace(tf, m_sig_routed[ch], p + "_routed");
        sc_core::sc_trace(tf, m_sig_rr[ch], p + "_rr");
    }
}
