// ============================================================================
// xbar_4x8.h — 4×8 crossbar: PE queue (depth 2) + clock-edge RR arbiter
// ============================================================================
#ifndef XBAR_4X8_H
#define XBAR_4X8_H

#include <systemc.h>
#include <queue>
#include "channel_scheduler.h"

SC_MODULE(Xbar4x8)
{
public:
    SC_HAS_PROCESS(Xbar4x8);

    sc_in_clk   clock;
    sc_in<bool> reset;

    Xbar4x8(sc_module_name name);

    bool route(int src_pe, const ReqEntry& req);
    void bindScheduler(int ch, ChannelScheduler* sched);
    void traceAll(sc_core::sc_trace_file* tf) const;

private:
    void resetArbitrate();

    // 8 independent per-channel SC_METHODs (sensitive to clock.pos())
    void arbitrateCh0(); void arbitrateCh1(); void arbitrateCh2(); void arbitrateCh3();
    void arbitrateCh4(); void arbitrateCh5(); void arbitrateCh6(); void arbitrateCh7();
    void arbitrateOne(int ch);

    ChannelScheduler* m_sched[8] = {};
    std::queue<ReqEntry> m_pe_buf[4];
    int  m_max_qdepth = 2;
    int  m_rr_ptr[8] = {};

    sc_signal<int> m_sig_routed[8];
    sc_signal<int> m_sig_rr[8];
};

#endif
