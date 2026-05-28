// ============================================================================
// xbar_4x8.h — 4×8 crossbar: routes PE requests to channel schedulers
// ============================================================================
#ifndef XBAR_4X8_H
#define XBAR_4X8_H

#include <systemc.h>
#include "channel_scheduler.h"

class TrafficPE;

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
    void clearBusy();  // SC_METHOD: reset busy flags each cycle

    ChannelScheduler* m_sched[8] = {};
    bool m_busy[8];
    int  m_last_pe[8];    // last PE served per channel
    int  m_prefer_pe[8];  // preferred PE this cycle = (last_pe+1)%4
    sc_signal<int> m_sig_routed[8];
    sc_signal<int> m_sig_prefer[8];
};

#endif // XBAR_4X8_H
