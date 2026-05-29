// ============================================================================
// xbar_4x8.h — 4×8 crossbar: per-cycle FCFS per channel
// ============================================================================
#ifndef XBAR_4X8_H
#define XBAR_4X8_H

#include <systemc.h>
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
    void clearBusy();

    ChannelScheduler* m_sched[8] = {};
    bool m_busy[8];
    sc_signal<int> m_sig_routed[8];
};

#endif
