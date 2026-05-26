// ============================================================================
// xbar_4x8.h — 4×8 crossbar: routes PE requests to channel schedulers
// ============================================================================
#ifndef XBAR_4X8_H
#define XBAR_4X8_H

#include <systemc.h>
#include "channel_scheduler.h"

class TrafficPE;  // forward decl

SC_MODULE(Xbar4x8)
{
public:
    Xbar4x8(sc_module_name name);

    // Bind a PE to a crossbar input port
    void bindPE(int port, TrafficPE* pe);

    // Route a request: decode channel → enqueue to scheduler
    // Returns false if scheduler queue is full (backpressure)
    bool route(int src_pe, const ReqEntry& req);

    // Bind a channel scheduler to an output
    void bindScheduler(int ch, ChannelScheduler* sched);

    // VCD trace
    void traceAll(sc_core::sc_trace_file* tf) const;

private:
    ChannelScheduler* m_sched[8] = {};
    sc_signal<int> m_sig_routed[8];  // per-channel route count
};

#endif // XBAR_4X8_H
