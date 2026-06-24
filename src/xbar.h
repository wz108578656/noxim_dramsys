// ============================================================================
// xbar.h — 4×N crossbar: PE queue (depth 2) + clock-edge RR arbiter
// ============================================================================
#ifndef XBAR_H
#define XBAR_H

#include <systemc.h>
#include <queue>
#include <vector>
#include "channel_scheduler.h"

SC_MODULE(Xbar)
{
public:
    SC_HAS_PROCESS(Xbar);

    sc_in_clk   clock;
    sc_in<bool> reset;

    Xbar(sc_module_name name);

    bool route(int src_pe, const ReqEntry& req);
    bool routeBatch(int src_pe, const std::vector<ReqEntry>& batch);
    void bindScheduler(int ch, ChannelScheduler* sched);
    void traceAll(sc_core::sc_trace_file* tf) const;

private:
    void resetArbitrate();
    void arbitrateAll();  // single SC_METHOD looping over all channels
    void arbitrateOne(int ch);

    ChannelScheduler* m_sched[NUM_CHANNELS] = {};
    std::queue<ReqEntry> m_pe_buf[4][NUM_CHANNELS];
    int  m_max_qdepth = 16;
    int  m_rr_ptr[NUM_CHANNELS] = {};

    sc_signal<int> m_sig_routed[NUM_CHANNELS];
    sc_signal<int> m_sig_rr[NUM_CHANNELS];
};

#endif
