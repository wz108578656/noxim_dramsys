// ============================================================================
// dram_pe.h — DRAM interface: ReqEntry → TLM bridge to DRAMSys
// ============================================================================
#ifndef DRAM_PE_H
#define DRAM_PE_H

#include <systemc.h>
#include <tlm.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <deque>
#include "channel_scheduler.h"

class SimpleMM : public tlm::tlm_mm_interface
{
public:
    void free(tlm::tlm_generic_payload* p) override { delete p; }
};

struct PendingTx {
    tlm::tlm_generic_payload* trans;
};

class ChannelScheduler;  // forward decl

SC_MODULE(DramPE)
{
public:
    SC_HAS_PROCESS(DramPE);

    tlm_utils::simple_initiator_socket_tagged<DramPE, 32> m_ini{"ini"};

    // Clock & reset only (ABP removed, uses direct ReqEntry from scheduler)
    sc_in_clk   clock;
    sc_in<bool> reset;

    DramPE(sc_module_name name, int channel, ChannelScheduler* sched,
           double clock_period = 1.0);

    void bindToDramsys(void* tSocket, int tag);

    uint64_t completed() const { return m_completed; }
    uint64_t bytesTransferred() const { return m_bytes; }
    bool hasPending() const { return !m_pending.empty(); }
    int  channel() const { return m_channel; }

private:
    void process();    // SC_THREAD: pull from scheduler → TLM

    tlm::tlm_sync_enum nb_transport_bw(int tag,
        tlm::tlm_generic_payload& trans,
        tlm::tlm_phase& phase, sc_core::sc_time& delay);

    int  m_channel;
    int  m_tag;
    int  m_maxInFlight = 64;
    uint64_t m_completed;
    uint64_t m_bytes;

    ChannelScheduler* m_scheduler;
    double m_clockPeriod;

    SimpleMM* m_mm;
    std::deque<PendingTx> m_pending;
    sc_core::sc_event m_pendingSlot;
    sc_core::sc_event m_endReqEvent;

public:
    // VCD trace
    void traceAll(sc_core::sc_trace_file* tf) const;

private:
    sc_signal<int>      m_sig_pending{"pending"};
    sc_signal<uint64_t> m_sig_completed{"completed"};
    sc_signal<uint64_t> m_sig_bytes{"bytes"};
    sc_signal<uint64_t> m_sig_req_addr{"req_addr"};
    sc_signal<uint64_t> m_sig_req_row{"req_row"};
    sc_signal<int>      m_sig_req_bank{"req_bank"};
    sc_signal<int>      m_sig_req_bg{"req_bg"};
    sc_signal<int>      m_sig_req_cmd{"req_cmd"};
    sc_signal<uint64_t> m_sig_resp_addr{"resp_addr"};
    sc_signal<uint64_t> m_sig_resp_row{"resp_row"};
    sc_signal<int>      m_sig_resp_bank{"resp_bank"};
    sc_signal<int>      m_sig_resp_bg{"resp_bg"};
    sc_signal<int>      m_sig_resp_cmd{"resp_cmd"};
};

#endif // DRAM_PE_H
