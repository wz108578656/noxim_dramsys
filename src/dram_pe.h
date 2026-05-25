// ============================================================================
// dram_pe.h — DRAM interface PE: ABP → TLM bridge to DRAMSys
// ============================================================================
#ifndef DRAM_PE_H
#define DRAM_PE_H

#include <systemc.h>
#include <tlm.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <deque>
#include <queue>
#include "DataStructs.h"
#include "Utils.h"

namespace DRAMSys { class Arbiter; }

class SimpleMM : public tlm::tlm_mm_interface
{
public:
    void free(tlm::tlm_generic_payload* p) override { delete p; }
};

struct PendingTx {
    tlm::tlm_generic_payload* trans;
};

// A complete packet received from NoC, ready to dispatch to DRAMSys
struct RxPacket {
    uint64_t address;
    bool     is_write;
    int      tag;
    uint32_t data[128];  // up to 512 bytes
    int      data_len;  // in bytes
};

SC_MODULE(DramPE)
{
public:
    SC_HAS_PROCESS(DramPE);

    tlm_utils::simple_initiator_socket_tagged<DramPE, 32> m_ini{"ini"};

    // ABP ports
    sc_in_clk   clock;
    sc_in<bool> reset;
    sc_in<Flit>              flit_rx;
    sc_in<bool>              req_rx;
    sc_out<bool>             ack_rx;
    sc_out<TBufferFullStatus> buffer_full_status_rx;

    DramPE(sc_module_name name, int channel);

    void bindToDramsys(void* tSocket, int tag);

    uint64_t completed() const { return m_completed; }
    uint64_t bytesTransferred() const { return m_bytes; }
    bool hasPending() const { return !m_pending.empty(); }
    int  channel() const { return m_channel; }

private:
    void process();    // SC_THREAD: dispatch packets to DRAMSys
    void rxProcess();  // SC_METHOD: receive flits, assemble RxPacket

    tlm::tlm_sync_enum nb_transport_bw(int tag,
        tlm::tlm_generic_payload& trans,
        tlm::tlm_phase& phase, sc_core::sc_time& delay);

    int  m_channel;
    int  m_tag;
    int  m_maxInFlight = 8;   // DRAM pipeline depth; backpressure when full
    uint64_t m_completed;
    uint64_t m_bytes;

    SimpleMM* m_mm;
    std::deque<PendingTx> m_pending;
    sc_core::sc_event m_pendingSlot;

    // Rx assembly state
    Flit m_rx_buf[5];   // HEAD + up to 3×BODY + TAIL (max 512B)
    int  m_rx_len;
    int  m_rx_seq;

    // Queue of fully-assembled packets ready for TLM dispatch
    std::queue<RxPacket> m_rx_pkts;
    sc_core::sc_event m_rxPktEvent;

    // ABP
    bool m_current_level_rx;

public:
    // VCD trace
    void traceAll(sc_core::sc_trace_file* tf) const;

private:
    sc_signal<int>      m_sig_pending{"pending"};
    sc_signal<uint64_t> m_sig_completed{"completed"};
    sc_signal<uint64_t> m_sig_bytes{"bytes"};
    sc_signal<int>      m_sig_rx_seq{"rx_seq"};
};

#endif // DRAM_PE_H
