// ============================================================================
// dram_pe.cpp — DRAM interface: pull ReqEntry from scheduler → TLM to DRAMSys
// ============================================================================
#include "dram_pe.h"
#include <tlm_utils/multi_passthrough_target_socket.h>
#include "DRAMSys/DRAMSys.h"
#include <iostream>
#include <cstring>

using ArbiterSocket = tlm_utils::multi_passthrough_target_socket_optional<DRAMSys::Arbiter, 32>;

using namespace std;
using namespace sc_core;
using namespace tlm;

// ---------------------------------------------------------------------------
DramPE::DramPE(sc_module_name name, int channel, ChannelScheduler* sched,
               double clock_period)
    : sc_module(name)
    , m_channel(channel)
    , m_tag(0)
    , m_completed(0)
    , m_bytes(0)
    , m_scheduler(sched)
    , m_clockPeriod(clock_period)
    , m_mm(new SimpleMM())
{
    m_ini.register_nb_transport_bw(this, &DramPE::nb_transport_bw, m_tag);
    SC_THREAD(process);
    sensitive << m_scheduler->reqEvent();
    sensitive << m_pendingSlot;
}

void DramPE::bindToDramsys(void* tSocketPtr, int tag)
{
    m_tag = tag;
    auto& tSocket = *static_cast<ArbiterSocket*>(tSocketPtr);
    m_ini.bind(tSocket);
}

// ---------------------------------------------------------------------------
// process — SC_THREAD: pull from scheduler, send TLM to DRAMSys
// ---------------------------------------------------------------------------
void DramPE::process()
{
    while (true) {
        // Backpressure first
        if (static_cast<int>(m_pending.size()) >= m_maxInFlight) {
            wait(m_pendingSlot);
            continue;
        }

        ReqEntry req;
        if (!m_scheduler->dequeue(req)) {
            wait(m_scheduler->reqEvent() | m_pendingSlot);
            continue;
        }

        // Build TLM payload
        auto* trans = new tlm_generic_payload();
        trans->set_mm(m_mm);

        int data_len = 256;  // fixed 256B transactions
        unsigned char* data_ptr = new unsigned char[data_len]();
        if (req.is_write) {
            memcpy(data_ptr, req.data, sizeof(req.data));
            trans->set_command(TLM_WRITE_COMMAND);
        } else {
            trans->set_command(TLM_READ_COMMAND);
        }
        trans->set_data_ptr(data_ptr);

        // Translate address for DRAMSys
        uint64_t low = req.address & 0x1FFFFFFFULL;
        uint64_t dramsys_addr = (low & ~(0xFULL << 12))
                              | (static_cast<uint64_t>(m_channel) << 12);
        trans->set_address(dramsys_addr);

        // VCD: request trace
        m_sig_req_addr.write(dramsys_addr);
        m_sig_req_row.write((dramsys_addr >> 20) & 0x3FFF);
        m_sig_req_bank.write(static_cast<int>((dramsys_addr >> 18) & 0x3));
        m_sig_req_bg.write(static_cast<int>((dramsys_addr >> 16) & 0x3));
        m_sig_req_cmd.write(req.is_write ? 0 : 1);

        trans->set_data_length(data_len);
        trans->set_byte_enable_ptr(nullptr);
        trans->set_byte_enable_length(0);
        trans->set_dmi_allowed(false);

        // AT: send BEGIN_REQ (1 cycle latency, derived from clock)
        tlm_phase phase = BEGIN_REQ;
        sc_time delay = sc_time(m_clockPeriod, SC_NS);
        tlm_sync_enum status = m_ini->nb_transport_fw(*trans, phase, delay);

        if (status == TLM_ACCEPTED && phase == BEGIN_REQ)
            wait(m_endReqEvent);

        m_pending.push_back({trans});
        m_sig_pending.write(static_cast<int>(m_pending.size()));
    }
}

// ---------------------------------------------------------------------------
// nb_transport_bw — TLM backward path
// ---------------------------------------------------------------------------
tlm_sync_enum DramPE::nb_transport_bw(int /*tag*/,
    tlm_generic_payload& trans, tlm_phase& phase, sc_time& delay)
{
    if (phase == END_REQ) {
        m_endReqEvent.notify(SC_ZERO_TIME);
        return TLM_ACCEPTED;
    }

    if (phase == BEGIN_RESP) {
        tlm_phase end_phase = END_RESP;
        sc_time end_delay = SC_ZERO_TIME;
        m_ini->nb_transport_fw(trans, end_phase, end_delay);

        if (!m_pending.empty()) {
            m_completed++;
            m_bytes += trans.get_data_length();
            // VCD: response trace
            uint64_t resp_addr = trans.get_address();
            m_sig_resp_addr.write(resp_addr);
            m_sig_resp_row.write((resp_addr >> 20) & 0x3FFF);
            m_sig_resp_bank.write(static_cast<int>((resp_addr >> 18) & 0x3));
            m_sig_resp_bg.write(static_cast<int>((resp_addr >> 16) & 0x3));
            m_sig_resp_cmd.write(trans.get_command() == TLM_WRITE_COMMAND ? 0 : 1);

            delete[] trans.get_data_ptr();
            m_pending.pop_front();
            m_sig_completed.write(m_completed);
            m_sig_bytes.write(m_bytes);
            m_sig_pending.write(static_cast<int>(m_pending.size()));
        }

        m_pendingSlot.notify(SC_ZERO_TIME);
        return TLM_COMPLETED;
    }

    return TLM_ACCEPTED;
}

// ---------------------------------------------------------------------------
// traceAll
// ---------------------------------------------------------------------------
void DramPE::traceAll(sc_core::sc_trace_file* tf) const
{
    sc_core::sc_trace(tf, m_sig_pending, m_sig_pending.name());
    sc_core::sc_trace(tf, m_sig_completed, m_sig_completed.name());
    sc_core::sc_trace(tf, m_sig_bytes, m_sig_bytes.name());
    sc_core::sc_trace(tf, m_sig_req_addr, m_sig_req_addr.name());
    sc_core::sc_trace(tf, m_sig_req_row, m_sig_req_row.name());
    sc_core::sc_trace(tf, m_sig_req_bank, m_sig_req_bank.name());
    sc_core::sc_trace(tf, m_sig_req_bg, m_sig_req_bg.name());
    sc_core::sc_trace(tf, m_sig_req_cmd, m_sig_req_cmd.name());
    sc_core::sc_trace(tf, m_sig_resp_addr, m_sig_resp_addr.name());
    sc_core::sc_trace(tf, m_sig_resp_row, m_sig_resp_row.name());
    sc_core::sc_trace(tf, m_sig_resp_bank, m_sig_resp_bank.name());
    sc_core::sc_trace(tf, m_sig_resp_bg, m_sig_resp_bg.name());
    sc_core::sc_trace(tf, m_sig_resp_cmd, m_sig_resp_cmd.name());
}
