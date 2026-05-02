// ============================================================================
// dram_channel.cpp — DRAM channel: NoC → DRAMSys pipelined AT protocol
// ============================================================================
// Pipelined: sends transactions without waiting for responses. Responses
// arrive in FIFO order via nb_transport_bw. A pending deque tracks
// in-flight transactions; maxInFlight limits concurrency.
//
// Transaction lifecycle:
//   - process() creates trans with set_mm(m_mm), sends BEGIN_REQ
//   - Arbiter/Controller acquire/release transaction via refcount
//   - When refcount reaches 0, SystemC calls m_mm->free(trans) → delete
//   - process() only deletes MemTransaction (tx), NOT tlm_generic_payload
// ============================================================================
#include "dram_channel.h"
#include "noc_xbar.h"

#include <iostream>
#include <cstring>

using namespace std;
using namespace sc_core;
using namespace tlm;

DramChannel::DramChannel(sc_module_name name, int channel, NoCXbar* xbar)
    : sc_module(name)
    , m_channel(channel)
    , m_tag(channel)
    , m_xbar(xbar)
    , m_completed(0)
    , m_bytes(0)
    , m_mm(new SimpleMM())  // heap: outlives DramChannel for shutdown safety
{
    m_ini.register_nb_transport_bw(this, &DramChannel::nb_transport_bw, m_tag);
    SC_THREAD(process);
    cout << "  [DramChannel " << channel << "] Created (AT pipelined, tag="
         << m_tag << ", maxInFlight=" << m_maxInFlight << ")" << endl;
}

void DramChannel::bindToDramsys(
    tlm_utils::multi_passthrough_target_socket_optional<
    DRAMSys::Arbiter, 32>& tSocket, int tag)
{
    m_ini.bind(tSocket);
    cout << "  [DramChannel " << m_channel
         << "] Bound to DRAMSys::tSocket (tag=" << tag << ")" << endl;
}

void DramChannel::process()
{
    while (true) {

        // Clean up completed MemTransactions.
        // tlm_generic_payload is managed by SimpleMM::free() — never delete here.
        while (!m_doneQueue.empty()) {
            PendingTx& p = m_doneQueue.front();
            m_completed++;
            m_bytes += p.tx->data_len;
            delete p.tx;          // only delete the MemTransaction wrapper
            // p.trans is managed by SimpleMM — do NOT delete
            m_doneQueue.pop_front();
        }

        // Backpressure: don't exceed max in-flight limit
        if (static_cast<int>(m_pending.size()) >= m_maxInFlight) {
            wait(m_pendingSlot);
            continue;
        }

        MemTransaction* tx = nullptr;
        if (m_xbar->popOutput(m_channel, tx)) {

            auto* trans = new tlm_generic_payload();
            trans->set_mm(m_mm);   // SimpleMM handles deletion via refcount

            trans->set_command(tx->is_write ? TLM_WRITE_COMMAND : TLM_READ_COMMAND);
            trans->set_address(tx->address);
            trans->set_data_ptr(reinterpret_cast<unsigned char*>(tx->data));
            trans->set_data_length(tx->data_len);
            trans->set_byte_enable_ptr(nullptr);
            trans->set_byte_enable_length(0);
            trans->set_dmi_allowed(false);

            tlm_phase phase = BEGIN_REQ;
            sc_time delay = SC_ZERO_TIME;
            m_ini->nb_transport_fw(*trans, phase, delay);

            m_pending.push_back({trans, tx});

        } else {
            wait(sc_time(1, SC_NS));
        }
    }
}

tlm_sync_enum DramChannel::nb_transport_bw(int /*tag*/,
                                            tlm_generic_payload& trans,
                                            tlm_phase& phase,
                                            sc_time& /*delay*/)
{
    if (phase == END_REQ) {
        return TLM_ACCEPTED;
    }
    else if (phase == BEGIN_RESP) {
        // Send END_RESP back (release will be called by Arbiter PEQ)
        tlm_phase end_phase = END_RESP;
        sc_time end_delay = SC_ZERO_TIME;
        m_ini->nb_transport_fw(trans, end_phase, end_delay);

        // Move from pending to doneQueue for MemTransaction cleanup.
        // tlm_generic_payload deletion happens via SimpleMM::free()
        // when SystemC refcount reaches 0 after END_RESP processing.
        if (!m_pending.empty()) {
            m_doneQueue.push_back(m_pending.front());
            m_pending.pop_front();
            m_pendingSlot.notify(SC_ZERO_TIME);
        }

        return TLM_COMPLETED;
    }
    return TLM_ACCEPTED;
}
