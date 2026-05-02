// ============================================================================
// dram_channel.cpp — DRAM channel: NoC → DRAMSys AT protocol
// ============================================================================
// Uses tagged simple_initiator_socket with tag = channel index.
// The tag becomes the thread ID in DRAMSys's Arbiter.
//
// AT protocol flow (same as DRAMSys RequestIssuer):
//   1. Heap-alloc tlm_generic_payload + set_mm(&m_mm)
//   2. m_ini->nb_transport_fw(BEGIN_REQ, tag) — DRAMSys internally acquire()
//   3. m_transportDone.wait() — block until response
//   4. nb_transport_bw(tag, BEGIN_RESP) → send END_RESP, post semaphore
//   5. DRAMSys PEQ handles END_RESP: release() → SimpleMM::free (no-op)
//   6. process() wakes up, delete trans
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
{
    // Register backward callback with tag = channel index
    m_ini.register_nb_transport_bw(this, &DramChannel::nb_transport_bw, m_tag);
    SC_THREAD(process);
    cout << "  [DramChannel " << channel << "] Created (AT tagged, tag="
         << m_tag << ", direct to DRAMSys::tSocket)" << endl;
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
        MemTransaction* tx = nullptr;

        if (m_xbar->popOutput(m_channel, tx)) {

            // Heap-allocate payload with memory manager for AT protocol
            auto* trans = new tlm_generic_payload();
            trans->set_mm(&m_mm);

            trans->set_command(tx->is_write ? TLM_WRITE_COMMAND : TLM_READ_COMMAND);
            trans->set_address(tx->address);
            trans->set_data_ptr(reinterpret_cast<unsigned char*>(tx->data));
            trans->set_data_length(tx->data_len);
            trans->set_byte_enable_ptr(nullptr);
            trans->set_byte_enable_length(0);
            trans->set_dmi_allowed(false);

            // AT forward path — tag = channel → Arbiter thread ID
            tlm_phase phase = BEGIN_REQ;
            sc_time delay = SC_ZERO_TIME;
            m_transactionPending = true;

            tlm_sync_enum sync = m_ini->nb_transport_fw(*trans, phase, delay);

            if (sync == TLM_ACCEPTED) {
                // Standard AT path: wait for nb_transport_bw callback
                m_transportDone.wait();
            }

            m_transactionPending = false;

            if (trans->is_response_error()) {
                cerr << "  [DramChannel " << m_channel
                     << "] ERROR: transaction failed" << endl;
            }

            delete trans;

            m_completed++;
            m_bytes += tx->data_len;
            delete tx;

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
        tlm_phase end_phase = END_RESP;
        sc_time end_delay = SC_ZERO_TIME;
        m_ini->nb_transport_fw(trans, end_phase, end_delay);
        m_transportDone.post();
        return TLM_COMPLETED;
    }
    return TLM_ACCEPTED;
}
